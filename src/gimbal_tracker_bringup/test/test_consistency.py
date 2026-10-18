"""全工程静态一致性回归（**纯文本/AST 扫描，无需 ROS2**）。

这是 T05 的"结构防线"：把 DESIGN 里那些**可机械校验**的硬约束固化成断言，
任何一次改动破坏了它们都会在这里立刻 fail。

覆盖：
  A. 全部 `.py` 语法可编译
  B. 每个 `setup.py` 的 `console_scripts` → 模块文件存在且含 `main()`
  C. `launch/` 里引用的每个 `(package, executable)` 与 entry_points 完全一致
  D. `launch/` 里 include 的其它 launch 文件真实存在
  E. `tracker_params.yaml` 的每个参数键至少被一处非测试代码引用（查拼写/悬空参数）
  F. `launch/` 里的 overrides 键必须能在代码或 YAML 中找到
  G. DESIGN §4.3 话题表里的每个话题都在代码中出现
  H. T-C8：`src/` 下（排除 `gimbal_debug`）不得出现真值话题字面量
  I. DESIGN §8.6：除 `virtual.yaml` / 测试外，不得出现相机内参字面量
  J. 所有 YAML / RViz 配置可解析
  K. DESIGN §8.2：生产代码不得真实调用墙上时钟（AST 判定，忽略注释/文档串）
  L. launch 运行期**必需**从 YAML 读到的键必须存在
  M. `static_tf` 的光学系 rpy 与 `gimbal_common.frames` 单一真源一致

⚠ 只能做**静态**校验；`colcon build`、msg 编译、实际话题连通性仍需在
Ubuntu 22.04 + ROS 2 Humble 容器中验证（流程见根 README）。
"""

from __future__ import annotations

import ast
import glob
import os
from typing import Any
from typing import Dict
from typing import List
from typing import Set
from typing import Tuple

import pytest
import yaml

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.abspath(os.path.join(TEST_DIR, '..'))
SRC_DIR = os.path.abspath(os.path.join(PKG_DIR, '..'))
WS_DIR = os.path.abspath(os.path.join(SRC_DIR, '..'))
PACKAGE_DIRS = {os.path.basename(os.path.dirname(p)): os.path.dirname(p)
                for p in glob.glob(os.path.join(SRC_DIR, '*', 'setup.py'))}
PACKAGE_DIRS['virtual_gimbal'] = os.path.join(WS_DIR, 'gimbal-edu-tools', 'virtual_gimbal')

CONFIG_DIR = os.path.join(PKG_DIR, 'config')
TRACKER_PARAMS = os.path.join(CONFIG_DIR, 'tracker_params.yaml')
VIRTUAL_YAML = os.path.join(CONFIG_DIR, 'virtual.yaml')

GT_LITERAL = '/virtual_gimbal/' + 'target' + '_gt'
FORBIDDEN_INTRINSICS = ('554.26', '319.5', '239.5')

# 本工程自己的包（检查 I / K 时用于给出可读信息）
OUR_PACKAGES = ('grid_mapper', 'chassis_advisor', 'gimbal_dataio', 'gimbal_debug',
                'gimbal_tracker_bringup')

DESIGN_TOPICS = (
    '/perception/marker_corners', '/virtual_gimbal/camera_info',
    '/virtual_gimbal/status', '/offboardlink/imu_data', '/gimbal/state',
    '/gimbal/cmd_omega', '/gimbal/base_pose_raw', '/gimbal/base_pose',
    '/gimbal/t_map_world', '/perception/marker_cam', '/estimation/target_state',
    '/map/grid', '/map/ray_grid', '/chassis/advice', '/debug/angle_error_deg',
)


# ---------------------------------------------------------------------------
# 扫描工具
# ---------------------------------------------------------------------------
def _all_py() -> List[str]:
    return sorted(glob.glob(os.path.join(SRC_DIR, '**', '*.py'), recursive=True))


def _non_test_py() -> List[str]:
    return [p for p in _all_py()
            if 'test' not in p.replace('\\', '/').split('/')]


def _rel(path: str) -> str:
    """相对 `src/` 的 POSIX 风格路径。"""
    return os.path.relpath(path, SRC_DIR).replace('\\', '/')


@pytest.fixture(scope='module')
def text_of() -> Dict[str, str]:
    """`{绝对路径: 文本}`（只读一次，供各测试复用）。"""
    out: Dict[str, str] = {}
    for p in _all_py():
        with open(p, 'r', encoding='utf-8', errors='ignore') as f:
            out[p] = f.read()
    return out


@pytest.fixture(scope='module')
def params_yaml() -> Dict[str, Any]:
    """`tracker_params.yaml` 的 `/**/ros__parameters`。"""
    with open(TRACKER_PARAMS, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data['/**']['ros__parameters']


@pytest.fixture(scope='module')
def entry_points() -> Dict[str, Tuple[str, str]]:
    """`{可执行文件名: (包目录名, 模块路径)}`。"""
    import re
    out: Dict[str, Tuple[str, str]] = {}
    for pkg_dir in sorted(PACKAGE_DIRS.values()):
        setup_py = os.path.join(pkg_dir, 'setup.py')
        with open(setup_py, 'r', encoding='utf-8') as f:
            txt = f.read()
        for m in re.finditer(r"'([A-Za-z0-9_]+)\s*=\s*([A-Za-z0-9_.]+):([A-Za-z0-9_]+)'",
                             txt):
            out[m.group(1)] = (os.path.basename(pkg_dir), m.group(2))
    return out


# ---------------------------------------------------------------------------
# A. 语法
# ---------------------------------------------------------------------------
def test_a_all_python_compiles(text_of):
    """全部 `.py` 必须能编译（catch SyntaxError / 缩进错）。"""
    bad = []
    for path, txt in text_of.items():
        try:
            compile(txt, path, 'exec')
        except SyntaxError as exc:
            bad.append('%s: %s' % (_rel(path), exc))
    assert bad == [], '存在语法错误:\n' + '\n'.join(bad)


# ---------------------------------------------------------------------------
# B. entry_points
# ---------------------------------------------------------------------------
def test_b_entry_points_resolve(entry_points):
    """每个 `console_scripts` 指向的模块文件必须存在且定义 `main()`。"""
    assert entry_points, '未解析到任何入口点'
    bad = []
    for exe, (pkg, modpath) in entry_points.items():
        mod_file = os.path.join(PACKAGE_DIRS[pkg], *modpath.split('.')) + '.py'
        if not os.path.isfile(mod_file):
            bad.append('%s -> 模块缺失 %s' % (exe, _rel(mod_file)))
            continue
        with open(mod_file, 'r', encoding='utf-8') as f:
            if 'def main(' not in f.read():
                bad.append('%s -> %s 未定义 main()' % (exe, _rel(mod_file)))
    assert bad == [], '\n'.join(bad)


# ---------------------------------------------------------------------------
# C. launch ↔ entry_points
# ---------------------------------------------------------------------------
def test_c_launch_references_match_entry_points(entry_points):
    """launch 里引用的每个 (package, executable) 必须与 entry_points 完全一致。"""
    import re
    bad = []
    launch_files = sorted(glob.glob(os.path.join(PKG_DIR, 'launch', '**', '*.py'),
                                    recursive=True))
    assert launch_files, '未找到 launch 文件'
    for lf in launch_files:
        with open(lf, 'r', encoding='utf-8') as f:
            txt = f.read()
        pairs = set()
        for m in re.finditer(r"package='([^']+)'\s*,\s*\n?\s*executable='([^']+)'", txt):
            pairs.add((m.group(1), m.group(2)))
        for m in re.finditer(r"\(\d+,\s*'([a-z_]+)',\s*'([A-Za-z0-9_]+)'\)", txt):
            pairs.add((m.group(1), m.group(2)))
        for pkg, exe in sorted(pairs):
            if exe in ('static_transform_publisher', 'rviz2'):
                continue                      # 来自 tf2_ros / rviz2，不属于本工程
            if (pkg, exe) == ('point_lio', 'pointlio_mapping'):
                continue  # original navigation package, not copied into core src
            if (pkg, exe) == ('gimbal_serial_42', 'robo_bridge_node'):
                bridge = os.path.join(SRC_DIR, '..', '42-gimbal_serial')
                with open(os.path.join(bridge, 'CMakeLists.txt'), encoding='utf-8') as f:
                    cmake = f.read()
                assert 'add_executable(robo_bridge_node src/robo_bridge_node_main.cpp)' in cmake
                assert 'install(TARGETS robo_bridge_node DESTINATION lib/${PROJECT_NAME})' in cmake
                assert os.path.isfile(os.path.join(bridge, 'src', 'robo_bridge_node_main.cpp'))
                continue  # C++节点由CMake登记，不是Python console_script。
            if exe not in entry_points:
                bad.append('%s: %s 不在任何 entry_points 中' % (_rel(lf), exe))
            elif entry_points[exe][0] != pkg:
                bad.append('%s: package=%s 与 %s 实际所属 %s 不符'
                           % (_rel(lf), pkg, exe, entry_points[exe][0]))
    assert bad == [], '\n'.join(bad)


def test_c2_launch_ladder_matches_design_order():
    """`NODE_LADDER` 的 (package, executable) 顺序必须与 DESIGN §6.14 一致。"""
    path = os.path.join(PKG_DIR, 'launch', 'include', 'nodes.launch.py')
    with open(path, 'r', encoding='utf-8') as f:
        src = f.read()
    import re
    ladder = re.findall(r"\((\d+),\s*'([a-z_]+)',\s*'([A-Za-z0-9_]+)'\)", src)
    assert ladder, '未解析到 NODE_LADDER'
    order = [(pkg, exe) for _i, pkg, exe in ladder]
    assert order[0] == ('grid_mapper', 'grid_mapper_node'), \
        'grid_mapper 必须最先启动（发 TRANSIENT_LOCAL 地图）'
    expected = [
        ('grid_mapper', 'grid_mapper_node'),
        ('gimbal_localization', 'base_pose_provider_node'),
        ('gimbal_localization', 'map_frame_broadcaster_node'),
        ('gimbal_serial_bridge', 'vgimbal_bridge_node'),
        ('gimbal_perception', 'aruco_detector_node'),
        ('gimbal_perception', 'marker_pnp_node'),
        ('gimbal_estimation', 'target_estimator_node'),
        ('gimbal_control', 'gimbal_controller_node'),
        ('chassis_advisor', 'chassis_advisor_node'),
        ('gimbal_judge_io', 'judge_io_node'),
        ('gimbal_dataio', 'trajectory_annotator_node'),
        ('gimbal_debug', 'gt_error_monitor_node'),
    ]
    assert order == expected, '启动顺序与 DESIGN §6.14 不一致: %s' % (order,)


def test_c3_no_arbitrary_process_launched():
    """禁止用 `ExecuteProcess` 启动任意进程（如 `robo_bridge_edu` 这类串口桥）。"""
    offenders = []
    for lf in glob.glob(os.path.join(PKG_DIR, 'launch', '**', '*.py'), recursive=True):
        with open(lf, 'r', encoding='utf-8') as f:
            txt = f.read()
        if 'ExecuteProcess' in txt:
            offenders.append(_rel(lf))
    assert offenders == [], 'launch 不得直接执行任意进程: %s' % offenders


# ---------------------------------------------------------------------------
# D. launch include 路径
# ---------------------------------------------------------------------------
def test_d_included_launch_files_exist():
    """launch 里 include 的 `*.launch.py` 在 share 目录下必须存在。"""
    import re
    bad = []
    for lf in glob.glob(os.path.join(PKG_DIR, 'launch', '**', '*.py'), recursive=True):
        with open(lf, 'r', encoding='utf-8') as f:
            txt = f.read()
        for name in re.findall(r"(?:'|\")([a-z_]+\.launch\.py)(?:'|\")", txt):
            candidates = [
                os.path.join(os.path.dirname(lf), name),
                os.path.join(PKG_DIR, 'launch', name),
                os.path.join(PKG_DIR, 'launch', 'include', name),
                os.path.join(SRC_DIR, '..', '42-gimbal_serial', 'launch', name),
            ]
            if not any(os.path.isfile(c) for c in candidates):
                bad.append('%s 引用了不存在的 %s' % (_rel(lf), name))
    assert bad == [], '\n'.join(bad)


def test_d2_share_dir_uses_get_package_share_directory():
    """launch 定位配置一律走 `get_package_share_directory`（禁止相对/当前目录拼接）。"""
    for lf in glob.glob(os.path.join(PKG_DIR, 'launch', '**', '*.py'), recursive=True):
        with open(lf, 'r', encoding='utf-8') as f:
            txt = f.read()
        if 'config' not in txt and '.yaml' not in txt:
            continue  # standalone submission entry has no configuration file
        assert 'get_package_share_directory' in txt, \
            '%s 未使用 get_package_share_directory' % _rel(lf)
        assert 'os.getcwd()' not in txt, \
            '%s 不得依赖当前工作目录定位配置' % _rel(lf)


# ---------------------------------------------------------------------------
# E. 参数被消费 / 悬空参数
# ---------------------------------------------------------------------------
def test_e_params_yaml_keys_are_consumed(params_yaml, text_of):
    """`tracker_params.yaml` 的每个键都必须有**非测试代码**消费（命中即 fail）。

    这条能挡住两类问题：参数名拼写错误（YAML 与代码对不上）、以及"文档里有、实际没人读"
    的悬空参数。
    """
    corpus = '\n'.join(txt for p, txt in text_of.items()
                       if 'test' not in p.replace('\\', '/').split('/'))
    orphans = sorted(k for k in params_yaml
                     if ("'%s'" % k) not in corpus and ('"%s"' % k) not in corpus)
    assert orphans == [], '悬空参数（无代码消费，疑似拼写错误）: %s' % orphans


def test_e2_params_yaml_has_required_namespaces(params_yaml):
    """§6.13 各命名空间的关键参数必须在参数文件里存在。"""
    required = ['serial.port', 'serial.baud', 'gimbal.pitch_sign',
                'control.kp_yaw', 'perception.marker_size_m', 'estimation.jerk_std',
                'localization.base_pose_source', 'chassis.v_max', 'grid.resolution',
                'csv.out_dir', 'csv.run_id', 'mission.run_duration_s',
                'mission.shutdown_grace_s', 'debug.enable_gt_monitor',
                'config.virtual_yaml', 'config.field_yaml']
    missing = [k for k in required if k not in params_yaml]
    assert missing == [], '缺少参数: %s' % missing


def test_e3_pitch_sign_is_single_source(params_yaml):
    """`gimbal.pitch_sign` 必须为 -1.0（DESIGN §2.3 修正 1）。"""
    assert float(params_yaml['gimbal.pitch_sign']) == -1.0


def test_e4_no_legacy_csv_format_switch(params_yaml):
    """比赛仅保留九列tick写出，不再配置旧八列模式。"""
    assert "csv.csv_extra_cols" not in params_yaml
    assert "judge.emit_t_tick" not in params_yaml


def test_e5_rate_params_are_floats(params_yaml):
    """所有 `*.rate_hz` 必须是 float（节点按 float 声明，int 会类型不匹配）。"""
    bad = {k: v for k, v in params_yaml.items()
           if k.endswith('rate_hz') and not isinstance(v, float)}
    assert bad == {}, '这些速率参数不是 float: %s' % bad


# ---------------------------------------------------------------------------
# F. launch overrides 键
# ---------------------------------------------------------------------------
def test_f_launch_override_keys_are_known(text_of, params_yaml):
    """launch 里写入 overrides 的参数名必须能在代码或 YAML 中找到。"""
    import re
    corpus = '\n'.join(txt for p, txt in text_of.items()
                       if 'test' not in p.replace('\\', '/').split('/'))
    keys: Set[str] = set()
    for lf in glob.glob(os.path.join(PKG_DIR, 'launch', '**', '*.py'), recursive=True):
        with open(lf, 'r', encoding='utf-8') as f:
            txt = f.read()
        keys |= set(re.findall(r"over\['([a-z_]+\.[a-z_]+)'\]", txt))
        keys |= set(re.findall(r"_put(?:_num|_int)?_if_set\(\s*over,\s*'([a-z_]+\.[a-z_]+)'",
                               txt))
    assert keys, '未解析到任何 overrides 键'
    missing = [k for k in sorted(keys)
               if ("'%s'" % k) not in corpus and ('"%s"' % k) not in corpus
               and k not in params_yaml]
    assert missing == [], 'launch 覆盖了未知参数: %s' % missing


# ---------------------------------------------------------------------------
# G. 话题表
# ---------------------------------------------------------------------------
def test_g_design_topics_present(text_of):
    """DESIGN §4.3 话题表里的每个话题都要在非测试代码中出现。"""
    corpus = '\n'.join(txt for p, txt in text_of.items()
                       if 'test' not in p.replace('\\', '/').split('/'))
    missing = [t for t in DESIGN_TOPICS
               if ("'%s'" % t) not in corpus and ('"%s"' % t) not in corpus]
    assert missing == [], '话题缺失: %s' % missing


# ---------------------------------------------------------------------------
# H. T-C8
# ---------------------------------------------------------------------------
def test_h_ground_truth_topic_only_in_gimbal_debug(text_of):
    """T-C8：`src/` 下（排除 gimbal_debug）不得出现真值话题字面量。"""
    hits = []
    for path, txt in text_of.items():
        if 'gimbal_debug' in path.replace('\\', '/').split('/'):
            continue
        if GT_LITERAL in txt:
            hits.append(_rel(path))
    assert hits == [], '真值话题泄漏到非 gimbal_debug 文件: %s' % hits


def test_h2_gimbal_debug_is_the_only_subscriber(text_of):
    """反向确认：真值话题确实被 gimbal_debug 订阅（否则说明功能丢了）。"""
    found = False
    for path, txt in text_of.items():
        if 'gimbal_debug' in path.replace('\\', '/').split('/') and GT_LITERAL in txt:
            found = True
    assert found, 'gimbal_debug 中未找到真值话题订阅'


# ---------------------------------------------------------------------------
# I. 禁用内参字面量
# ---------------------------------------------------------------------------
def _intrinsics_hits(text_of) -> List[str]:
    """返回出现禁用内参字面量的 `src/` 相对路径（只扫**生产代码**）。

    排除项：
      * `config/virtual.yaml` —— 它本来就是内参的**唯一数据源**（DESIGN §8.6）；
      * 测试文件 —— 测试需要校验"相机看到多少像素"这类量，可能出现与内参数值
        偶然相同的字面量（例如栅格 cell 索引 `239.5`），不构成违规。
        DESIGN §8.6 约束的是**生产代码路径**不得二次手抄内参。
    """
    hits = []
    for path, txt in text_of.items():
        rel = _rel(path)
        if '/test/' in rel or rel.startswith('test/'):
            continue
        if path.endswith('virtual.yaml'):
            continue
        if any(bad in txt for bad in FORBIDDEN_INTRINSICS):
            hits.append(rel)
    return sorted(set(hits))


def test_i_no_intrinsics_literals(text_of):
    """DESIGN §8.6：内参字面量只允许出现在 `config/virtual.yaml`（命中即 fail）。

    生产代码一旦需要"相机看到多少像素/多大视场"这类量，必须从 `virtual.yaml` 读
    `camera.width` / `camera.fx` 现算（例如
    `gimbal_control/search_pattern.py` 的 `half_fov_rad_from_intrinsics(width, fx)`），
    不允许把数值二次手抄进代码，否则换外参会静默产生系统偏差。
    """
    hits = _intrinsics_hits(text_of)
    assert hits == [], (
        '内参字面量必须只存在于 config/virtual.yaml；违规文件: %s' % hits)


def test_i2_virtual_yaml_is_the_only_intrinsics_source():
    """正向确认：内参确实被 `virtual.yaml` 提供，且有代码从中读取。"""
    with open(VIRTUAL_YAML, 'r', encoding='utf-8') as f:
        cam = yaml.safe_load(f)['camera']
    for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy'):
        assert key in cam, 'virtual.yaml 缺少 camera.%s' % key
    assert float(cam['fx']) > 0.0 and float(cam['fy']) > 0.0


# ---------------------------------------------------------------------------
# J. YAML / RViz 可解析
# ---------------------------------------------------------------------------
def test_j_all_yaml_parses():
    """所有 `.yaml` 与 `.rviz` 必须能被 `yaml.safe_load` 解析。"""
    files = sorted(glob.glob(os.path.join(SRC_DIR, '**', '*.yaml'), recursive=True)
                   + glob.glob(os.path.join(SRC_DIR, '**', '*.rviz'), recursive=True))
    assert files, '未找到 YAML/RViz 文件'
    bad = []
    for p in files:
        try:
            with open(p, 'r', encoding='utf-8') as f:
                yaml.safe_load(f)
        except (OSError, UnicodeError, yaml.YAMLError) as exc:  # 汇总实际文件/解析错误，不隐藏测试代码错误。
            bad.append('%s: %r' % (_rel(p), exc))
    assert bad == [], '\n'.join(bad)


# ---------------------------------------------------------------------------
# K. 墙上时钟（AST）
# ---------------------------------------------------------------------------
def test_k_no_wall_clock_calls(text_of):
    """DESIGN §8.2：生产代码不得真实调用 `time.time()` / `datetime.now()`。

    用 AST 判定，忽略注释与文档字符串（避免"文档里提到禁用写法"被误判）。
    """
    offenders = []
    for path, txt in text_of.items():
        rel = _rel(path)
        if '/test/' in rel or rel.startswith('test/'):
            continue
        try:
            tree = ast.parse(txt, path)
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            if isinstance(fn, ast.Attribute) and isinstance(fn.value, ast.Name):
                pair = (fn.value.id, fn.attr)
                if pair in (('time', 'time'), ('datetime', 'now'),
                            ('datetime', 'utcnow')):
                    offenders.append('%s:%d %s.%s()' % (rel, getattr(node, 'lineno', -1),
                                                        pair[0], pair[1]))
    assert offenders == [], '禁止使用墙上时钟: %s' % offenders


def test_k2_monotonic_time_is_used(text_of):
    """正向确认：工程里确实有 `time_utils.now()` 这一唯一时间入口。"""
    hits = [p for p, t in text_of.items() if 'time_utils' in t and 'now' in t]
    assert hits, '未发现使用 gimbal_common.time_utils 的代码'


# ---------------------------------------------------------------------------
# L. launch 运行期必需 YAML 键
# ---------------------------------------------------------------------------
def test_l_launch_required_yaml_keys(params_yaml):
    """launch 在运行期必须能从 YAML 读到的键（缺失会让 launch 直接抛错）。"""
    with open(VIRTUAL_YAML, 'r', encoding='utf-8') as f:
        vcfg = yaml.safe_load(f)
    ex = vcfg.get('extrinsics') or {}
    frames = ex.get('frames') or {}
    cam = vcfg.get('camera') or {}

    checks = {
        'virtual.yaml:extrinsics.camera_xyz': 'camera_xyz' in ex,
        'virtual.yaml:extrinsics.camera_rpy': 'camera_rpy' in ex,
        'virtual.yaml:extrinsics.frames.gimbal': 'gimbal' in frames,
        'virtual.yaml:extrinsics.frames.camera': 'camera' in frames,
        'virtual.yaml:extrinsics.frames.optical': 'optical' in frames,
        'virtual.yaml:camera.width': 'width' in cam,
        'virtual.yaml:camera.fx': 'fx' in cam,
        'tracker_params:localization.gimbal_mount_xyz_rpy': len(
            params_yaml.get('localization.gimbal_mount_xyz_rpy', [])) == 6,
        'tracker_params:mission.run_duration_s': 'mission.run_duration_s' in params_yaml,
        'tracker_params:mission.shutdown_grace_s':
            'mission.shutdown_grace_s' in params_yaml,
        'tracker_params:config.virtual_yaml': 'config.virtual_yaml' in params_yaml,
        'tracker_params:config.field_yaml': 'config.field_yaml' in params_yaml,
        'tracker_params:config.maps_dir': 'config.maps_dir' in params_yaml,
        'tracker_params:config.outputs_dir': 'config.outputs_dir' in params_yaml,
        'tracker_params:grid.field_yaml': 'grid.field_yaml' in params_yaml,
        'tracker_params:grid.out_dir': 'grid.out_dir' in params_yaml,
        'tracker_params:grid.map_topic': 'grid.map_topic' in params_yaml,
        'tracker_params:grid.republish_period_s':
            'grid.republish_period_s' in params_yaml,
        'tracker_params:grid.write_preview_png':
            'grid.write_preview_png' in params_yaml,
        'tracker_params:use_sim_time': 'use_sim_time' in params_yaml,
    }
    missing = sorted(k for k, ok in checks.items() if not ok)
    assert missing == [], 'launch 需要的 YAML 键缺失: %s' % missing


def test_l2_use_sim_time_is_false(params_yaml):
    """DESIGN §8.2 第 4 条：`use_sim_time` 必须显式为 false。"""
    assert params_yaml['use_sim_time'] is False


def test_l3_virtual_extrinsics_single_source():
    """`virtual.yaml` 的外参必须是"虚拟云台同一份"（全 0），否则会有 1.4° 系统偏差。"""
    with open(VIRTUAL_YAML, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    ex = cfg['extrinsics']
    assert ex['source'] == 'virtual'
    assert [float(v) for v in ex['camera_xyz']] == [0.0, 0.0, 0.0]
    assert [float(v) for v in ex['camera_rpy']] == [0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# M. 光学系旋转单一真源
# ---------------------------------------------------------------------------
def test_m_optical_rotation_matches_frames():
    """`static_tf` 的光学系 rpy 与 `gimbal_common.frames.R_opt_from_link()` 必须一致。"""
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    import importlib.util
    import numpy as np
    from gimbal_common import frames as FR

    launch_py = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             'launch', 'include', 'static_tf.launch.py')
    spec = importlib.util.spec_from_file_location('static_tf_launch_under_test', launch_py)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    rpy = list(mod._OPTICAL_RPY)
    assert len(rpy) == 3
    R_launch = FR.rpy_to_mat(rpy)
    assert float(np.abs(R_launch - FR.R_opt_from_link()).max()) < 1e-9, \
        'static_tf 的光学系旋转与 gimbal_common.frames 不一致（出现两份真源）'
    mod._assert_optical_rpy_matches_frames()          # 运行时自检不得抛错


def test_m2_optical_rpy_is_camera_link_to_optical():
    """该 rpy 的物理含义：光学系 z（前）→ 机体 x（前）、x（右）→ 机体 −y、y（下）→ 机体 −z。"""
    import numpy as np
    from gimbal_common import frames as FR

    R = FR.R_opt_from_link()
    assert np.allclose(R @ np.array([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0], atol=1e-12)
    assert np.allclose(R @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, -1.0], atol=1e-12)
