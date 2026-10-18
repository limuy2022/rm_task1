"""静态 TF 发布（**从 `config/virtual.yaml` 读外参，禁止手抄数值**，DESIGN §3.13 / §7-T05）。

发布（TF 表 §4.3）：

| TF | 来源 |
|---|---|
| `gimbal_link → camera_link` | `virtual.yaml` 的 `extrinsics.camera_xyz` / `camera_rpy` |
| `camera_link → camera_optical_frame` | 固定光学系旋转 `rpy = (-π/2, 0, -π/2)`（与 `gimbal_common.frames` 断言一致） |
| `base_link → gimbal_mount`（可选） | `tracker_params.yaml` 的 `localization.gimbal_mount_xyz_rpy` |

⚠ **不发布** `world → odom` / `map → odom`：那两个由 `map_frame_broadcaster_node` 发布
（`publish_world_odom` / `publish_map_odom`），避免出现双发布者（DESIGN §8.7）。

⚠ `static_transform_publisher` 的数值来自 YAML 文件内容，本文件不出现任何相机内参 / 外参字面量
（DESIGN §8.6）；光学系旋转会与 `gimbal_common.frames.R_opt_from_link()` 做**运行时一致性断言**。

⚠ 本文件无法在 Windows（无 ROS 2）上验证，待 Ubuntu 24.04 + ROS 2 Jazzy 上
``ros2 launch ... --show-args`` 与 ``ros2 run tf2_ros tf2_echo`` 验证。
"""

from __future__ import annotations

import math
import os
from typing import Any
from typing import List
from typing import Sequence

import yaml
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PACKAGE_NAME = 'gimbal_tracker_bringup'

# camera_link -> camera_optical_frame 的固定光学系旋转（REP-105：x 右 / y 下 / z 前）。
# 该值同时由 gimbal_common.frames._OPT_RPY 定义；下方 _launch_setup 会做一致性断言，
# 一旦两处不一致立即报错，避免"两份真源"。
_OPTICAL_RPY: Sequence[float] = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
def _share_dir(package: str = _PACKAGE_NAME) -> str:
    """从 ament 安装索引定位包的 share 目录。"""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _default_config(name: str) -> str:
    """share/config/<name> 的默认绝对路径。"""
    return os.path.join(_share_dir(), 'config', name)


def _as_bool(text: Any, default: bool = False) -> bool:
    """字符串转布尔（`true/1/yes/on`）。"""
    if text is None:
        return default
    s = str(text).strip().lower()
    if s == '':
        return default
    if s in ('1', 'true', 'yes', 'on', 'y', 't'):
        return True
    if s in ('0', 'false', 'no', 'off', 'n', 'f'):
        return False
    raise ValueError(f'非法布尔参数: {text!r}')


def _num(v: Any) -> str:
    """数值转字符串（给 static_transform_publisher 的命令行参数用）。"""
    return repr(float(v))


def _load_yaml_required(path: str) -> dict:
    """加载 YAML；缺失/非法直接抛错（DESIGN §8.4：配置文件缺失 = FATAL）。"""
    if not path or not os.path.isfile(path):
        raise RuntimeError(f'配置文件缺失: {path!r}（DESIGN §8.4 规定为 FATAL）')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError(f'配置文件内容不是映射: {path!r}')
    return data


def _vec3(value: Any, label: str) -> List[float]:
    """取出并校验长度为 3 的数值向量。"""
    seq = [float(x) for x in value]
    if len(seq) != 3 or not all(math.isfinite(x) for x in seq):
        raise ValueError(f'{label} 必须是三个有限数值，收到 {value!r}')
    return seq


def _vec6(value: Any, label: str) -> List[float]:
    """取出并校验长度为 6 的 ``[x,y,z,roll,pitch,yaw]``。"""
    seq = [float(x) for x in value]
    if len(seq) != 6 or not all(math.isfinite(x) for x in seq):
        raise ValueError(f'{label} 必须是六个有限数值，收到 {value!r}')
    return seq


def _assert_optical_rpy_matches_frames() -> None:
    """断言本文件的 `_OPTICAL_RPY` 与 `gimbal_common.frames` 的单一真源一致。

    Raises:
        RuntimeError: 二者不一致（说明有人改了外参定义却没同步）。
    """
    from gimbal_common.frames import R_opt_from_link, rpy_to_mat
    R_here = rpy_to_mat(list(_OPTICAL_RPY))
    R_ref = R_opt_from_link()
    if not bool(abs(R_here - R_ref).max() < 1e-9):
        raise RuntimeError(
            'static_tf.launch.py 的光学系旋转与 gimbal_common.frames.R_opt_from_link() '
            '不一致：请只保留 gimbal_common.frames 这一处真源')


def _make_tf(name: str, parent: str, child: str, xyz: Sequence[float],
             rpy: Sequence[float], use_sim_time: bool) -> Node:
    """构造一个 `static_transform_publisher` 节点（Jazzy 的 `--flag` 参数风格）。"""
    args = [
        '--x', _num(xyz[0]), '--y', _num(xyz[1]), '--z', _num(xyz[2]),
        '--roll', _num(rpy[0]), '--pitch', _num(rpy[1]), '--yaw', _num(rpy[2]),
        '--frame-id', str(parent), '--child-frame-id', str(child),
    ]
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        emulate_tty=True,
        arguments=args,
        parameters=[{'use_sim_time': bool(use_sim_time)}],
    )


# ---------------------------------------------------------------------------
# launch setup
# ---------------------------------------------------------------------------
def _launch_setup(context: LaunchContext, *_args: Any, **_kwargs: Any) -> List[Any]:
    """在 launch 时读取 YAML 并生成 static TF 节点。"""
    virtual_yaml = LaunchConfiguration('virtual_yaml').perform(context)
    tracker_params = LaunchConfiguration('tracker_params').perform(context)
    publish_mount = _as_bool(LaunchConfiguration('publish_mount_tf').perform(context), True)
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context), False)

    _assert_optical_rpy_matches_frames()

    vcfg = _load_yaml_required(virtual_yaml)
    ex = vcfg.get('extrinsics')
    if not isinstance(ex, dict):
        raise RuntimeError(f'{virtual_yaml!r} 缺少 extrinsics 段（DESIGN §10）')
    frames = ex.get('frames')
    if not isinstance(frames, dict):
        raise RuntimeError(f'{virtual_yaml!r} 缺少 extrinsics.frames 段（DESIGN §10）')

    cam_xyz = _vec3(ex.get('camera_xyz', [0.0, 0.0, 0.0]), 'extrinsics.camera_xyz')
    cam_rpy = _vec3(ex.get('camera_rpy', [0.0, 0.0, 0.0]), 'extrinsics.camera_rpy')
    f_gimbal = str(frames.get('gimbal', 'gimbal_link'))
    f_camera = str(frames.get('camera', 'camera_link'))
    f_optical = str(frames.get('optical', 'camera_optical_frame'))

    actions: List[Any] = [
        _make_tf('static_tf_gimbal_camera', f_gimbal, f_camera,
                 cam_xyz, cam_rpy, use_sim_time),
        _make_tf('static_tf_camera_optical', f_camera, f_optical,
                 (0.0, 0.0, 0.0), _OPTICAL_RPY, use_sim_time),
    ]

    if publish_mount:
        tcfg = _load_yaml_required(tracker_params)
        params = tcfg.get('/**', {}).get('ros__parameters', {}) if isinstance(
            tcfg.get('/**'), dict) else {}
        mount = params.get('localization.gimbal_mount_xyz_rpy')
        if mount is None:
            mount = [0.0] * 6
        m6 = _vec6(mount, 'localization.gimbal_mount_xyz_rpy')
        actions.append(_make_tf('static_tf_base_gimbal_mount', 'base_link',
                                'gimbal_mount', m6[0:3], m6[3:6], use_sim_time))

    return actions


def generate_launch_description() -> LaunchDescription:
    """生成静态 TF 的 LaunchDescription。"""
    return LaunchDescription([
        DeclareLaunchArgument(
            'virtual_yaml', default_value=_default_config('virtual.yaml'),
            description='相机内参/外参的唯一数据源（DESIGN §8.6）'),
        DeclareLaunchArgument(
            'tracker_params', default_value=_default_config('tracker_params.yaml'),
            description='用于取 localization.gimbal_mount_xyz_rpy'),
        DeclareLaunchArgument(
            'publish_mount_tf', default_value='true',
            description='是否发布 base_link → gimbal_mount 静态 TF'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='固定 false（DESIGN §8.2 第 4 条）'),
        OpaqueFunction(function=_launch_setup),
    ])
