"""gt_error_monitor_node：GT 真值角度误差监测 + 评估报告（DESIGN §3.12 / §7-T05）。

⚠ **本包是全工程唯一允许订阅 GT 真值话题的包**（DESIGN §8.7 / T-C8）。
该话题的字符串字面量只出现在本文件里；`src/` 下其它任何文件都不得出现
（校验：``grep -r <真值话题> src/ --exclude-dir=team43_debug`` 为空）。

功能：
1. 订阅 GT 真值（world 系 `geometry_msgs/PointStamped`）、`/gimbal/state`（`GimbalState`）、
   `/estimation/target_state`（`TargetState`）；另订阅 `/gimbal/t_map_world`（可选，
   用于把 map 系估计换算到 world 与真值比对）。
2. 按 PRD §6.1 计算 ``e_ang = angle(p_target_true − p_gimbal, 相机光轴方向)``（world 系），
   定频发布 `/debug/angle_error_deg`（`std_msgs/Float64`）。
3. 额外发布 `/debug/target_path`（`nav_msgs/Path`，真值轨迹）供 RViz 显示轨迹。
4. 退出时写 ``outputs/<run_id>/eval_report.md``（+ 误差曲线 ``eval_error_curve.csv``）。

时钟（主理人拍板 1，混合时钟）：`header.stamp` 用 ROS 系统时钟；
**所有采样时刻、定时器 dt、统计时间轴一律用 `team43_common.time_utils.now()`**。

本文件依赖 rclpy；离线检查不能替代 Ubuntu 22.04 + ROS2 Humble 通信验证。
"""

from __future__ import annotations

import math
import os
import sys
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from team43_interfaces.msg import GimbalState, TargetState

from team43_common import config_loader as CL
from team43_common import frames as FR
from team43_common import node_utils as NU
from team43_common.log_utils import get_logger, throttled
from team43_common.qos_utils import latched_qos, reliable_qos, sensor_qos
from team43_common.time_utils import now

from team43_debug import metrics as M

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
NODE_NAME = 'gt_error_monitor_node'

# ⚠ GT 真值话题字面量：**全工程仅此一处**（DESIGN §8.7 / T-C8）
GT_TOPIC_DEFAULT = '/virtual_gimbal/target_gt'

TOPIC_GIMBAL_STATE = '/gimbal/state'
TOPIC_TARGET_STATE = '/estimation/target_state'
TOPIC_T_MAP_WORLD = '/gimbal/t_map_world'          # map -> world，TRANSIENT_LOCAL

PARAM_DEFAULTS: Dict[str, Any] = {
    'debug.gt_topic': GT_TOPIC_DEFAULT,
    'debug.error_topic': '/debug/angle_error_deg',
    'debug.path_topic': '/debug/target_path',
    'debug.path_period_s': 0.5,
    'debug.path_max_points': 20000,
    'debug.sample_rate_hz': M.DEFAULT_SAMPLE_RATE_HZ,
    'debug.convergence_window_s': M.DEFAULT_CONVERGENCE_S,
    'debug.convergence_tol_deg': M.DEFAULT_CONVERGENCE_TOL_DEG,
    'debug.rms_max_deg': M.DEFAULT_RMS_MAX_DEG,
    'debug.max_deg': M.DEFAULT_ABS_MAX_DEG,
    'debug.duration_s': M.DEFAULT_DURATION_S,
    'debug.min_samples': M.DEFAULT_MIN_SAMPLES,
    'debug.post_convergence_skip_s': M.DEFAULT_POST_CONVERGENCE_SKIP_S,
    'debug.require_swing': False,
    'debug.bucket_s': 5.0,
    'debug.write_error_curve': True,
    'debug.report_name': 'eval_report.md',
    'debug.curve_name': 'eval_error_curve.csv',
    'debug.metrics_name': 'eval_metrics.json',
    'debug.gimbal_origin_xyz': [0.0, 0.0, 0.0],
    'debug.stale_timeout_s': 0.5,
    'config.virtual_yaml': 'virtual.yaml',
    'csv.out_dir': 'outputs',
    'csv.run_id': 'latest',
}

_CURVE_HEADER = '# t_mono,t_rel_s,e_ang_deg,bearing_deg,track_status,est_err_m\n'


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _resolve_virtual_yaml(name_or_path: str) -> str:
    """只接受显式绝对路径或已安装包的config文件。"""
    if os.path.isabs(name_or_path):
        return name_or_path
    return os.path.join(get_package_share_directory('team43_bringup'), 'config', name_or_path)


def half_fov_rad_from_yaml(path: str) -> float:
    """真实相机配置缺失/错误应使评估失败，不能跳过摆幅判定。"""
    cam = CL.load_yaml(path)['camera']
    width, fx = float(cam['width']), float(cam['fx'])
    if not math.isfinite(width) or not math.isfinite(fx) or width <= 0 or fx <= 0:
        raise ValueError('相机width/fx必须为有限正数')
    return math.atan(width / (2 * fx))


def _quat_xyzw_to_wxyz(q: Any) -> List[float]:
    """``geometry_msgs/Quaternion``(x,y,z,w) → ``[w,x,y,z]``。"""
    return [float(q.w), float(q.x), float(q.y), float(q.z)]


def _T_world_from_map(msg: Any) -> Optional[np.ndarray]:
    """把 ``map → world`` 的 ``TransformStamped`` 转成 ``T_world←map`` (4,4)。"""
    t = msg.transform.translation
    R = (FR.quat_to_mat(_quat_xyzw_to_wxyz(msg.transform.rotation)))
    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[0:3, 3] = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
    return T




def _ensure_dir(d: str) -> None:
    """确保目录存在。"""
    if d:
        os.makedirs(d, exist_ok=True)


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------
class GtErrorMonitorNode:
    """GT 真值角度误差监测器（构造后由 :func:`main` 负责 spin 与收尾）。"""

    def __init__(self, node: Any) -> None:
        self._node = node
        self._log = get_logger(node, 'gt_error')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        self._t_start = now()
        self._t_last = self._t_start
        self._t_last_path = self._t_start

        # 相机外参（单一真源 team43_common.frames），显式取一次避免每次重算
        self._R_gimbal_from_opt = (np.asarray(FR.R_opt_from_link(), dtype=np.float64))
        self._half_fov = half_fov_rad_from_yaml(
            _resolve_virtual_yaml(str(p.get('config.virtual_yaml', 'virtual.yaml'))))
        self._gimbal_origin = np.asarray(
            p.get('debug.gimbal_origin_xyz', [0.0, 0.0, 0.0]),
            dtype=np.float64).reshape(-1)
        if self._gimbal_origin.size < 3:
            self._gimbal_origin = np.concatenate(
                [self._gimbal_origin, np.zeros(3 - self._gimbal_origin.size)])

        # 最新外部输入
        self._gt_xyz: Optional[np.ndarray] = None
        self._t_gt: float = -1e18
        self._quat: Optional[np.ndarray] = None
        self._t_state: float = -1e18
        self._link_state: int = -1
        self._est_map_xyz: Optional[np.ndarray] = None
        self._track_status: int = -1
        self._est_valid: bool = False
        self._T_world_map: Optional[np.ndarray] = None

        # 累积
        self._samples: List[M.ErrorSample] = []
        self._path = None
        self._n_skipped = 0
        self._warned_no_tf = False

        rate = max(float(p.get('debug.sample_rate_hz', M.DEFAULT_SAMPLE_RATE_HZ)), 1e-3)
        self._period = 1.0 / rate

        # ---- QoS / 话题
        # 订阅端 sensor_qos（BEST_EFFORT）才能匹配 vgimbal/estimator 的 BEST_EFFORT 发布端；
        # error/path 发布端保持 RELIABLE，兼容 RViz 默认 Reliable 显示。
        node.create_subscription(PointStamped, str(p['debug.gt_topic']),
                                 self._on_gt, sensor_qos())
        node.create_subscription(GimbalState, TOPIC_GIMBAL_STATE,
                                 self._on_gimbal_state, sensor_qos())
        node.create_subscription(TargetState, TOPIC_TARGET_STATE,
                                 self._on_target_state, sensor_qos())
        node.create_subscription(TransformStamped, TOPIC_T_MAP_WORLD,
                                 self._on_t_map_world, latched_qos())
        self._pub_err = node.create_publisher(Float64, str(p['debug.error_topic']),
                                              reliable_qos())
        self._pub_path = node.create_publisher(Path, str(p['debug.path_topic']),
                                               reliable_qos())
        self._path = Path()
        self._path.header.frame_id = 'world'
        node.create_timer(self._period, self._on_timer)

        self._log.info(
            f'gt_error_monitor 启动: 真值话题={p["debug.gt_topic"]}, 采样 {rate:.1f}Hz, '
            f'半视场={math.degrees(self._half_fov):.2f}°, '
            f'RMS≤{float(p["debug.rms_max_deg"]):.2f}°/max≤{float(p["debug.max_deg"]):.2f}°'
            f'/收敛≤{float(p["debug.convergence_window_s"]):.2f}s')

    # -- 回调 ----------------------------------------------------------------
    def _on_gt(self, msg: Any) -> None:
        """缓存目标真值（world 系）。仅日志用外部 stamp（DESIGN §8.2 第 3 条）。"""
        self._gt_xyz = np.array([float(msg.point.x), float(msg.point.y),
                                 float(msg.point.z)], dtype=np.float64)
        self._t_gt = now()

    def _on_gimbal_state(self, msg: Any) -> None:
        """缓存云台姿态（quat = world→gimbal_link）与链路状态。"""
        q = np.asarray([float(msg.quat[0]), float(msg.quat[1]),
                        float(msg.quat[2]), float(msg.quat[3])],
                       dtype=np.float64)
        n = float(np.linalg.norm(q))
        if not math.isfinite(n) or n <= 1e-9:
            return
        self._quat = q / n
        self._link_state = int(msg.link_state)
        self._t_state = now()

    def _on_target_state(self, msg: Any) -> None:
        """缓存目标估计（map 系）与 track_status。"""
        self._est_valid = bool(msg.valid)
        self._track_status = int(msg.track_status)
        self._est_map_xyz = np.array([float(msg.cur_x), float(msg.cur_y), 0.0],
                                     dtype=np.float64)

    def _on_t_map_world(self, msg: Any) -> None:
        """缓存 ``T_world←map``（用于把 map 系估计换算到 world 比对真值）。"""
        T = _T_world_from_map(msg)
        if T is not None:
            self._T_world_map = T

    # -- 主循环 --------------------------------------------------------------
    def _on_timer(self) -> None:
        """定频采样：算 e_ang → 发布 → 累积。dt / 时刻全部用单调时钟。"""
        t_now = now()
        dt = t_now - self._t_last
        self._t_last = t_now
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0

        stale = max(float(self._params.get('debug.stale_timeout_s', 0.5)), 1e-3)
        if (self._gt_xyz is None or self._quat is None
                or (t_now - self._t_gt) > stale or (t_now - self._t_state) > stale):
            self._n_skipped += 1
            throttled(self._log, 'warning',
                      '尚未收到/已超时（%.2fs）真值或云台姿态，跳过本次采样'
                      % stale,
                      'team43_debug.waiting')
            return

        e_deg = M.angle_error_deg(self._gt_xyz, self._gimbal_origin, self._quat,
                                  self._R_gimbal_from_opt)
        bearing = M.bearing_azimuth_rad(self._gt_xyz, self._gimbal_origin)
        est_err = float('nan')
        if self._est_valid and self._est_map_xyz is not None:
            if self._T_world_map is None:
                if not self._warned_no_tf:
                    self._warned_no_tf = True
                    self._log.warning(f'未收到 {TOPIC_T_MAP_WORLD}，'
                                      '估计 vs 真值比对暂不可用（不影响 e_ang）')
            else:
                p_est_world = (self._T_world_map[0:3, 0:3] @ self._est_map_xyz
                               + self._T_world_map[0:3, 3])
                est_err = float(np.linalg.norm(p_est_world - self._gt_xyz))

        self._samples.append(M.ErrorSample(
            t=t_now, e_deg=e_deg, bearing_rad=bearing,
            est_err_m=est_err, track_status=self._track_status))

        self._publish_error(e_deg)
        self._append_path(t_now)

    def _publish_error(self, e_deg: float) -> None:
        """发布 ``/debug/angle_error_deg``。"""
        if not math.isfinite(e_deg):  # pragma: no cover
            return
        msg = Float64()
        msg.data = float(e_deg)
        self._pub_err.publish(msg)

    def _append_path(self, t_now: float) -> None:
        """把真值点追加到 ``/debug/target_path`` 并按周期发布（供 RViz 显示轨迹）。"""
        if self._path is None or self._gt_xyz is None:  # pragma: no cover
            return
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position.x = float(self._gt_xyz[0])
        ps.pose.position.y = float(self._gt_xyz[1])
        ps.pose.position.z = float(self._gt_xyz[2])
        ps.pose.orientation.w = 1.0
        self._path.poses.append(ps)
        max_n = int(self._params.get('debug.path_max_points', 20000))
        if max_n > 0 and len(self._path.poses) > max_n:
            del self._path.poses[0:len(self._path.poses) - max_n]
        if (t_now - self._t_last_path) >= float(
                self._params.get('debug.path_period_s', 0.5)):
            self._path.header.stamp = self._node.get_clock().now().to_msg()
            self._pub_path.publish(self._path)
            self._t_last_path = t_now

    # -- 收尾 ----------------------------------------------------------------
    def _build_thresholds(self) -> M.EvalThresholds:
        """组装判据。"""
        p = self._params
        return M.EvalThresholds(
            rms_max_deg=float(p.get('debug.rms_max_deg', M.DEFAULT_RMS_MAX_DEG)),
            abs_max_deg=float(p.get('debug.max_deg', M.DEFAULT_ABS_MAX_DEG)),
            convergence_time_s=float(p.get('debug.convergence_window_s',
                                           M.DEFAULT_CONVERGENCE_S)),
            duration_s=float(p.get('debug.duration_s', M.DEFAULT_DURATION_S)),
            min_samples=int(p.get('debug.min_samples', M.DEFAULT_MIN_SAMPLES)),
            post_convergence_skip_s=float(p.get('debug.post_convergence_skip_s',
                                                M.DEFAULT_POST_CONVERGENCE_SKIP_S)),
            convergence_tol_deg=float(p.get('debug.convergence_tol_deg',
                                            M.DEFAULT_CONVERGENCE_TOL_DEG)),
            require_swing=bool(p.get('debug.require_swing', False)),
            half_fov_rad=float(self._half_fov),
        )

    def shutdown(self) -> None:
        """写 ``eval_report.md`` / ``eval_error_curve.csv`` / ``eval_metrics.json``。"""
        th = self._build_thresholds()
        res = M.evaluate(self._samples, th)
        buckets = M.bucket_rms(self._samples,
                               bucket_s=float(self._params.get('debug.bucket_s', 5.0)))

        run_dir = os.path.join(str(self._params.get('csv.out_dir', 'outputs')),
                               str(self._params.get('csv.run_id', 'latest')))
        _ensure_dir(run_dir)

        meta = {
            'run_id': str(self._params.get('csv.run_id', 'latest')),
            'gt_topic': str(self._params.get('debug.gt_topic', GT_TOPIC_DEFAULT)),
            'sample_rate_hz': '%.1f' % float(
                self._params.get('debug.sample_rate_hz', M.DEFAULT_SAMPLE_RATE_HZ)),
            'half_fov_deg': '%.3f' % math.degrees(self._half_fov),
            'n_skipped_no_data': '%d' % int(self._n_skipped),
            'gimbal_origin_world_m': '[%.3f, %.3f, %.3f]' % tuple(self._gimbal_origin[:3]),
            'link_state_last': '%d' % int(self._link_state),
            'elapsed_mono_s': '%.3f' % max(0.0, now() - self._t_start),
            'est_vs_gt_available': ('yes' if self._T_world_map is not None else 'no'),
        }
        notes = [
            '本报告由 GT 真值监测节点生成，仅在 `debug.enable_gt_monitor=true` 时存在；'
            '正式比赛的算法路径**不依赖**真值（DESIGN §8.7）。',
            '真值话题与云台姿态的 ROS `header.stamp` 只用于日志；'
            '统计时间轴用 `team43_common.time_utils.now()`（单调）。',
        ]
        if res.n_samples == 0:
            notes.append('未采集到任何有效样本：请确认 GT 真值话题与 `/gimbal/state` 有数据。')

        report = M.format_report_md(res, th, meta=meta, buckets=buckets, notes=notes)
        report_path = os.path.join(run_dir, str(self._params.get('debug.report_name',
                                                                 'eval_report.md')))
        self._write_text(report_path, report)

        if bool(self._params.get('debug.write_error_curve', True)):
            self._write_curve(os.path.join(
                run_dir, str(self._params.get('debug.curve_name',
                                              'eval_error_curve.csv'))))

        metrics_path = os.path.join(run_dir, str(self._params.get('debug.metrics_name',
                                                                  'eval_metrics.json')))
        self._write_json(metrics_path, res.as_dict())

        conv = ('None' if res.convergence_time_s is None
                else f'{res.convergence_time_s:.3f}s')
        self._log.info(f'评估完成: RMS={_s(res.rms_deg)}° max={_s(res.max_deg)}° '
                       f'收敛={conv} → {"PASS" if res.passed else "FAIL"}'
                       f'（{report_path}）')
        if not res.passed:
            self._log.warning(f'未达标：{res.reason or "RMS/max/收敛之一超限"}')

    def _write_text(self, path: str, text: str) -> None:
        """写文本文件（失败只报错，不影响退出）。"""
        try:
            _ensure_dir(os.path.dirname(os.path.abspath(path)))
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
        except OSError as exc:  # 退出时报告/曲线是可选产物；写盘失败明确报告，继续保存其余文件。
            self._log.error(f'写 {path} 失败: {exc!r}')

    def _write_curve(self, path: str) -> None:
        """写误差曲线 CSV（便于离线核对报告数字）。"""
        try:
            _ensure_dir(os.path.dirname(os.path.abspath(path)))
            t0 = self._samples[0].t if self._samples else 0.0
            with open(path, 'w', encoding='utf-8', newline='\n') as f:
                f.write(_CURVE_HEADER)
                for s in self._samples:
                    f.write('%.6f,%.6f,%s,%.6f,%d,%s\n' % (
                        float(s.t), float(s.t) - float(t0),
                        ('%.6f' % s.e_deg) if math.isfinite(s.e_deg) else 'nan',
                        math.degrees(s.bearing_rad) if math.isfinite(s.bearing_rad) else float('nan'),
                        int(s.track_status),
                        ('%.6f' % s.est_err_m) if math.isfinite(s.est_err_m) else 'nan'))
        except OSError as exc:  # 退出时报告/曲线是可选产物；写盘失败明确报告，继续保存其余文件。
            self._log.error(f'写误差曲线失败: {exc!r}')

    def _write_json(self, path: str, data: Dict[str, Any]) -> None:
        """写 JSON（失败只报错）。"""
        import json
        try:
            _ensure_dir(os.path.dirname(os.path.abspath(path)))
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                f.write('\n')
        except OSError as exc:  # 退出时报告/曲线是可选产物；写盘失败明确报告，继续保存其余文件。
            self._log.error(f'写 {path} 失败: {exc!r}')


def _s(v: Any) -> str:
    """数值转字符串（非有限显示 n/a）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):  # 报告中的缺失统计值显示n/a，不伪造成数字。
        return 'n/a'
    return ('%.3f' % f) if math.isfinite(f) else 'n/a'


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main(args: Optional[list] = None) -> int:
    """节点入口（`use_sim_time=false` 由 `node_utils.create_node` 强制）。"""
    rclpy.init(args=args)
    node = NU.create_node(NODE_NAME)
    app = GtErrorMonitorNode(node)
    try:
        NU.spin_with_finally(node, on_shutdown=app.shutdown)
    finally:
        NU.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
