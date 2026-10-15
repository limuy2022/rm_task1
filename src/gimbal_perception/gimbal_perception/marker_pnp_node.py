"""视觉感知主节点（DESIGN §3.5 / §6.4-§6.6）。

订阅 ``/virtual_gimbal/marker_corners``（``geometry_msgs/PolygonStamped``）与可选的
``/virtual_gimbal/camera_info``，解 PnP 后发布
``gimbal_interfaces/MarkerObservation``（``frame_id = camera_optical_frame``）。

要点：

* 内参**默认**来自 ``virtual.yaml``（``camera_info_source: yaml``，单一数据源）；
  ``topic`` 模式才用 ``/virtual_gimbal/camera_info``。
* PnP 失败 / 重投影超标 / 深度越界 → **视为本帧无观测**（正常路径，不报错）。
* 只输出 optical 系下的点，world/map 换算放在 estimation（D-3）。
* 外部话题的 ``header.stamp`` 只用于日志与丢帧统计；本地时刻用单调时钟。

⚠ 本文件依赖 ROS 2，只能在 Ubuntu 24.04 + ROS 2 Jazzy 上编译/运行验证。
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CameraInfo

from gimbal_common import config_loader as CL
from gimbal_common import log_utils as LU
from gimbal_common import node_utils as NU
from gimbal_common import time_utils as TU
from gimbal_interfaces.msg import MarkerObservation

from .marker_source import make_marker_source
from .noise_model import NoiseModel
from .pnp_solver import PnpSolver

NODE_NAME = 'marker_pnp_node'

PARAM_DEFAULTS = {
    'perception.marker_size_m': 0.16,
    'perception.camera_info_source': 'yaml',
    'perception.reproj_max_px': 2.0,
    'perception.depth_min': 0.05,
    'perception.depth_max': 30.0,
    'perception.target_digit': 0,
    'perception.id_filter_mode': 'off',
    'perception.corner_noise_px': 0.5,
    'perception.k_lat_floor': 5.0e-4,
    'perception.k_depth_floor': 4.0e-3,
    'perception.force_k_lat': 0.0,
    'perception.force_k_depth': 0.0,
    'perception.r_min_sigma': 0.002,
    'perception.marker_corners_topic': '/virtual_gimbal/marker_corners',
    'perception.camera_info_topic': '/virtual_gimbal/camera_info',
    'config.virtual_yaml': 'virtual.yaml',
}


def _sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _resolve_virtual_yaml(name_or_path: str) -> str:
    """解析 ``virtual.yaml`` 路径（包名相对 → 绝对；缺失时返回原样由上层 FATAL）。"""
    if not name_or_path:
        name_or_path = 'virtual.yaml'
    if os.path.isabs(name_or_path) and os.path.isfile(name_or_path):
        return name_or_path
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('gimbal_tracker_bringup')
        cand = os.path.join(share, 'config', os.path.basename(name_or_path))
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    cand = os.path.join(os.getcwd(), 'src', 'gimbal_tracker_bringup', 'config',
                        os.path.basename(name_or_path))
    return cand if os.path.isfile(cand) else name_or_path


class MarkerPnpNode:
    """marker 角点 → PnP → ``MarkerObservation`` 的节点。"""

    def __init__(self, node) -> None:
        self._node = node
        self._log = LU.get_logger(node, 'pnp')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        # ---- 内参：默认取自 virtual.yaml（唯一数据源）
        vpath = _resolve_virtual_yaml(str(p['config.virtual_yaml']))
        try:
            vcfg = CL.load_yaml(vpath, required=True)
        except CL.ConfigError as exc:
            self._log.fatal(f'{exc}（DESIGN §8.6：virtual.yaml 是内参/外参唯一数据源，'
                            '禁止用其它外参跑仿真）')
            raise
        cam = vcfg.get('camera', {})
        self._solver = PnpSolver.from_camera_config(
            cam,
            marker_size_m=float(p['perception.marker_size_m']),
            reproj_max_px=float(p['perception.reproj_max_px']),
            depth_min=float(p['perception.depth_min']),
            depth_max=float(p['perception.depth_max']),
        )
        self._noise = NoiseModel(
            fx=self._solver.fx,
            marker_size_m=float(p['perception.marker_size_m']),
            corner_noise_px=float(p['perception.corner_noise_px']),
            k_lat_floor=float(p['perception.k_lat_floor']),
            k_depth_floor=float(p['perception.k_depth_floor']),
            force_k_lat=float(p['perception.force_k_lat']),
            force_k_depth=float(p['perception.force_k_depth']),
            r_min_sigma=float(p['perception.r_min_sigma']),
        )
        self._use_topic_info = str(p['perception.camera_info_source']).lower() == 'topic'

        self._source = make_marker_source(
            'single_topic',
            target_digit=int(p['perception.target_digit']),
            id_filter_mode=str(p['perception.id_filter_mode']),
        )

        qos = _sensor_qos(10)
        self._pub = node.create_publisher(MarkerObservation, '/perception/marker_cam', qos)
        node.create_subscription(PolygonStamped,
                                 str(p['perception.marker_corners_topic']),
                                 self._on_corners, qos)
        node.create_subscription(CameraInfo, str(p['perception.camera_info_topic']),
                                 self._on_camera_info, qos)

        self._n_obs: int = 0
        self._n_reject: int = 0
        self._t_last_obs: float = 0.0
        self._log.info(
            f'感知节点启动：内参源={p["perception.camera_info_source"]}（{vpath}），'
            f'fx={self._solver.fx:.2f}，marker={p["perception.marker_size_m"]}m，'
            f'reproj_max={p["perception.reproj_max_px"]}px，源={self._source.name()}，'
            f'k_lat={self._noise.k_lat:.3e} k_depth={self._noise.k_depth:.3e}')

    # ------------------------------------------------------------------ 回调
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """``camera_info_source=topic`` 时用话题内参覆盖（逐帧刷新，开销可忽略）。"""
        if not self._use_topic_info or msg is None:
            return
        try:
            k = list(msg.k)
            fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
            if fx > 0.0 and fy > 0.0:
                self._solver.set_intrinsics(fx, fy, cx, cy, list(msg.d) if msg.d else None)
                self._noise.fx = fx
        except Exception as exc:
            LU.throttled(self._log, 'warning', f'camera_info 解析失败: {exc!r}', 'cam_info', 1.0)

    def _on_corners(self, msg: PolygonStamped) -> None:
        """收到角点 → 过滤 → PnP → 发布 ``MarkerObservation``。"""
        try:
            target = int(self._node.get_parameter('perception.target_digit').value)
            if target < 0:
                return
            self._params['perception.target_digit'] = target
            self._source.target_digit = target
            self._input_stamp = msg.header.stamp
            t_mono = TU.now()
            raw = self._source.on_polygon(msg, marker_id=int(self._params['perception.target_digit']),
                                          t_mono=t_mono)
            self._source.poll()  # 本节点同步消费raw，不累积另一份待处理队列
            if raw is None:
                # 空/长度≠4/id 不匹配 —— 正常路径，不报错
                return
            res = self._solver.solve(raw.corners_uv)
            if not res.ok:
                self._n_reject += 1
                self._publish(raw.marker_id, np.zeros(3), self._noise.cov_optical(0.0),
                              0.0, float('inf'), False)
                LU.throttled(self._log, 'info', f'PnP 拒绝：{res.reason}', f'pnp_{res.reason}', 2.0)
                return
            self._n_obs += 1
            self._t_last_obs = t_mono
            cov = self._noise.cov_optical(res.depth)
            self._publish(raw.marker_id, res.tvec_opt, cov, res.depth,
                          res.reproj_err_px, True)
        except Exception as exc:
            LU.throttled(self._log, 'error', f'corners 回调异常: {exc!r}', 'corners_cb', 1.0)

    def _publish(self, marker_id: int, tvec: np.ndarray, cov: np.ndarray,
                 depth: float, reproj: float, valid: bool) -> None:
        """发布一条 ``MarkerObservation``。"""
        msg = MarkerObservation()
        msg.header.stamp = self._input_stamp
        msg.header.frame_id = 'camera_optical_frame'
        msg.marker_id = int(marker_id)
        msg.x = float(tvec[0])
        msg.y = float(tvec[1])
        msg.z = float(tvec[2])
        msg.cov_optical = [float(v) for v in np.asarray(cov).reshape(9)]
        msg.depth = float(depth)
        msg.reproj_err_px = float(reproj)
        msg.valid = bool(valid)
        self._pub.publish(msg)

    def shutdown(self) -> None:
        """收尾日志。"""
        self._log.info(f'感知节点退出：有效观测 {self._n_obs} 帧，拒绝 {self._n_reject} 帧，'
                       f'过滤统计={getattr(self._source, "stats", {})}')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = NU.create_node(NODE_NAME)
    try:
        app = MarkerPnpNode(node)
    except Exception:
        node.destroy_node()
        NU.shutdown()
        return 1
    try:
        NU.spin_with_finally(node, on_shutdown=app.shutdown)
    finally:
        node.destroy_node()
        NU.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
