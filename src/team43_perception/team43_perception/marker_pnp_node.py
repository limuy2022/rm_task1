"""视觉感知主节点（DESIGN §3.5 / §6.4-§6.6）。

订阅 ``/perception/marker_corners``（``team43_interfaces/MarkerCorners``）与可选的
``/virtual_gimbal/camera_info``，解 PnP 后发布
``team43_interfaces/MarkerObservation``（``frame_id = camera_optical_frame``）。

要点：

* 内参**默认**来自 ``virtual.yaml``（``camera_info_source: yaml``，单一数据源）；
  ``topic`` 模式才用 ``/virtual_gimbal/camera_info``。
* PnP 失败 / 重投影超标 / 深度越界 → **视为本帧无观测**（正常路径，不报错）。
* 只输出 optical 系下的点，world/map 换算放在 estimation（D-3）。
* 输出保留原始图像stamp；丢失、拒绝和ID变化时显式发无效观测并清初值。

本文件依赖 ROS 2；实际比赛接口须在 Ubuntu 22.04 + ROS 2 Humble 验证。
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo

from team43_common import config_loader as CL
from team43_common.log_utils import get_logger, throttled
from team43_common import node_utils as NU
from team43_common import time_utils as TU
from team43_common.qos_utils import sensor_qos
from team43_interfaces.msg import MarkerCorners, MarkerObservation

from .marker_source import make_marker_source
from .noise_model import NoiseModel
from .pnp_solver import PnpSolver

NODE_NAME = 'marker_pnp_node'

PARAM_DEFAULTS = {
    'perception.marker_size_m': 0.075,
    'perception.camera_info_source': 'yaml',
    'perception.reproj_max_px': 2.0,
    'perception.depth_min': 0.05,
    'perception.depth_max': 30.0,
    # -1=等待裁判通知（tools/set_target.bash 设置）；权威默认在 tracker_defaults.yaml，
    # 与 aruco_detector_node 的安全默认保持一致。
    'perception.target_digit': -1,
    'perception.id_filter_mode': 'off',
    'perception.corner_noise_px': 0.5,
    'perception.k_lat_floor': 5.0e-4,
    'perception.k_depth_floor': 4.0e-3,
    'perception.force_k_lat': 0.0,
    'perception.force_k_depth': 0.0,
    'perception.r_min_sigma': 0.002,
    'perception.marker_corners_topic': '/perception/marker_corners',
    'perception.camera_info_topic': '/virtual_gimbal/camera_info',
    'config.virtual_yaml': 'virtual.yaml',
}


def _resolve_virtual_yaml(name_or_path: str) -> str:
    """绝对路径原样读取；文件名从安装的 bringup/config 获取。"""
    if not name_or_path:
        raise ValueError('config.virtual_yaml cannot be empty')
    if os.path.isabs(name_or_path):
        return name_or_path
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('team43_bringup'),
                        'config', name_or_path)


class MarkerPnpNode:
    """marker 角点 → PnP → ``MarkerObservation`` 的节点。"""

    def __init__(self, node) -> None:
        self._node = node
        self._log = get_logger(node, 'pnp')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        # ---- 内参：默认取自 virtual.yaml（唯一数据源）
        vpath = _resolve_virtual_yaml(str(p['config.virtual_yaml']))
        vcfg = CL.load_yaml(vpath, required=True)
        cam = vcfg['camera']
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

        qos = sensor_qos(10)
        self._pub = node.create_publisher(MarkerObservation, '/perception/marker_cam', qos)
        node.create_subscription(MarkerCorners,
                                 str(p['perception.marker_corners_topic']),
                                 self._on_corners, qos)
        node.create_subscription(CameraInfo, str(p['perception.camera_info_topic']),
                                 self._on_camera_info, qos)

        self._n_obs: int = 0
        self._n_reject: int = 0
        self._t_last_obs: float = 0.0
        self._last_target = int(p['perception.target_digit'])
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
        except (TypeError, ValueError, OverflowError) as exc:
            # 外部 CameraInfo 数值非法时保留上一份有效标定，不写入无效内参。
            throttled(self._log, 'warning', f'camera_info 解析失败: {exc!r}', 'cam_info')

    def _on_corners(self, msg: MarkerCorners) -> None:
        """收到角点 → 过滤 → PnP → 发布 ``MarkerObservation``。"""
        target = int(self._node.get_parameter('perception.target_digit').value)
        self._input_stamp = msg.header.stamp
        if target != self._last_target:
            self._solver.reset_guess()
            self._source.reset()
            self._last_target = target
        # ID is supplied by the decoder, never inferred from a parameter.
        # This rejects queued old-target frames during a two-node ID switch.
        if (target < 0 or msg.marker_id != target or not msg.valid
                or msg.header.frame_id != 'camera_optical_frame'):
            self._clear_observation(max(0, target))
            return
        self._params['perception.target_digit'] = target
        self._source.target_digit = target
        t_mono = TU.now()
        raw = self._source.on_polygon(msg, t_mono=t_mono)
        self._source.poll()  # 本节点同步消费raw，不累积另一份待处理队列
        if raw is None:
            self._clear_observation(target)
            return
        res = self._solver.solve(raw.corners_uv)
        if not res.ok:
            self._n_reject += 1
            self._clear_observation(target)
            # Numeric errors change every frame. A key such as
            # pnp_reproj_2.81px bypasses throttling and floods a real NUC log.
            category = res.reason.split('_', 1)[0]
            throttled(self._log, 'info', f'PnP 拒绝：{res.reason}',
                         f'pnp_reject_{category}', 2.0)
            return
        self._n_obs += 1
        self._t_last_obs = t_mono
        cov = self._noise.cov_optical(res.depth)
        self._publish(raw.marker_id, res.tvec_opt, cov, res.depth,
                      res.reproj_err_px, True)

    def _clear_observation(self, target: int) -> None:
        self._source.reset()
        self._solver.reset_guess()
        self._publish(target, np.zeros(3), self._noise.cov_optical(0.0),
                      0.0, float('inf'), False)

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
                       f'过滤统计={self._source.stats}')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = NU.create_node(NODE_NAME)
    try:
        app = MarkerPnpNode(node)
        NU.spin_with_finally(node, on_shutdown=app.shutdown)
    finally:
        node.destroy_node()
        NU.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
