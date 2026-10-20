"""aruco_detector_node：从 `/image_raw` 检测 ArUco 码，输出角点给 PnP。

官方环境（campus-contest-env「上车后你能得到什么」/「常见坑」）：

- 相机 `/image_raw`，`sensor_msgs/Image`，rgb8，1440x1080，约 50 fps，
  每帧约 4.6 MB。
- 发布端是 reliable，订阅**也必须 reliable + KEEP_LAST + depth 1**；
  用 SensorDataQoS（best-effort）在 50 fps 下只能收到约 21 Hz，丢掉一半以上。
- **不要在回调里做重活**：回调只保存最新帧的引用，检测在定时器里做，
  并按 `detect_rate_hz` 节流（评分是 10 Hz，30 Hz 检测绰绰有余）。
- 码用 `DICT_4X4_50`，边长 0.1 m。

输出：``gimbal_interfaces/MarkerCorners``，ID和角点携带同一图像stamp。
保持ArUco解码后的规范角点顺序，不按图像左上重新排序。
``frame_id = camera_optical_frame``；valid=false、空corners表示无观测。

精度（cur_x/cur_y 权重合计 0.3，是仅次于 base 的大项）：
``cv2.aruco.detectMarkers`` 的角点是**整像素**级，直接拿去做 PnP 会带来
明显偏差；这里默认开启 ``cornerSubPix`` 亚像素 refinement，把角点精化到
约 0.1 px，可以显著降低定位误差。
"""

from __future__ import annotations

import sys
import time
from typing import Any
from typing import Dict
from typing import Optional

import numpy as np

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from gimbal_interfaces.msg import MarkerCorners

from cv_bridge import CvBridge, CvBridgeError

from gimbal_common.log_utils import get_logger, throttled
from gimbal_common.node_utils import declare_params
from gimbal_common.qos_utils import reliable_qos

NODE_NAME = 'aruco_detector_node'

PARAM_DEFAULTS: Dict[str, Any] = {
    'perception.image_topic': '/image_raw',
    'perception.marker_corners_topic': '/perception/marker_corners',
    'perception.corners_frame_id': 'camera_optical_frame',
    'perception.target_digit': 0,           # 赛前由主办方告知、需要瞄准的数字
    'perception.aruco_dict': 'DICT_4X4_50',
    'perception.detect_rate_hz': 30.0,      # 评分 10 Hz，30 Hz 足够
    'perception.do_subpix': True,           # 亚像素精化（降低 PnP 误差）
    'perception.subpix_win': 3,
    'perception.publish_empty_on_lost': True, # 丢失时发空 polygon（下游按无观测处理）
    'perception.max_corners': 4,
    'perception.image_timeout_s': 0.2,
}


def _make_detector_params() -> Any:
    return cv2.aruco.DetectorParameters_create()


class ArucoDetectorNode(Node):  # type: ignore[misc]
    """ArUco 检测节点：图像 → 角点。"""

    def __init__(self, name: str = NODE_NAME) -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._params: Dict[str, Any] = declare_params(self, PARAM_DEFAULTS)

        self._bridge = CvBridge()
        self._dict = self._make_dict(str(self._params['perception.aruco_dict']))
        self._detector = _make_detector_params()

        self._target_digit = int(self._params['perception.target_digit'])
        self._latest: Optional[Any] = None
        self._latest_stamp: Optional[Any] = None
        self._frame_seq: int = 0
        self._last_image_received: Optional[float] = None
        self._loss_announced = False
        self._n_det: int = 0
        self._n_frame: int = 0
        self._t_last_detect: float = 0.0

        self._image_sub = self.create_subscription(
            Image, str(self._params['perception.image_topic']),
            self._on_image, reliable_qos(1))
        self._pub = self.create_publisher(
            MarkerCorners, str(self._params['perception.marker_corners_topic']),
            reliable_qos(10))

        period = 1.0 / max(float(self._params['perception.detect_rate_hz']), 1e-3)
        self.create_timer(period, self._on_timer)
        self._log.info(f'aruco_detector_node 启动: '
                       f'{self._params["perception.image_topic"]} → '
                       f'{self._params["perception.marker_corners_topic"]}，'
                       f'dict={self._params["perception.aruco_dict"]}，'
                       f'target={self._target_digit}，'
                       f'{float(self._params["perception.detect_rate_hz"]):.0f}Hz')

    # -- 参数 -----------------------------------------------------------------
    def _make_dict(self, name: str) -> Any:
        """配置名非法直接报错。"""
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    # -- 回调 -----------------------------------------------------------------
    def _on_image(self, msg: Any) -> None:
        """只保存最新帧引用，**不在回调里做重活**（官方明确要求）。"""
        self._latest = msg
        self._latest_stamp = msg.header.stamp
        self._last_image_received = time.monotonic()
        self._loss_announced = False
        self._frame_seq += 1

    def _on_timer(self) -> None:
        """按 detect_rate_hz 处理最新一帧。"""
        # ROS参数服务可在启动后设置ID；负数表示尚未收到赛前ID。
        target = int(self.get_parameter('perception.target_digit').value)
        if target != self._target_digit:
            self._target_digit = target
            self._params['perception.target_digit'] = target
            self._latest = None  # 不使用切换前的缓存图像
            self._log.info('目标ID更新为 %d' % target)
            self._publish(None, self.get_clock().now().to_msg())
        if target < 0:
            return
        msg = self._latest
        if msg is None:
            # 相机与 detect 定时器同频（如 30fps 对 30Hz）拍频下，周期性出现
            # "本周期没有新帧"的空 tick 是正常现象，不能当成图像断流告警；
            # 只有从未收到图、或距上一帧已超时才是真异常。
            received = self._last_image_received
            timeout = float(self._params['perception.image_timeout_s'])
            topic = self._image_sub.topic_name
            if received is None:
                throttled(self._log, 'warning',
                          '尚未收到任何图像（话题 %s）' % topic,
                          'aruco.no_image_never', 2.0)
            elif time.monotonic() - received > timeout:
                if not self._loss_announced:
                    self._publish(None, self.get_clock().now().to_msg())
                    self._loss_announced = True
                throttled(self._log, 'warning',
                          '%.1fs 未收到新图像（话题 %s）'
                          % (time.monotonic() - received, topic),
                          'aruco.no_image_stale', 2.0)
            return
        self._latest = None                 # 只处理最新帧，旧的直接丢
        try:
            gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except (CvBridgeError, TypeError, ValueError, cv2.error) as exc:
            throttled(self._log, 'error', '图像转换失败: %r' % (exc,),
                       'aruco.conv_err')
            self._publish(None, msg.header.stamp)
            return
        self._n_frame += 1

        corners, ids = self._detect(gray)
        chosen = self._pick(corners, ids)
        if chosen is None:
            if bool(self._params['perception.publish_empty_on_lost']):
                self._publish(None, msg.header.stamp)
            return

        if bool(self._params['perception.do_subpix']):
            chosen = self._subpix(gray, chosen)
        self._n_det += 1
        self._publish(chosen, msg.header.stamp)

    # -- 检测 -----------------------------------------------------------------
    def _detect(self, gray: np.ndarray):
        """返回 (corners, ids)；无检测时 ids is None。"""
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._detector)
            return corners, ids
        except cv2.error as exc:
            # 坏图像可能令 OpenCV 拒绝检测；按无观测处理，而非沿用旧目标。
            throttled(self._log, 'error', 'detectMarkers 异常: %r' % (exc,),
                       'aruco.detect_err')
            return [], None

    def _pick(self, corners: Any, ids: Any) -> Optional[np.ndarray]:
        """按 target_digit 挑出目标码的 4 个角点（左上→右上→右下→左下）。"""
        if ids is None or len(corners) == 0:
            return None
        try:
            flat = np.asarray(ids).reshape(-1)
            for k, mid in enumerate(flat):
                if int(mid) == int(self._target_digit):
                    c = np.asarray(corners[k], dtype=np.float64).reshape(-1, 2)
                    if c.shape == (4, 2) and np.all(np.isfinite(c)):
                        return c.copy()
        except (TypeError, ValueError):
            # 外部检测结果形状/数值非法时，不能把它传给 PnP。
            return None
        return None

    def _subpix(self, gray: np.ndarray, c: np.ndarray) -> np.ndarray:
        """亚像素精化角点（整像素角点做 PnP 误差明显）。"""
        try:
            win = int(self._params['perception.subpix_win'])
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            refined = cv2.cornerSubPix(gray, c.reshape(-1, 1, 2).astype(np.float32),
                                       (win, win), (-1, -1), crit)
            refined = np.asarray(refined, dtype=np.float64).reshape(-1, 2)
            if (refined.shape != (4, 2) or not np.all(np.isfinite(refined))
                    or np.max(np.abs(refined - c)) > win + 1):
                return c
            return refined
        except cv2.error:
            # 边缘角点可能无法精化；保留本次实际检测的原始角点。
            return c

    # -- 发布 -----------------------------------------------------------------
    def _publish(self, corners: Optional[np.ndarray], stamp: Any) -> None:
        """Publish ID and corners atomically; None explicitly clears tracking."""
        msg = MarkerCorners()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self._params['perception.corners_frame_id'])
        msg.marker_id = int(self._target_digit)
        msg.valid = corners is not None
        if corners is not None:
            for k in range(4):
                p = Point32()
                p.x = float(corners[k, 0])
                p.y = float(corners[k, 1])
                p.z = 0.0
                msg.corners.append(p)
        self._pub.publish(msg)

    def shutdown(self) -> None:
        """退出日志。"""
        rate = (100.0 * self._n_det / self._n_frame) if self._n_frame else 0.0
        self._log.info(f'aruco_detector_node 退出: 处理 {self._n_frame} 帧，'
                       f'命中 {self._n_det} 帧（{rate:.1f}%）')


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # Ctrl+C 是正常停机请求；finally 仍执行停止/关闭资源。
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
