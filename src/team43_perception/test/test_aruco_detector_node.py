"""`team43_perception/aruco_detector_node.py` 图像告警语义回归（真 ROS2 Humble）。

手法：**真 rclpy 节点** + 录制型 logger 替身 + 直调 `_on_image`/`_on_timer`。

背景：相机 30 fps 与 detect 定时器 30 Hz 同频拍频，必然周期性出现"本周期
没有新帧"的空 tick；旧实现在任何空 tick 上都打"还没有收到 /image_raw"，
造成图像流正常（跟踪正常）时的持续误报。此处锁定修复后的语义：

- 空 tick（最近刚收到过帧）→ 静默；
- 从未收到图 / 真超时（> ``perception.image_timeout_s``）→ 告警，
  文案带实际订阅的话题名；
- 真超时只补发一次空角点（``_loss_announced`` 语义不变）。
"""

from __future__ import annotations

import os
import sys
import time

import pytest

_PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
_SRC_DIR = os.path.abspath(os.path.join(_PKG_DIR, '..'))
sys.path.insert(0, _PKG_DIR)
sys.path.insert(0, os.path.join(_SRC_DIR, 'team43_common'))

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv_bridge')
pytest.importorskip('team43_interfaces')

from sensor_msgs.msg import Image  # noqa: E402

from team43_common.log_utils import reset_throttle  # noqa: E402

import team43_perception.aruco_detector_node as NODE  # noqa: E402


class _RecordingLogger(object):
    """记录等级调用的 logger 替身（兼容直接调用与 throttled 路径）。"""

    def __init__(self) -> None:
        self.records = []

    def _rec(self, level):
        return lambda message: self.records.append((level, str(message)))

    def debug(self, message):
        self._rec('debug')(message)

    info = property(lambda self: self._rec('info'))
    warning = property(lambda self: self._rec('warning'))
    warn = property(lambda self: self._rec('warning'))
    error = property(lambda self: self._rec('error'))
    fatal = property(lambda self: self._rec('fatal'))


def _image(w: int = 32, h: int = 32) -> Image:
    msg = Image()
    msg.height, msg.width = h, w
    msg.encoding = 'rgb8'
    msg.step = 3 * w
    msg.data = [128] * (3 * w * h)
    return msg


def _warn_texts(recorder: _RecordingLogger) -> list:
    return [m for (lv, m) in recorder.records if lv == 'warning']


@pytest.fixture()
def ros():
    if not rclpy.ok():
        rclpy.init()
    reset_throttle()
    yield
    reset_throttle()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture()
def detector(ros):
    node = NODE.ArucoDetectorNode()
    node._log = _RecordingLogger()  # 替换 logger 以录制告警
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# 空 tick：图像流正常时不得误报
# ---------------------------------------------------------------------------
def test_empty_tick_right_after_frame_is_silent(detector):
    detector._on_image(_image())
    detector._on_timer()                  # 消费该帧
    detector._on_timer()                  # 空 tick：上一帧刚处理过
    assert _warn_texts(detector._log) == []


def test_beat_frequency_empty_ticks_are_silent_when_images_flow(detector):
    """隔帧到达（相机与检测同频拍频）是误报的原始场景，必须保持静默。"""
    for i in range(8):
        if i % 2 == 0:
            detector._on_image(_image())
        detector._on_timer()
    assert [m for m in _warn_texts(detector._log) if '未收到' in m] == []
    assert detector._n_frame == 4         # 帧确实在流动并被处理


# ---------------------------------------------------------------------------
# 真异常：从未收到 / 真超时
# ---------------------------------------------------------------------------
def test_warns_when_no_image_ever_received(detector):
    detector._on_timer()
    warns = _warn_texts(detector._log)
    assert any('尚未收到任何图像' in m for m in warns)
    # 文案必须带实际订阅的话题名，而不是硬编码字符串
    assert any(detector._image_sub.topic_name in m for m in warns)


def test_warns_and_publishes_empty_once_when_image_stale(detector, monkeypatch):
    detector._on_image(_image())
    detector._on_timer()                  # 正常消费一帧
    detector._last_image_received = time.monotonic() - 10.0   # 模拟真断流
    published = []
    monkeypatch.setattr(detector._pub, 'publish', lambda m: published.append(m))
    detector._on_timer()
    detector._on_timer()                  # 第二个 stale tick：不重复补发
    warns = _warn_texts(detector._log)
    assert any('未收到新图像' in m for m in warns)
    assert any(detector._image_sub.topic_name in m for m in warns)
    assert detector._loss_announced is True
    assert len(published) == 1
    assert published[0].valid is False


def test_stale_warning_recovers_after_new_image(detector):
    """断流告警后重新来图：恢复处理且不再触发告警分支。"""
    detector._on_image(_image())
    detector._on_timer()
    detector._last_image_received = time.monotonic() - 10.0
    detector._on_timer()                  # 触发一次真超时告警
    assert any('未收到新图像' in m for m in _warn_texts(detector._log))

    detector._on_image(_image())
    detector._on_timer()
    assert detector._loss_announced is False
    assert detector._n_frame == 2
