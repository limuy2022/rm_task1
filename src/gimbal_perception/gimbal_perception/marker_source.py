"""marker 观测适配层（DESIGN §6.4，⚠ 修正 4：多 marker 适配降级到 P1）。

P0 只实现 :class:`SingleTopicMarkerSource`：

* 输入 ``geometry_msgs/PolygonStamped``，像素在 ``points[i].x / .y``，
  顺序 **左上 → 右上 → 右下 → 左下**，``frame_id = camera_optical_frame``。
* 空 polygon 或长度 ≠ 4 → 视为**本帧无观测**（正常路径，不报错）。
* 过滤：`id_filter_mode=off`（默认）接受唯一观测；`on` 要求 ``marker_id == target_digit``。

``MultiTopicSource`` / ``ImageDetectSource`` 为 P1 预留的抽象，**P0 不实现**。

本模块不 import ROS 消息类型（鸭子类型读取 ``points``），可在无 ROS 环境单测。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence

import numpy as np

__all__ = ['RawMarker', 'MarkerSource', 'SingleTopicMarkerSource',
           'MultiTopicSource', 'ImageDetectSource', 'make_marker_source']


@dataclass
class RawMarker:
    """一次原始 marker 观测。"""

    marker_id: int = 0
    corners_uv: np.ndarray = field(default_factory=lambda: np.zeros((4, 2), dtype=np.float64))
    t_mono: float = 0.0
    frame_id: str = 'camera_optical_frame'
    valid: bool = True

    @property
    def n_corners(self) -> int:
        """角点个数。"""
        return int(np.asarray(self.corners_uv).shape[0])


class MarkerSource(ABC):
    """marker 观测源抽象基类。"""

    @abstractmethod
    def poll(self) -> List[RawMarker]:
        """取出自上次调用以来累积的观测（取完即清空）。"""

    @abstractmethod
    def name(self) -> str:
        """源名称（用于日志）。"""

    def reset(self) -> None:
        """清空待处理队列（可选实现）。"""


class SingleTopicMarkerSource(MarkerSource):
    """单话题单 marker 源（P0 唯一实现）。"""

    def __init__(self, target_digit: int = 0, id_filter_mode: str = 'off') -> None:
        """构造。

        Args:
            target_digit: 目标数字（与仿真 ``marker_id`` 一致，默认 0）。
            id_filter_mode: ``off``（接受唯一观测）| ``on``（要求 id 相等）。
        """
        self.target_digit: int = int(target_digit)
        self.id_filter_mode: str = str(id_filter_mode).lower()
        if self.id_filter_mode not in ('off', 'on'):
            raise ValueError(f'未知 id_filter_mode: {id_filter_mode!r}（可选 off|on）')
        self._pending: List[RawMarker] = []
        self._n_seen: int = 0
        self._n_filtered: int = 0
        self._n_malformed: int = 0

    def name(self) -> str:
        return f'single_topic(target={self.target_digit}, filter={self.id_filter_mode})'

    def on_polygon(self, msg: Any, marker_id: int = 0,
                   t_mono: Optional[float] = None) -> Optional[RawMarker]:
        """喂入一条 ``geometry_msgs/PolygonStamped``（鸭子类型）。

        Args:
            msg: 具有 ``points``（元素含 ``x``/``y``）与 ``header.frame_id`` 的消息。
            marker_id: 该观测的 marker id（话题本身不带 id 时由外部给出）。
            t_mono: 本地收到时刻（单调秒）；None 时取当前单调时刻。

        Returns:
            RawMarker: 通过过滤的观测；被过滤/畸形时返回 None（**不算错误**）。
        """
        if msg is None:
            return None
        if t_mono is None:
            import time as _time
            t_mono = _time.monotonic()
        try:
            pts = list(msg.points)
            frame_id = str(getattr(getattr(msg, 'header', None), 'frame_id', '')
                           or 'camera_optical_frame')
        except AttributeError:
            self._n_malformed += 1
            return None
        if len(pts) != 4:
            # 空 polygon / 长度不为 4 → 本帧无观测，正常路径
            self._n_malformed += 1
            return None
        uv = np.array([[float(p.x), float(p.y)] for p in pts], dtype=np.float64).reshape(4, 2)
        if not np.all(np.isfinite(uv)):
            self._n_malformed += 1
            return None
        self._n_seen += 1
        if self.id_filter_mode == 'on' and int(marker_id) != self.target_digit:
            self._n_filtered += 1
            return None
        marker = RawMarker(marker_id=int(marker_id), corners_uv=uv,
                           t_mono=float(t_mono), frame_id=frame_id, valid=True)
        self._pending.append(marker)
        return marker

    def poll(self) -> List[RawMarker]:
        """取出并清空待处理观测。"""
        out, self._pending = self._pending, []
        return out

    def reset(self) -> None:
        self._pending = []

    @property
    def stats(self) -> dict:
        """返回过滤统计。"""
        return {'seen': int(self._n_seen), 'filtered': int(self._n_filtered),
                'malformed': int(self._n_malformed)}


class MultiTopicSource(MarkerSource):
    """多话题多 marker 源（**P1，P0 不实现**）。

    DESIGN §6.4 修正 4：多形态适配降级到 P1。这里只保留抽象，调用会明确报错，
    避免 P0 阶段误以为已支持多 marker。
    """

    def __init__(self, topics: Sequence[str] = (), target_digit: int = 0) -> None:
        self.topics = list(topics)
        self.target_digit = int(target_digit)

    def name(self) -> str:
        return 'multi_topic(P1_未实现)'

    def poll(self) -> List[RawMarker]:
        raise NotImplementedError('MultiTopicSource 属于 P1，P0 阶段请使用 SingleTopicMarkerSource')


class ImageDetectSource(MarkerSource):
    """直接检测图像的 marker 源（**P1，P0 不实现**）。

    赛事图像由外部 ``virtual_vision`` 给出角点，P0 不需要自己做检测。
    """

    def __init__(self, dictionary: str = 'DICT_4X4_50', target_digit: int = 0) -> None:
        self.dictionary = str(dictionary)
        self.target_digit = int(target_digit)

    def name(self) -> str:
        return 'image_detect(P1_未实现)'

    def poll(self) -> List[RawMarker]:
        raise NotImplementedError('ImageDetectSource 属于 P1，P0 阶段请使用 SingleTopicMarkerSource')


def make_marker_source(kind: str = 'single_topic', **kwargs: Any) -> MarkerSource:
    """marker 源工厂。

    Args:
        kind: ``single_topic``（P0）| ``multi_topic`` / ``image_detect``（P1，不可实例化使用）。
        **kwargs: 透传给具体源。

    Returns:
        MarkerSource: 具体源实例。
    """
    kind = str(kind).lower()
    if kind == 'single_topic':
        return SingleTopicMarkerSource(
            target_digit=int(kwargs.get('target_digit', 0)),
            id_filter_mode=str(kwargs.get('id_filter_mode', 'off')),
        )
    if kind == 'multi_topic':
        return MultiTopicSource(topics=kwargs.get('topics', ()),
                                target_digit=int(kwargs.get('target_digit', 0)))
    if kind == 'image_detect':
        return ImageDetectSource(dictionary=str(kwargs.get('dictionary', 'DICT_4X4_50')),
                                 target_digit=int(kwargs.get('target_digit', 0)))
    raise ValueError(f'未知 marker_source: {kind!r}')
