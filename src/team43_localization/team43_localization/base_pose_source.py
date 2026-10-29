"""底盘位姿源（DESIGN §5.4 / §6.9，⚠ 修正 3：必须可插拔）。

仿真里根本没有底盘和 ``/odom``，而 CSV 第 5/6 列需要底盘位置，因此提供：

* :class:`OdomTopicSource`：订阅 ``/odom``（正式赛）。
* :class:`MockSource`：内置可配置底盘运动（仿真默认）。

``MockSource`` 支持的运动（``τ`` = 自 t0 起的**单调**秒）:

| motion | 公式 |
|---|---|
| ``static`` | ``x=x0, y=y0, yaw=yaw0`` |
| ``sine`` | ``x = x0 + amp_x·sin(2π·freq·τ)``；``y = y0 + amp_y·sin(π·freq·τ)``；``yaw = yaw0`` |
| ``circle`` | ``x = x0 + R·cos(2π·freq·τ)``；``y = y0 + R·sin(2π·freq·τ)``；``yaw = yaw0 + 2π·freq·τ`` |

本模块**不 import rclpy / nav_msgs**：``on_odom`` 用鸭子类型读取
``msg.pose.pose.position/orientation`` 与 ``msg.twist.twist.linear``，
因此可在无 ROS 环境直接单测。
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Sequence

import numpy as np

from team43_common.frames import Pose2D, freeze_T_map_odom
from .odometry_geometry import ConvertedOdometry, convert_odometry, rigid_transform


_TWO_PI: float = 2.0 * math.pi


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数 (x,y,z,w) 取绕 z 的 yaw。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def T_map_world_from_odom(x: float, y: float, yaw: float,
                          T_world_odom: Optional[np.ndarray] = None) -> np.ndarray:
    """由 t0 底盘位姿（odom 系）求 ``T_map←world``（4x4）。

    ``T_map←odom = Translate(−P) ∘ RotZ(−yaw)``；``T_map←world = T_map←odom · T_odom←world``。

    Args:
        x: t0 时刻底盘 x（odom 系）。
        y: t0 时刻底盘 y（odom 系）。
        yaw: t0 时刻底盘朝向（odom 系）。
        T_world_odom: (4,4) ``T_world←odom``；None 表示单位阵。

    Returns:
        np.ndarray: (4,4) ``T_map←world``。
    """
    T_map_odom = freeze_T_map_odom(x, y, yaw)
    if T_world_odom is None:
        return T_map_odom
    T_wo = np.asarray(T_world_odom, dtype=np.float64).reshape(4, 4)
    T_ow = np.eye(4, dtype=np.float64)
    T_ow[0:3, 0:3] = T_wo[0:3, 0:3].T
    T_ow[0:3, 3] = -T_wo[0:3, 0:3].T @ T_wo[0:3, 3]
    return T_map_odom @ T_ow


class BasePoseSource(ABC):
    """底盘位姿源抽象基类。"""

    def __init__(self, frame_id: str = 'odom', child_frame_id: str = 'base_link') -> None:
        self.frame_id: str = str(frame_id)
        self.child_frame_id: str = str(child_frame_id)
        self.latest: Optional[ConvertedOdometry] = None

    @abstractmethod
    def poll(self, t_mono: float) -> Pose2D:
        """取当前底盘位姿。

        Args:
            t_mono: 当前单调时刻（秒）。

        Returns:
            Pose2D: 底盘位姿；无有效数据时 ``valid=False``。
        """

    @abstractmethod
    def name(self) -> str:
        """返回源名称（用于日志）。"""

    def reset(self) -> None:
        """重置内部状态（可选实现）。"""


class OdomTopicSource(BasePoseSource):
    """订阅 ``/odom`` 的底盘位姿源（正式赛）。"""

    def __init__(self, topic: str = '/odom', timeout_s: float = 0.5,
                 frame_id: str = 'odom', child_frame_id: str = 'base_link',
                 output_frame_id: str = 'odom', base_frame_id: str = 'base_link',
                 sensor_in_base_xyz=(0., 0., 0.), sensor_in_base_rpy=(0., 0., 0.),
                 sensor_extrinsics_confirmed: bool = False,
                 source_parent_in_output_xyz=(0., 0., 0.),
                 source_parent_in_output_rpy=(0., 0., 0.),
                 parent_transform_confirmed: bool = False,
                 twist_source: str = 'message') -> None:
        super().__init__(output_frame_id, base_frame_id)
        self.topic = str(topic)
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError('odom_timeout_s must be positive')
        self.expected_frame_id = str(frame_id)
        self.expected_child_frame_id = str(child_frame_id)
        if not all((self.expected_frame_id, self.expected_child_frame_id,
                    self.frame_id, self.child_frame_id)):
            raise ValueError('odometry frame names must be explicit and nonempty')
        if self.expected_child_frame_id != self.child_frame_id and not sensor_extrinsics_confirmed:
            raise ValueError('sensor/base frames differ: confirm measured sensor_in_base extrinsics first')
        if self.expected_frame_id != self.frame_id and not parent_transform_confirmed:
            raise ValueError('parent frames differ: explicitly confirm source_parent_in_output transform first')
        self.T_base_sensor = rigid_transform(sensor_in_base_xyz, sensor_in_base_rpy)
        self.T_output_parent = rigid_transform(source_parent_in_output_xyz, source_parent_in_output_rpy)
        if self.expected_child_frame_id == self.child_frame_id and not np.allclose(self.T_base_sensor, np.eye(4)):
            raise ValueError('same sensor/base frame cannot have nonidentity extrinsics')
        if self.expected_frame_id == self.frame_id and not np.allclose(self.T_output_parent, np.eye(4)):
            raise ValueError('same parent frame cannot have nonidentity transform')
        self.twist_source = str(twist_source)
        if self.twist_source not in ('message', 'pose_difference'):
            raise ValueError('odom_twist_source must be message or pose_difference')
        self._last: Optional[Pose2D] = None
        self._last_t_mono = -math.inf
        self._n_msgs = 0
        self._invalid = False

    def name(self) -> str:
        return f'odom_topic({self.topic})'

    def on_odom(self, msg: Any, t_mono: Optional[float] = None) -> None:
        """Validate the declared frames, convert full 3D pose, retain source stamp.

        Receiving the same source stamp again never renews freshness. A frame
        mismatch or backwards stamp invalidates the current output until a new,
        consistent sample arrives. ROS clock age is additionally checked by the
        provider; this source uses monotonic receive time only for transport age.
        """
        import time as _time
        t = _time.monotonic() if t_mono is None else float(t_mono)
        try:
            if str(msg.header.frame_id) != self.expected_frame_id:
                raise ValueError(f'expected parent {self.expected_frame_id!r}, got {msg.header.frame_id!r}')
            if str(msg.child_frame_id) != self.expected_child_frame_id:
                raise ValueError(f'expected child {self.expected_child_frame_id!r}, got {msg.child_frame_id!r}')
            converted = convert_odometry(msg, self.T_base_sensor, self.T_output_parent,
                                         self.twist_source, self.latest)
            stamp = (converted.stamp_sec, converted.stamp_nanosec)
            previous_stamp = None if self.latest is None else (self.latest.stamp_sec, self.latest.stamp_nanosec)
            if stamp == previous_stamp:
                return
            if previous_stamp is not None and stamp < previous_stamp:
                raise ValueError('odometry stamp moved backwards; reset source before a new replay')
        except (TypeError, ValueError):
            # A rejected external sample invalidates cached output until recovery.
            self._invalid = True
            raise
        self.latest = converted
        position = converted.T_output_base[:3, 3]
        velocity = converted.linear_output
        self._last = Pose2D(x=float(position[0]), y=float(position[1]),
                            yaw=converted.yaw, vx=float(velocity[0]),
                            vy=float(velocity[1]), valid=True, t_mono=t)
        self._last_t_mono = t
        self._invalid = False
        self._n_msgs += 1

    def poll(self, t_mono: float) -> Pose2D:
        """返回最近一条 odom；超过 ``timeout_s`` 视为失效。"""
        if self._last is None or self._invalid:
            return Pose2D(valid=False, t_mono=float(t_mono))
        if (float(t_mono) - self._last_t_mono) > self.timeout_s:
            stale = Pose2D(x=self._last.x, y=self._last.y, yaw=self._last.yaw,
                           vx=0.0, vy=0.0, valid=False, t_mono=float(t_mono))
            return stale
        out = Pose2D(x=self._last.x, y=self._last.y, yaw=self._last.yaw,
                     vx=self._last.vx, vy=self._last.vy, valid=True,
                     t_mono=self._last_t_mono)
        return out

    @property
    def n_msgs(self) -> int:
        """已收到的 odom 消息数。"""
        return self._n_msgs

    def reset(self) -> None:
        self._last = None
        self.latest = None
        self._invalid = False
        self._last_t_mono = -math.inf


class MockSource(BasePoseSource):
    """内置可配置底盘运动（仿真默认，DESIGN §5.4）。"""

    def __init__(self,
                 motion: str = 'static',
                 origin_xy: Sequence[float] = (0.0, 0.0),
                 yaw0: float = 0.0,
                 amp_xy: Sequence[float] = (1.0, 0.5),
                 radius: float = 1.5,
                 freq: float = 0.1,
                 t0: Optional[float] = None,
                 frame_id: str = 'odom',
                 child_frame_id: str = 'base_link') -> None:
        super().__init__(frame_id, child_frame_id)
        self.motion: str = str(motion).lower()
        if self.motion not in ('static', 'sine', 'circle'):
            raise ValueError(f'未知 mock_motion: {motion!r}（可选 static|sine|circle）')
        o = np.asarray(origin_xy, dtype=np.float64).reshape(-1)
        self.x0: float = float(o[0]) if o.size > 0 else 0.0
        self.y0: float = float(o[1]) if o.size > 1 else 0.0
        self.yaw0: float = float(yaw0)
        a = np.asarray(amp_xy, dtype=np.float64).reshape(-1)
        self.amp_x: float = float(a[0]) if a.size > 0 else 0.0
        self.amp_y: float = float(a[1]) if a.size > 1 else 0.0
        self.radius: float = float(radius)
        self.freq: float = float(freq)
        self._t0: Optional[float] = None if t0 is None else float(t0)

    def name(self) -> str:
        return f'mock({self.motion})'

    def set_t0(self, t0: float) -> None:
        """设定运动起算时刻（单调秒）。"""
        self._t0 = float(t0)

    def poll(self, t_mono: float) -> Pose2D:
        """按当前单调时刻生成位姿（含解析速度）。"""
        t = float(t_mono)
        if self._t0 is None:
            self._t0 = t
        tau = max(0.0, t - self._t0)
        w = _TWO_PI * self.freq
        if self.motion == 'static':
            return Pose2D(x=self.x0, y=self.y0, yaw=self.yaw0,
                          vx=0.0, vy=0.0, valid=True, t_mono=t)
        if self.motion == 'sine':
            ph_x = w * tau
            ph_y = math.pi * self.freq * tau
            x = self.x0 + self.amp_x * math.sin(ph_x)
            y = self.y0 + self.amp_y * math.sin(ph_y)
            vx = self.amp_x * w * math.cos(ph_x)
            vy = self.amp_y * math.pi * self.freq * math.cos(ph_y)
            return Pose2D(x=x, y=y, yaw=self.yaw0, vx=vx, vy=vy, valid=True, t_mono=t)
        # circle
        ph = w * tau
        x = self.x0 + self.radius * math.cos(ph)
        y = self.y0 + self.radius * math.sin(ph)
        vx = -self.radius * w * math.sin(ph)
        vy = self.radius * w * math.cos(ph)
        yaw = self.yaw0 + ph
        return Pose2D(x=x, y=y, yaw=yaw, vx=vx, vy=vy, valid=True, t_mono=t)

    def reset(self) -> None:
        self._t0 = None


def make_base_pose_source(cfg: Dict[str, Any]) -> BasePoseSource:
    """按参数字典构造底盘位姿源（工厂）。

    Args:
        cfg: 至少包含 ``base_pose_source``（``odom`` 或 ``mock``）；
             ``odom`` 用 ``odom_topic`` / ``odom_timeout_s``；
             ``mock`` 用 ``mock_motion`` / ``mock_origin_xy`` / ``mock_yaw0`` /
             ``mock_amp_xy`` / ``mock_radius`` / ``mock_freq``。

    Returns:
        BasePoseSource: 具体源实例。

    Raises:
        ValueError: ``base_pose_source`` 取值非法。
    """
    kind = str(cfg.get('base_pose_source', 'mock')).lower()
    if kind == 'odom':
        return OdomTopicSource(
            topic=str(cfg.get('odom_topic', '/odom')),
            timeout_s=float(cfg.get('odom_timeout_s', 0.5)),
            frame_id=str(cfg.get('odom_frame_id', 'odom')),
            child_frame_id=str(cfg.get('odom_child_frame_id', 'base_link')),
            output_frame_id=str(cfg.get('output_frame_id', 'odom')),
            base_frame_id=str(cfg.get('base_frame_id', 'base_link')),
            sensor_in_base_xyz=cfg.get('sensor_in_base_xyz', (0., 0., 0.)),
            sensor_in_base_rpy=cfg.get('sensor_in_base_rpy', (0., 0., 0.)),
            sensor_extrinsics_confirmed=bool(cfg.get('sensor_extrinsics_confirmed', False)),
            source_parent_in_output_xyz=cfg.get('source_parent_in_output_xyz', (0., 0., 0.)),
            source_parent_in_output_rpy=cfg.get('source_parent_in_output_rpy', (0., 0., 0.)),
            parent_transform_confirmed=bool(cfg.get('parent_transform_confirmed', False)),
            twist_source=str(cfg.get('odom_twist_source', 'message')),
        )
    if kind == 'mock':
        return MockSource(
            motion=str(cfg.get('mock_motion', 'static')),
            origin_xy=cfg.get('mock_origin_xy', (0.0, 0.0)),
            yaw0=float(cfg.get('mock_yaw0', 0.0)),
            amp_xy=cfg.get('mock_amp_xy', (1.0, 0.5)),
            radius=float(cfg.get('mock_radius', 1.5)),
            freq=float(cfg.get('mock_freq', 0.1)),
        )
    raise ValueError(f'未知 base_pose_source: {kind!r}（可选 odom|mock）')
