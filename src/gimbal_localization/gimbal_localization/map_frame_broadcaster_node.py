"""map 系（给定点）冻结与广播节点（DESIGN §2.1 / §3.6 / §5.4）。

行为：

1. 订阅 ``/gimbal/base_pose_raw``（odom 系）；
2. **赛题一**：``localization.origin_mode='judge_replay_start'`` 时，等
   ``/judge/replay_start``（经 ``gimbal_judge_io`` 转成锁存话题 ``/gimbal/origin_freeze``）
   到达即冻结 t0 —— 给定点 = 回放起点时底盘的位姿（原点在底盘中心，x 轴朝车头，y 轴朝左）；
   超时 ``origin_timeout_s``（默认 5 s）未收到则回退到 ``base_init`` 的 settle 逻辑；
3. ``origin_mode='base_init'``（旧行为）：首个有效位姿后再等 ``origin_settle_s``（0.5s）→ **t0 冻结**：
   ``T_map←odom = Translate(−P_base(t0)) ∘ RotZ(−yaw_base(t0))``，之后永不更新；
3. 发布静态 TF ``map→odom``（t0 冻结）与 ``world→odom``（来自 ``world_odom_xyz_rpy``，默认单位阵）；
4. 发布**锁存**话题 ``/gimbal/t_map_world``（``TransformStamped``，parent=``map``，
   child=``world``，即 ``T_map←world``，QoS TRANSIENT_LOCAL depth=1）；
5. 50Hz 发布 ``/gimbal/base_pose``（``nav_msgs/Odometry``，``frame_id=map``）。

⚠ ``T_map←world`` **只发话题，不发 TF**：TF 树里 world 已经是 root（外部发布
``world→gimbal_link_gt``），再发 ``map→world`` 会产生环路。算法侧一律用话题拿矩阵。

⚠ 时钟：TF/话题 stamp 用 ROS 系统时钟；t0 判定与 settle 计时用单调时钟。
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from gimbal_common import frames as FR
from gimbal_common import log_utils as LU
from gimbal_common import node_utils as NU
from gimbal_common import time_utils as TU

from .base_pose_source import T_map_world_from_odom, quat_yaw

NODE_NAME = 'map_frame_broadcaster_node'

PARAM_DEFAULTS = {
    'localization.origin_mode': 'judge_replay_start',   # 赛题一：由 /judge/replay_start 冻结
    'localization.origin_timeout_s': 5.0,               # 超时未收到 → 回退 base_init
    'localization.origin_settle_s': 0.5,
    'localization.publish_map_odom': True,
    'localization.publish_world_odom': True,
    'localization.world_odom_xyz_rpy': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'localization.gimbal_mount_xyz_rpy': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'localization.base_pose_rate_hz': 50.0,
}


def _sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _xyz_rpy_to_T(xyz_rpy) -> np.ndarray:
    """``[x,y,z,roll,pitch,yaw]`` → (4,4) 齐次变换（复用 ``frames.rpy_to_mat``，单一实现）。"""
    v = [float(x) for x in list(xyz_rpy)[:6]]
    while len(v) < 6:
        v.append(0.0)
    return FR.compose_T(FR.rpy_to_mat(v), np.array(v[0:3], dtype=np.float64))


class MapFrameBroadcasterNode:
    """map 系冻结与广播节点。"""

    def __init__(self, node) -> None:
        self._node = node
        self._log = LU.get_logger(node, 'map_frame')
        self._params = NU.declare_params(node, PARAM_DEFAULTS)
        p = self._params

        self._origin_mode: str = str(p['localization.origin_mode'])
        self._settle_s: float = float(p['localization.origin_settle_s'])
        self._origin_timeout_s: float = float(p['localization.origin_timeout_s'])
        self._T_world_odom: np.ndarray = _xyz_rpy_to_T(p['localization.world_odom_xyz_rpy'])

        self._frozen: bool = False
        self._T_map_world: Optional[np.ndarray] = None
        self._T_map_odom: Optional[np.ndarray] = None
        self._t_first_valid: Optional[float] = None
        self._last_pose_odom = None          # (x, y, yaw, vx, vy)
        self._n_pose: int = 0
        self._freeze_requested: bool = False  # /judge/replay_start 已到达

        qos = _sensor_qos(10)
        self._sub = node.create_subscription(Odometry, '/gimbal/base_pose_raw',
                                             self._on_base_pose, qos)
        if self._origin_mode == 'judge_replay_start':
            # gimbal_judge_io 把 /judge/replay_start 转成锁存话题（绝不直接订阅 /judge/
            # 之外的东西，也避免 QoS 写错拿不到）
            node.create_subscription(Header, '/gimbal/origin_freeze',
                                     self._on_origin_freeze, _latched_qos(1))
        self._pub_t = node.create_publisher(TransformStamped, '/gimbal/t_map_world',
                                            _latched_qos(1))
        self._pub_pose = node.create_publisher(Odometry, '/gimbal/base_pose', qos)
        self._static_tf = StaticTransformBroadcaster(node)

        # world→odom 静态 TF（默认单位阵；实机由标定参数给出）
        if bool(p['localization.publish_world_odom']):
            self._send_static(self._T_world_odom, 'world', 'odom')
            self._log.info('已发布静态 TF world→odom')
        self._publish_map_odom: bool = bool(p['localization.publish_map_odom'])

        rate = max(1.0, float(p['localization.base_pose_rate_hz']))
        node.create_timer(1.0 / rate, self._tick)
        self._log.info(f'map 系广播器启动：origin_mode={self._origin_mode}，'
                       f'settle={self._settle_s}s，timeout={self._origin_timeout_s}s，'
                       f'{rate:.0f}Hz')

    # ------------------------------------------------------------------ 订阅
    def _on_base_pose(self, msg: Odometry) -> None:
        """缓存最新底盘位姿；到点后冻结 map 系。

        外部话题的 ``header.stamp`` 只用于日志，冻结判据一律用本地单调时刻。
        """
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            x, y = float(pos.x), float(pos.y)
            yaw = quat_yaw(float(ori.x), float(ori.y), float(ori.z), float(ori.w))
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
        except AttributeError as exc:
            LU.throttled(self._log, 'error', f'base_pose_raw 结构异常: {exc!r}', 'bp_bad', 1.0)
            return
        if not all(math.isfinite(v) for v in (x, y, yaw, vx, vy)):
            return
        t_mono = TU.now()
        self._n_pose += 1
        self._last_pose_odom = (x, y, yaw, vx, vy)
        if self._frozen:
            return
        if self._t_first_valid is None:
            self._t_first_valid = t_mono
            return

        if self._origin_mode == 'judge_replay_start':
            if self._freeze_requested:
                self._freeze(x, y, yaw, t_mono)
            elif (t_mono - self._t_first_valid) >= self._origin_timeout_s:
                self._log.warning(
                    '等待 /judge/replay_start 超时 %.1fs，回退 base_init 冻结'
                    % self._origin_timeout_s)
                self._origin_mode = 'base_init'
            return

        if (t_mono - self._t_first_valid) >= self._settle_s:
            self._freeze(x, y, yaw, t_mono)

    # ------------------------------------------------------------- 给定点信号
    def _on_origin_freeze(self, _msg: Header) -> None:
        """``/judge/replay_start`` 到达 → 用**当时**的底盘位姿冻结给定点。

        回放起点信号与第 0 拍同时发布，此刻的底盘位姿就是给定点，
        之后 map 系永不更新。
        """
        self._freeze_requested = True
        if self._frozen:
            return
        if self._last_pose_odom is None:
            self._log.warning('收到 replay_start，但还没有底盘位姿，等下一帧再冻结')
            return
        x, y, yaw, _vx, _vy = self._last_pose_odom
        self._freeze(x, y, yaw, TU.now())

    # ------------------------------------------------------------------ 冻结
    def _freeze(self, x: float, y: float, yaw: float, t_mono: float) -> None:
        """t0 冻结：只做一次，之后永不更新。"""
        if self._frozen:
            return
        if self._origin_mode == 'world':
            self._T_map_world = np.eye(4, dtype=np.float64)
            self._T_map_odom = self._T_world_odom.copy()
        else:
            if self._origin_mode not in ('base_init', 'judge_replay_start'):
                self._log.warning(f'origin_mode={self._origin_mode!r} 不认识，'
                                  '本次退化为 base_init')
            # base_init 与 judge_replay_start 的冻结公式相同：
            # 给定点 = 那一刻的底盘位姿（原点在底盘中心，x 朝车头，y 朝左）
            self._T_map_world = T_map_world_from_odom(x, y, yaw, self._T_world_odom)
            self._T_map_odom = FR.freeze_T_map_odom(x, y, yaw)
        self._frozen = True
        self._log.info(
            f't0 冻结 map 系：base_odom(t0)=({x:.3f}, {y:.3f}, yaw={yaw:.3f})，'
            f'T_map←world 平移=({self._T_map_world[0, 3]:.3f}, '
            f'{self._T_map_world[1, 3]:.3f})，此后永不更新')
        self._publish_latched()
        if self._publish_map_odom:
            self._send_static(self._T_map_odom, 'map', 'odom')
            self._log.info('已发布静态 TF map→odom（t0 冻结）')

    def _publish_latched(self) -> None:
        """发布锁存话题 ``/gimbal/t_map_world``（只发一次，TRANSIENT_LOCAL）。"""
        if self._T_map_world is None:
            return
        T = self._T_map_world
        tf = TransformStamped()
        tf.header.stamp = self._node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'world'
        tf.transform.translation.x = float(T[0, 3])
        tf.transform.translation.y = float(T[1, 3])
        tf.transform.translation.z = float(T[2, 3])
        q = FR.quat_from_mat(T[0:3, 0:3])
        tf.transform.rotation.w = float(q[0])
        tf.transform.rotation.x = float(q[1])
        tf.transform.rotation.y = float(q[2])
        tf.transform.rotation.z = float(q[3])
        self._pub_t.publish(tf)

    def _send_static(self, T: np.ndarray, parent: str, child: str) -> None:
        """发布一条静态 TF。"""
        tf = TransformStamped()
        tf.header.stamp = self._node.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(T[0, 3])
        tf.transform.translation.y = float(T[1, 3])
        tf.transform.translation.z = float(T[2, 3])
        q = FR.quat_from_mat(T[0:3, 0:3])
        tf.transform.rotation.w = float(q[0])
        tf.transform.rotation.x = float(q[1])
        tf.transform.rotation.y = float(q[2])
        tf.transform.rotation.z = float(q[3])
        try:
            self._static_tf.sendTransform(tf)
        except Exception as exc:
            LU.throttled(self._log, 'warning', f'静态 TF {parent}->{child} 发布失败: {exc!r}',
                         'static_tf', 1.0)

    # ------------------------------------------------------------------ 定时
    def _tick(self) -> None:
        """50Hz 发布 map 系底盘位姿（冻结后才发）。"""
        try:
            if not self._frozen or self._last_pose_odom is None or self._T_map_world is None:
                if not self._frozen:
                    LU.throttled(self._log, 'warning',
                                 'map 系尚未冻结（等 base_pose_raw + settle）',
                                 'not_frozen', 2.0)
                return
            x, y, yaw, vx, vy = self._last_pose_odom
            p_map = self._T_map_world @ (self._T_world_odom @ np.array([x, y, 0.0, 1.0]))
            R2 = self._T_map_world[0:2, 0:2] @ self._T_world_odom[0:2, 0:2]
            v_map = R2 @ np.array([vx, vy])
            yaw_map = yaw + math.atan2(self._T_map_world[1, 0], self._T_map_world[0, 0])

            msg = Odometry()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = float(p_map[0])
            msg.pose.pose.position.y = float(p_map[1])
            msg.pose.pose.position.z = 0.0
            cy = math.cos(0.5 * yaw_map)
            sy = math.sin(0.5 * yaw_map)
            msg.pose.pose.orientation.z = sy
            msg.pose.pose.orientation.w = cy
            msg.twist.twist.linear.x = float(v_map[0])
            msg.twist.twist.linear.y = float(v_map[1])
            self._pub_pose.publish(msg)
        except Exception as exc:
            LU.throttled(self._log, 'error', f'map tick 异常: {exc!r}', 'map_tick', 1.0)

    @property
    def frozen(self) -> bool:
        """map 系是否已冻结。"""
        return self._frozen

    @property
    def T_map_world(self) -> Optional[np.ndarray]:
        """冻结得到的 ``T_map←world``（未冻结时为 None）。"""
        return None if self._T_map_world is None else self._T_map_world.copy()

    def shutdown(self) -> None:
        """收尾日志。"""
        if self._frozen:
            self._log.info(f'map 系广播器退出，T_map←world 已冻结（收到 {self._n_pose} 帧位姿）')
        else:
            self._log.warning(f'map 系广播器退出：始终未冻结（收到 {self._n_pose} 帧位姿）')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = NU.create_node(NODE_NAME)
    app = MapFrameBroadcasterNode(node)
    try:
        NU.spin_with_finally(node, on_shutdown=app.shutdown)
    finally:
        node.destroy_node()
        NU.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
