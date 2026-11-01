"""chassis_advisor_node：订阅目标状态 / 底盘位姿 / ray_grid → 发布 `/chassis/advice`。

对应 DESIGN §3.9 / §6.10。

订阅（§4.3）：
- `/estimation/target_state`（`team43_interfaces/TargetState`，100Hz，SENSOR_DATA）
- `/gimbal/base_pose`（`nav_msgs/Odometry`，frame=map，50Hz，SENSOR_DATA）
- `/map/ray_grid`（`nav_msgs/OccupancyGrid`，一次性，TRANSIENT_LOCAL depth=1）
- `/map/grid`（`nav_msgs/OccupancyGrid`，**未膨胀**三态，方向项解析求解用）

发布：
- `/chassis/advice`（`team43_interfaces/ChassisAdvice`，50Hz，SENSOR_DATA）

⚠ **赛题一适配（2026-10）**：方向项权重 0.2，标准方向 = "从底盘出发、沿地图可行、
射线最长 1 m、不与不可跨越地形相交、与目标方向夹角最小"的**连续**角度。
旧的 13 方向 12.5° 量化会带来约 3.6° 的角度 RMSE（≈0.004 总分），
改用 `free_dir.FreeDirSolver` 解析求解后可压到 <0.5°。
`chassis.dir_mode='analytic'`（独立节点默认）走解析路径，`'quantized'` 保留离散行为。
比赛direct路径不依赖地图，但默认用新鲜cloud_registered检查圆形车体前瞻碰撞；
点云/TF无效时建议归零，CSV不因缺图而停写。
另外：官方判定 `hypot(vx,vy) < 0.02` 为停车。速度下限仅用于已验证
可行的方向；地图缺失、输入过期、目标丢失或全部受阻时，不会为了分数制造穿墙方向。
解析与量化两种路径均考虑 OccupancyGrid 原点 yaw。

参考系默认 `map`（与 CSV 1~6 列一致）；`cmd_frame=base_link` 时额外左乘 `RotZ(−yaw_base)`（P1）。

时钟（主理人拍板 1，混合时钟）：
- `header.stamp` 用 **ROS 系统时钟** `node.get_clock().now()`（**不替换 node clock**）
- 所有 dt / 超时 / 定时器周期用 `team43_common.time_utils.now()`（= `time.monotonic()`）

运行目标为 Ubuntu 22.04 + ROS2 Humble；纯核离线检查不替代 NUC 通信与实图验证。
"""

from __future__ import annotations

import math
import sys
from typing import Any
from typing import Dict
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from team43_interfaces.msg import ChassisAdvice, TargetState

from team43_common.time_utils import now
from team43_common.log_utils import get_logger, throttled
from team43_common.node_utils import declare_params
from team43_common.qos_utils import latched_qos, sensor_qos

from team43_common.frames import rotz2
from team43_common.frames import quat_to_mat
from team43_grid_mapper.cloud_grid import pointcloud2_xyz

from .cost_raycast import (DIR_SPAN_DEG, LOOKAHEAD_M, N_DIRS, W_ANG, W_OBS,
                           W_UNK, CostRaycast, RayGridAdapter)
from .free_dir import FREE_DIR_L_M, V_STOP_OFFICIAL, FreeDirSolver
from .velocity_law import (A_MAX, D_SLOW, D_STOP, LOST_DECAY_S, LP_CUTOFF_HZ,
                           V_MAX, VelocityLaw)
from .obstacle_guard import guard_velocity

# 默认参数（DESIGN §6.13 `chassis.*`）
DEFAULTS: Dict[str, Any] = {
    'chassis.v_max': V_MAX,
    'chassis.d_slow': D_SLOW,
    'chassis.d_stop': D_STOP,
    'chassis.w_ang': W_ANG,
    'chassis.w_obs': W_OBS,
    'chassis.w_unk': W_UNK,
    'chassis.n_dirs': N_DIRS,
    'chassis.dir_span_deg': DIR_SPAN_DEG,
    'chassis.lookahead_m': LOOKAHEAD_M,
    'chassis.a_max': A_MAX,
    'chassis.lp_cutoff_hz': LP_CUTOFF_HZ,
    'chassis.lost_decay_s': LOST_DECAY_S,
    'chassis.cmd_frame': 'map',           # map | base_link（P1）
    'chassis.rate_hz': 50.0,              # 话题表：/chassis/advice 50Hz
    'chassis.enable_avoidance': True,     # false 时跳过射线，直接朝目标
    # --- 赛题一新增 ---------------------------------------------------------
    'chassis.dir_mode': 'analytic',       # analytic（连续）| quantized（旧 13 方向）
    'chassis.dir_lookahead_m': FREE_DIR_L_M,   # 官方固定 1.0 m
    'chassis.dir_block_unknown': True,    # 未观测区域不等于可通行区域；只建议已知可行方向
    'chassis.dir_use_pred': False,        # False=朝目标**当前**位置（cur），True=朝预测位置
    'chassis.v_floor': 0.30,              # 远离目标时的最小建议速度（>0.02 停车线）
    'chassis.stop_radius_m': 0.45,
    'chassis.frame_id': 'map',
    'chassis.base_frame_id': 'base_link',
    'chassis.base_timeout_s': 0.20,
    'chassis.target_timeout_s': 0.20,
    'chassis.last_observation_timeout_s': 0.50,
    'chassis.future_tolerance_s': 0.05,
    # 静态地图可锁存，0 不按年龄失效；实时建图可显式设 5.0 等超时。
    'chassis.map_timeout_s': 0.0,
    'chassis.obstacle_guard_enabled': False,  # 比赛入口显式开启；独立节点保持原模式。
    'chassis.obstacle_floor_z': 0.0,
    'chassis.obstacle_floor_valid': False,
    'chassis.obstacle_timeout_s': .5,
    'chassis.obstacle_clearance_m': .4,  # 长宽按30cm近似，中心到障碍的保护距离。
    'chassis.obstacle_horizon_s': 1.5,
    'chassis.obstacle_braking_mps2': .5,
}


class ChassisAdvisorNode(Node):  # type: ignore[misc]
    """底盘建议速度节点。"""

    def __init__(self, name: str = 'chassis_advisor_node') -> None:
        super().__init__(name)
        self._log = get_logger(self, name)
        self._params: Dict[str, Any] = declare_params(self, DEFAULTS)

        # --- 算法内核（纯 Python） -------------------------------------------
        self._ray = CostRaycast(
            lookahead_m=float(self._params['chassis.lookahead_m']),
            w_ang=float(self._params['chassis.w_ang']),
            w_obs=float(self._params['chassis.w_obs']),
            w_unk=float(self._params['chassis.w_unk']),
            n_dirs=int(self._params['chassis.n_dirs']),
            dir_span_deg=float(self._params['chassis.dir_span_deg']))
        self._law = VelocityLaw(
            v_max=float(self._params['chassis.v_max']),
            d_slow=float(self._params['chassis.d_slow']),
            d_stop=float(self._params['chassis.d_stop']),
            a_max=float(self._params['chassis.a_max']),
            lp_cutoff_hz=float(self._params['chassis.lp_cutoff_hz']),
            lost_decay_s=float(self._params['chassis.lost_decay_s']))
        # 方向项解析求解器（赛题一权重 0.2，最大的可优化项）
        self._free = FreeDirSolver(
            lookahead_m=float(self._params['chassis.dir_lookahead_m']),
            block_unknown=bool(self._params['chassis.dir_block_unknown']))

        # --- 状态 -------------------------------------------------------------
        self._t_last = now()
        self._t_start = self._t_last
        self._has_grid: bool = False
        self._has_base: bool = False
        self._has_target: bool = False
        self._base_stamp = self._target_stamp = -math.inf
        self._base_received = self._target_received = -math.inf
        self._observation_stamp = -math.inf
        self._obstacle_stamp = self._obstacle_received = -math.inf
        self._obstacle_points = None
        self._map_stamps = {'analytic': -math.inf, 'quantized': -math.inf}
        self._map_received = {'analytic': -math.inf, 'quantized': -math.inf}
        self._clean_adapter = None
        self._ray_adapter = None
        self._map_frame = str(self._params['chassis.frame_id'])
        if not self._map_frame:
            raise ValueError('chassis.frame_id must be nonempty')
        if str(self._params['chassis.cmd_frame']) not in ('map', 'base_link'):
            raise ValueError('chassis.cmd_frame must be map or base_link')
        for key in ('chassis.obstacle_timeout_s','chassis.obstacle_clearance_m',
                    'chassis.obstacle_horizon_s','chassis.obstacle_braking_mps2'):
            if not math.isfinite(float(self._params[key])) or float(self._params[key]) <= 0:
                raise ValueError(key+' must be positive and finite')
        for key in ('chassis.base_timeout_s', 'chassis.target_timeout_s',
                    'chassis.last_observation_timeout_s'):
            if not math.isfinite(float(self._params[key])) or float(self._params[key]) <= 0:
                raise ValueError(key+' must be positive and finite')
        if (not math.isfinite(float(self._params['chassis.map_timeout_s']))
                or float(self._params['chassis.map_timeout_s']) < 0):
            raise ValueError('chassis.map_timeout_s must be nonnegative and finite')
        for key in ('chassis.v_max', 'chassis.d_slow'):
            if not math.isfinite(float(self._params[key])) or float(self._params[key]) <= 0:
                raise ValueError(key+' must be positive and finite')
        if (not math.isfinite(float(self._params['chassis.stop_radius_m']))
                or not 0 <= float(self._params['chassis.stop_radius_m']) < float(self._params['chassis.d_slow'])):
            raise ValueError('chassis.stop_radius_m must be nonnegative and below d_slow')
        if (not math.isfinite(float(self._params['chassis.v_floor']))
                or float(self._params['chassis.v_floor']) < 0):
            raise ValueError('chassis.v_floor must be nonnegative and finite')
        self._base_xy = np.zeros(2, dtype=float)
        self._base_yaw: float = 0.0
        self._has_clean_grid: bool = False
        self._goal_xy = np.zeros(2, dtype=float)     # 目标**预测**位置
        self._cur_xy = np.zeros(2, dtype=float)      # 目标**当前**位置（方向项基准）
        self._track_status: int = 0
        self._t_lost: Optional[float] = None
        self._dist: float = 0.0
        self._theta: float = 0.0
        self._blocked: bool = False

        # --- 话题 -------------------------------------------------------------
        qos = sensor_qos()
        self.create_subscription(TargetState, '/estimation/target_state',
                                 self._on_target, qos)
        self.create_subscription(Odometry, '/gimbal/base_pose',
                                 self._on_base_pose, qos)
        self.create_subscription(OccupancyGrid, '/map/ray_grid',
                                 self._on_ray_grid, latched_qos())
        # 未膨胀三态栅格：方向项解析求解用（不可跨越地形 = 占用格 100）
        self.create_subscription(OccupancyGrid, '/map/grid',
                                 self._on_clean_grid, latched_qos())
        self._obstacle_tf = Buffer()
        self._obstacle_listener = TransformListener(self._obstacle_tf,self)
        self.create_subscription(PointCloud2,'/red_standard_robot1/cloud_registered',
                                 self._on_obstacles,sensor_qos(2))
        self._pub = self.create_publisher(ChassisAdvice, '/chassis/advice',
                                          qos)

        period = 1.0 / max(float(self._params['chassis.rate_hz']), 1e-3)
        self.create_timer(period, self._on_timer)
        self._log.info(f'chassis_advisor_node 启动: '
                       f'rate={float(self._params["chassis.rate_hz"]):.1f}Hz '
                       f'cmd_frame={self._params["chassis.cmd_frame"]} '
                       f'v_max={float(self._params["chassis.v_max"]):.3f} '
                       f'lookahead={float(self._params["chassis.lookahead_m"]):.2f}m')

    # -- 回调 -----------------------------------------------------------------
    def _on_obstacles(self, msg):
        # 裁判起点建立之前尚无map底盘位姿，正常等待，不把它打印成TF故障。
        if not bool(self._params['chassis.obstacle_guard_enabled']) or not self._has_base:
            return
        try:
            if msg.header.frame_id != 'lio_world' or not self._params['chassis.obstacle_floor_valid']:
                raise ValueError('需要有效lio_world点云和地面近似')
            stamp = self._source_stamp(msg.header)
            age = self._clock_seconds()-stamp
            if not -float(self._params['chassis.future_tolerance_s']) <= age <= float(self._params['chassis.obstacle_timeout_s']):
                raise ValueError('障碍点云过期或时间无效')
            if stamp <= self._obstacle_stamp:
                return
            xyz = pointcloud2_xyz(msg)
            if len(xyz) < 40:
                raise ValueError('有效点云不足，不能确认通道')
            floor = float(self._params['chassis.obstacle_floor_z'])
            xyz = xyz[(xyz[:,2]>floor+.15)&(xyz[:,2]<floor+1.8)]
            if len(xyz) < 20:
                raise ValueError('障碍高度范围内点数不足')
            tf = self._obstacle_tf.lookup_transform(self._map_frame,'lio_world',Time.from_msg(msg.header.stamp))
            q,t = tf.transform.rotation,tf.transform.translation
            R = quat_to_mat([q.w,q.x,q.y,q.z])
            mapped = xyz @ R.T+np.array([t.x,t.y,t.z])
            self._obstacle_points = mapped[:,:2]
            self._obstacle_stamp,self._obstacle_received = stamp,now()
        except (TransformException,ValueError,TypeError,OverflowError) as exc:
            self._obstacle_points = None
            throttled(self._log,'warning',f'实时障碍保护未就绪，建议速度归零: {exc!r}','obstacles')

    def _guard_command(self, cmd, t_now):
        if not bool(self._params['chassis.obstacle_guard_enabled']):
            return cmd,0.,False
        timeout = float(self._params['chassis.obstacle_timeout_s'])
        age = self._clock_seconds()-self._obstacle_stamp
        if (self._obstacle_points is None or t_now-self._obstacle_received > timeout
                or not -float(self._params['chassis.future_tolerance_s']) <= age <= timeout):
            return np.zeros(2),0.,True
        points = self._obstacle_points-self._base_xy
        # 只保留足以覆盖当前速度的前瞻范围；不掩掉车体近处的障碍回波。
        radius = (float(self._params['chassis.obstacle_clearance_m'])+
                  float(self._params['chassis.v_max'])*float(self._params['chassis.obstacle_horizon_s'])+.3)
        near = points[np.linalg.norm(points,axis=1)<=radius]
        if not len(near):
            return cmd,0.,False  # 有足量新鲜远处障碍点，近处未发现障碍。
        return guard_velocity(cmd,near,
            clearance=float(self._params['chassis.obstacle_clearance_m']),
            horizon=float(self._params['chassis.obstacle_horizon_s']),
            braking=float(self._params['chassis.obstacle_braking_mps2']))

    def _clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _source_stamp(header) -> float:
        sec, ns = int(header.stamp.sec), int(header.stamp.nanosec)
        if sec < 0 or not 0 <= ns < 1000000000:
            raise ValueError('invalid source stamp')
        return sec + ns * 1e-9

    def _check_header(self, header) -> float:
        if str(header.frame_id) != self._map_frame:
            raise ValueError(f'expected frame {self._map_frame!r}, got {header.frame_id!r}')
        stamp = self._source_stamp(header)
        if stamp-self._clock_seconds() > float(self._params['chassis.future_tolerance_s']):
            raise ValueError('source stamp is in the future / another clock domain')
        return stamp

    def _parse_grid(self, msg) -> RayGridAdapter:
        self._check_header(msg.header)
        info = msg.info
        w, h, res = int(info.width), int(info.height), float(info.resolution)
        values = np.asarray(msg.data, dtype=float)
        if w <= 0 or h <= 0 or not math.isfinite(res) or res <= 0 or values.size != w*h:
            raise ValueError('invalid OccupancyGrid dimensions/resolution/data length')
        if not np.all(np.isin(values, [-1, 0, 100])):
            raise ValueError('advisor expects the team43_grid_mapper three-state map')
        origin = np.array([info.origin.position.x, info.origin.position.y], dtype=float)
        q = info.origin.orientation
        xyzw = np.array([q.x, q.y, q.z, q.w], dtype=float)
        norm = np.linalg.norm(xyzw)
        if not np.all(np.isfinite(origin)) or not np.all(np.isfinite(xyzw)) or norm < 1e-12:
            raise ValueError('invalid grid origin')
        x,y,z,wq = xyzw/norm
        if not np.allclose([2*(x*z+wq*y),2*(y*z-wq*x),1-2*(x*x+y*y)], [0,0,1], atol=1e-6):
            raise ValueError('tilted grid cannot be treated as a 2D map')
        yaw = math.atan2(2*(wq*z+x*y),1-2*(y*y+z*z))
        return RayGridAdapter(values.astype(np.int8).reshape(h,w).T, origin, res, yaw)

    def _on_ray_grid(self, msg: Any) -> None:
        try:
            adapter = self._parse_grid(msg)
            stamp = self._source_stamp(msg.header)
            if stamp < self._map_stamps['quantized']:
                return
            if stamp > self._map_stamps['quantized']:
                self._map_received['quantized'] = now()
            self._map_stamps['quantized'] = stamp
            self._ray.set_grid(adapter)
            self._ray_adapter = adapter
            self._has_grid = True
        except (ValueError, TypeError, OverflowError) as exc:
            # 外部坏地图不能继续用于方向建议；下一条有效地图可恢复。
            self._has_grid = False
            throttled(self._log, 'warning', f'ray_grid 拒绝: {exc!r}', 'ray_grid')

    def _on_clean_grid(self, msg: Any) -> None:
        try:
            adapter = self._parse_grid(msg)
            stamp = self._source_stamp(msg.header)
            if stamp < self._map_stamps['analytic']:
                return
            if stamp > self._map_stamps['analytic']:
                self._map_received['analytic'] = now()
            self._map_stamps['analytic'] = stamp
            self._free.set_grid(adapter.arr, adapter.origin_xy, adapter.resolution, adapter.origin_yaw)
            self._clean_adapter = adapter
            self._has_clean_grid = True
        except (ValueError, TypeError, OverflowError) as exc:
            # 外部坏地图不能继续用于方向建议；下一条有效地图可恢复。
            self._has_clean_grid = False
            throttled(self._log, 'warning', f'grid 拒绝: {exc!r}', 'clean_grid')

    def _on_base_pose(self, msg: Any) -> None:
        try:
            stamp = self._check_header(msg.header)
            if str(msg.child_frame_id) != str(self._params['chassis.base_frame_id']):
                raise ValueError('unexpected base child frame')
            position = msg.pose.pose.position
            xy = np.array([position.x, position.y], dtype=float)
            q = msg.pose.pose.orientation
            quat = np.array([q.x,q.y,q.z,q.w], dtype=float)
            if not np.all(np.isfinite(xy)) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-12:
                raise ValueError('invalid base position/orientation')
            if stamp <= self._base_stamp:
                return  # duplicated data must not renew freshness
            self._base_xy = xy
            x,y,z,w = quat/np.linalg.norm(quat)
            self._base_yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            self._base_stamp, self._base_received = stamp, now()
            self._has_base = True
        except (ValueError, TypeError, OverflowError) as exc:
            # 错坐标/无效姿态属于输入故障，清除有效标志而非使用旧位置。
            self._has_base = False
            throttled(self._log, 'warning', f'base_pose 拒绝: {exc!r}', 'base_pose')

    def _on_target(self, msg: Any) -> None:
        try:
            stamp = self._check_header(msg.header)
            if not bool(msg.valid) or int(msg.track_status) not in (1, 2):
                raise ValueError('target is lost/invalid')
            observation_age = float(msg.last_obs_age)
            if not math.isfinite(observation_age) or not 0 <= observation_age <= float(self._params['chassis.last_observation_timeout_s']):
                raise ValueError('target observation is stale')
            goal = np.array([msg.pred_x,msg.pred_y], dtype=float)
            current = np.array([msg.cur_x,msg.cur_y], dtype=float)
            if not np.all(np.isfinite(goal)) or not np.all(np.isfinite(current)):
                raise ValueError('nonfinite target position')
            if stamp <= self._target_stamp:
                return
            self._goal_xy, self._cur_xy = goal, current
            self._target_stamp, self._target_received = stamp, now()
            self._observation_stamp = stamp-observation_age
            self._has_target, self._track_status = True, int(msg.track_status)
            self._t_lost = None
        except (ValueError, TypeError, OverflowError) as exc:
            # 丢失或损坏的目标观测不能产生建议速度。
            self._mark_lost()
            throttled(self._log, 'warning', f'target 拒绝: {exc!r}', 'target')

    def _mark_lost(self) -> None:
        self._has_target = False
        self._track_status = 0
        if self._t_lost is None:
            self._t_lost = now()

    def _inputs_fresh(self, t_now: float) -> bool:
        ros_now = self._clock_seconds()
        if not (self._has_base and self._has_target):
            return False
        if ros_now-self._observation_stamp > float(self._params['chassis.last_observation_timeout_s']):
            return False
        for stamp, received, key in ((self._base_stamp,self._base_received,'chassis.base_timeout_s'),
                                     (self._target_stamp,self._target_received,'chassis.target_timeout_s')):
            timeout = float(self._params[key])
            if t_now-received > timeout or ros_now-stamp > timeout or ros_now-stamp < -float(self._params['chassis.future_tolerance_s']):
                return False
        return True

    def _on_timer(self) -> None:
        t_now = now()
        dt = max(0., min(.2, t_now-self._t_last))
        self._t_last = t_now
        if not self._inputs_fresh(t_now):
            self._law.reset()
            self._publish(np.zeros(2), 0., 0., False, valid=False)
            return
        mode = str(self._params['chassis.dir_mode']).lower()
        if mode == 'direct':
            # 比赛建议值，不接底盘执行器；不等待地图；默认比赛入口另用实时点云约束碰撞风险。
            direction, distance = VelocityLaw.unit_direction(self._base_xy, self._cur_xy)
            if distance < float(self._params['chassis.stop_radius_m']):
                self._law.reset()
                cmd = np.zeros(2)
            else:
                result = self._law.step(self._base_xy, self._cur_xy, dt, u_safe=direction,
                                        blocked=False, track_status=self._track_status,
                                        t_now=t_now, t_lost=None)
                cmd = self._apply_v_floor(direction*float(np.linalg.norm(result['cmd'])), direction)
            # 放在速度下限/平滑之后；后续不能再把受限速度放大到穿过障碍。
            cmd,theta,blocked = self._guard_command(cmd,t_now)
            if blocked:
                self._law.reset()
            if str(self._params['chassis.cmd_frame']) == 'base_link':
                cmd = rotz2(-self._base_yaw) @ cmd
            self._dist, self._theta, self._blocked = distance, theta, blocked
            self._publish(cmd, distance, theta, blocked, valid=not blocked)
            return
        adapter = self._clean_adapter if mode == 'analytic' else self._ray_adapter
        has_map = self._has_clean_grid if mode == 'analytic' else self._has_grid
        if mode not in ('analytic','quantized') or not bool(self._params['chassis.enable_avoidance']) or not has_map or adapter is None:
            self._law.reset()
            self._publish(np.zeros(2), 0., 0., True, valid=False)
            return
        map_timeout = float(self._params['chassis.map_timeout_s'])
        map_age = self._clock_seconds()-self._map_stamps[mode]
        if (map_age < -float(self._params['chassis.future_tolerance_s'])
                or (map_timeout > 0 and (map_age > map_timeout
                    or t_now-self._map_received[mode] > map_timeout))):
            self._law.reset()
            self._publish(np.zeros(2), 0., 0., True, valid=False)
            return
        cell = adapter.world_to_cell(self._base_xy)
        if not adapter.in_bounds(cell) or adapter.at(cell) < 0:
            self._law.reset()
            self._publish(np.zeros(2), 0., 0., True, valid=False)
            return
        # 停车判据与速度始终依据“当前距离”。预测位置只可改变建议方向，
        # 否则目标预测穿过底盘时会在仍相距很远的情况下误报停车。
        current_u, distance = VelocityLaw.unit_direction(self._base_xy, self._cur_xy)
        reference = self._goal_xy if bool(self._params['chassis.dir_use_pred']) else self._cur_xy
        u, reference_distance = VelocityLaw.unit_direction(self._base_xy, reference)
        if reference_distance < 1e-6:
            u = current_u
        if distance < float(self._params['chassis.stop_radius_m']):
            self._law.reset()
            self._publish(np.zeros(2), distance, 0., False, valid=True)
            return
        if mode == 'analytic':
            direction, theta, ok = self._free.solve_unit(self._base_xy, u)
            blocked = not ok
        else:
            theta, direction, blocked, hits = self._ray.evaluate(self._base_xy, u,
                                                   float(self._params['chassis.lookahead_m']))
            # A weighted cost is not a proof of feasibility: the cheapest ray
            # can still hit an obstacle. Quantized advice must choose a free ray.
            free_hits = [hit for hit in hits if not hit.hit_occ and
                         (not bool(self._params['chassis.dir_block_unknown']) or not math.isfinite(hit.d_unk))]
            if not free_hits:
                blocked = True
            else:
                chosen = min(free_hits,key=lambda hit:(abs(hit.theta),hit.cost))
                theta, direction, blocked = chosen.theta, rotz2(chosen.theta)@u, False
        if blocked:
            self._law.reset()
            cmd = np.zeros(2)
        else:
            result = self._law.step(self._base_xy, self._cur_xy, dt, u_safe=direction,
                                    blocked=False, track_status=self._track_status,
                                    t_now=t_now, t_lost=None)
            # This is a scored direction recommendation, not an actuator command.
            # Vector smoothing could interpolate two free directions through a
            # wall, so retain only its speed and use the newly verified direction.
            cmd = direction * float(np.linalg.norm(result['cmd']))
            cmd = self._apply_v_floor(cmd, direction, blocked=False)
        if str(self._params['chassis.cmd_frame']) == 'base_link':
            cmd = rotz2(-self._base_yaw) @ cmd
        self._dist, self._theta, self._blocked = distance, float(theta), bool(blocked)
        self._publish(cmd, distance, theta, blocked, valid=True)

    def _apply_v_floor(self, cmd: np.ndarray, safe_direction=None, blocked=False) -> np.ndarray:
        """Scale only a verified feasible direction; never invent a zero direction."""
        v = np.asarray(cmd,dtype=float).reshape(2)
        if blocked or safe_direction is None or not np.all(np.isfinite(v)):
            return np.zeros(2)
        u = np.asarray(safe_direction,dtype=float).reshape(2)
        norm = float(np.linalg.norm(u))
        if not np.all(np.isfinite(u)) or norm < 1e-9:
            return np.zeros(2)
        distance = float(np.linalg.norm(self._cur_xy-self._base_xy))
        if distance < float(self._params['chassis.stop_radius_m']):
            return np.zeros(2)
        vmax = float(self._params['chassis.v_max'])
        # .30 m/s 的远处速度下限不能把近处减速全部抹掉。接近停止半径时
        # 线性降低上限；仍在停止带之外时保留 .04 m/s，避免误触官方停车线。
        stop = float(self._params['chassis.stop_radius_m'])
        slow = float(self._params['chassis.d_slow'])
        approach_cap = min(vmax, max(V_STOP_OFFICIAL*2,
                                    vmax*(distance-stop)/max(slow-stop, 1e-9)))
        floor = min(approach_cap, max(float(self._params['chassis.v_floor']),
                                     V_STOP_OFFICIAL*2))
        speed = min(approach_cap, max(floor, float(np.linalg.norm(v))))
        return u/norm*speed

    def _publish(self, cmd: np.ndarray, dist: float, theta: float,
                 blocked: bool, valid: bool) -> None:
        """发布 `/chassis/advice`（header.stamp 走 ROS 系统时钟）。"""
        msg = ChassisAdvice()
        msg.header.frame_id = (str(self._params['chassis.base_frame_id'])
                               if str(self._params['chassis.cmd_frame']) == 'base_link'
                               else self._map_frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cmd_vx = float(cmd[0])
        msg.cmd_vy = float(cmd[1])
        msg.dist_to_target = float(dist)
        msg.chosen_theta = float(theta)
        msg.blocked = bool(blocked)
        msg.valid = bool(valid)
        self._pub.publish(msg)
        if blocked:
            throttled(self._log, 'warning',
                      '没有得到可验证的可行方向，保持零建议；不会强造朝目标速度',
                      'team43_chassis_advisor.blocked')

    def shutdown(self) -> None:
        """退出日志摘要。"""
        self._log.info(f'chassis_advisor_node 退出，耗时 {now() - self._t_start:.2f}s，'
                       f'末次 d={self._dist:.3f} θ={math.degrees(self._theta):.1f}°')


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = ChassisAdvisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):  # 正常停机仍执行收尾。
        pass
    finally:
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
