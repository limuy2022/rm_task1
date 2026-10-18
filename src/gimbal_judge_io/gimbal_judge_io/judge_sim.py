"""judge_sim：主办方 `/judge` 话题的**哑模拟器**（赛前联调用，正式赛绝不能用）。

它只做一件事：按主办方的 QoS 与频率发拍，让 `judge_io_node` 在没有判分程序
的情况下也能写出 CSV / 地图，从而先把"输出契约"这条最致命的链路走通。

发布：

| 话题 | 类型 | QoS | 频率 |
|---|---|---|---|
| `/judge/replay_start` | `std_msgs/Header` | TRANSIENT_LOCAL + depth 1 | 启动时一次 |
| `/judge/tick` | `std_msgs/Header` | RELIABLE + KEEP_LAST + depth 10 | `sim.rate_hz`（默认 10 Hz） |

`/judge/tick` 的 `frame_id` = 拍号（十进制字符串），`stamp` = 当前时刻——
与主办方约定一致。默认的 8 列提交形态**不写** `stamp`；只有把
提交器固定将它写为 CSV 第 9 列 `t_tick`。

可选 `sim.extra_signals=true` 时，还会发**假的** `/gimbal/base_pose`、
`/estimation/target_state`、`/chassis/advice`，让 CSV 里不是一片 0，
可以在没有相机/云台/底盘的情况下验证"整条链路 + 列值合理性"。

用法：

```bash
# 本包已装（推荐）
ros2 run gimbal_judge_io judge_sim --ros-args -p sim.duration_s:=20.0

# 或直接跑源码
python3 src/gimbal_judge_io/gimbal_judge_io/judge_sim.py
```

参数（`-p 名字:=值`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `sim.rate_hz` | 10.0 | 拍频（主办方 10 Hz） |
| `sim.duration_s` | 30.0 | 发多久（s） |
| `sim.n_ticks` | 0 | >0 时按**拍数**跑，忽略 duration |
| `sim.start_index` | 0 | 起始拍号 |
| `sim.publish_replay_start` | true | 是否发起点信号 |
| `sim.extra_signals` | false | 是否同时发假传感器信号 |
| `sim.start_delay_s` | 0.0 | 开拍前等待（留给订阅方加入） |
"""

from __future__ import annotations

import math
import sys
import time
from typing import Any
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gimbal_interfaces.msg import ChassisAdvice, TargetState
from gimbal_common.node_utils import declare_params
from gimbal_common.qos_utils import latched_qos, reliable_qos, sensor_qos


NODE_NAME = 'judge_sim'

PARAM_DEFAULTS = {
    'sim.rate_hz': 10.0,
    'sim.duration_s': 30.0,
    'sim.n_ticks': 0,
    'sim.start_index': 0,
    'sim.publish_replay_start': True,
    'sim.extra_signals': False,
    'sim.start_delay_s': 0.0,
}

#: 假目标起始位置（map 系，给定点系）与假底盘速度
FAKE_TARGET0 = (1.5, 0.5)
FAKE_BASE_SPEED = 0.20


class JudgeSimNode(Node):  # type: ignore[misc]
    """哑判分器：发 tick（+可选假信号）。"""

    def __init__(self, name: str = NODE_NAME) -> None:
        super().__init__(name)
        self._log = self.get_logger()
        self._params = declare_params(self, PARAM_DEFAULTS)

        self._rate = max(float(self._params['sim.rate_hz']), 0.1)
        self._duration = max(float(self._params['sim.duration_s']), 0.0)
        self._n_max = int(self._params['sim.n_ticks'])
        self._index = int(self._params['sim.start_index'])
        self._extra = bool(self._params['sim.extra_signals'])
        self._delay = max(float(self._params['sim.start_delay_s']), 0.0)

        self._pub_tick = self.create_publisher(Header, '/judge/tick', reliable_qos())
        self._pub_start = self.create_publisher(Header, '/judge/replay_start',
                                                latched_qos(1))

        self._pub_base = None
        self._pub_target = None
        self._pub_advice = None
        if self._extra:
            self._pub_base = self.create_publisher(Odometry, '/gimbal/base_pose',
                                                   sensor_qos(10))
            self._pub_target = self.create_publisher(
                TargetState, '/estimation/target_state', sensor_qos(10))
            self._pub_advice = self.create_publisher(
                ChassisAdvice, '/chassis/advice', sensor_qos(10))
            self.create_timer(1.0 / 50.0, self._on_signal_timer)

        self._t0: Optional[float] = None
        self._t_start: float = time.monotonic()
        self._done: bool = False
        self._n_sent: int = 0

        self.create_timer(1.0 / self._rate, self._on_tick_timer)
        self._log.info('judge_sim 启动：%.1f Hz，duration=%.1fs，n_ticks=%d，'
                       'extra_signals=%s（⚠ 正式赛不要开这个节点）'
                       % (self._rate, self._duration, self._n_max, self._extra))

    # -- 属性 -----------------------------------------------------------------
    @property
    def done(self) -> bool:
        """是否已发完。"""
        return self._done

    # -- 定时器 ---------------------------------------------------------------
    def _on_tick_timer(self) -> None:
        """按拍发布 `/judge/tick`（frame_id=拍号，stamp=now）。"""
        if self._done:
            return
        now_m = time.monotonic()
        if self._t0 is None:
            if (now_m - self._t_start) < self._delay:
                return
            self._t0 = now_m
            if bool(self._params['sim.publish_replay_start']):
                self._publish_replay_start()

        if self._n_max > 0 and self._n_sent >= self._n_max:
            self._finish()
            return
        if self._n_max <= 0 and (now_m - self._t0) > self._duration:
            self._finish()
            return

        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self._index)
        self._pub_tick.publish(msg)
        self._n_sent += 1
        self._index += 1

    def _finish(self) -> None:
        """收尾：不再发包，打印统计。"""
        self._done = True
        self._log.info('judge_sim 发完 %d 拍（%d → %d），退出。'
                       % (self._n_sent, int(self._params['sim.start_index']),
                          self._index - 1))

    def _publish_replay_start(self) -> None:
        """发一次起点信号（TRANSIENT_LOCAL，晚加入的订阅者也能收到）。"""
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'sim_replay_start'
        self._pub_start.publish(msg)
        self._log.info('已发布 /judge/replay_start')

    # -- 假信号 ---------------------------------------------------------------
    def _on_signal_timer(self) -> None:
        """发假 base_pose / target_state / advice（仅在 extra_signals 时）。"""
        if self._t0 is None or self._done:
            return
        t = time.monotonic() - self._t0
        stamp = self.get_clock().now().to_msg()
        self._publish_base(t, stamp)
        self._publish_target(t, stamp)
        self._publish_advice(t, stamp)

    @staticmethod
    def _fake_target(t: float):
        """假目标：在 (1.5, 0.5) 附近做小幅圆周运动 → (cur, vel)。"""
        w = 0.5
        x = FAKE_TARGET0[0] + 0.20 * math.sin(w * t)
        y = FAKE_TARGET0[1] + 0.20 * math.cos(w * t)
        vx = 0.20 * w * math.cos(w * t)
        vy = -0.20 * w * math.sin(w * t)
        return (x, y), (vx, vy)

    @staticmethod
    def _fake_base(t: float):
        """假底盘：沿 +x 匀速 0.2 m/s 前进。"""
        return (FAKE_BASE_SPEED * t, 0.0)

    def _publish_base(self, t: float, stamp: Any) -> None:
        """发布假底盘位姿。"""
        bx, by = self._fake_base(t)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = bx
        msg.pose.pose.position.y = by
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = FAKE_BASE_SPEED
        self._pub_base.publish(msg)

    def _publish_target(self, t: float, stamp: Any) -> None:
        """发布假目标状态（cur = 真值，pred = cur + vel*0.1）。"""
        (x, y), (vx, vy) = self._fake_target(t)
        msg = TargetState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.cur_x, msg.cur_y = float(x), float(y)
        msg.pred_x, msg.pred_y = float(x + 0.1 * vx), float(y + 0.1 * vy)
        msg.vel_x, msg.vel_y = float(vx), float(vy)
        msg.track_status = 1
        msg.valid = True
        self._pub_target.publish(msg)

    def _publish_advice(self, t: float, stamp: Any) -> None:
        """发布假底盘建议：朝目标，速度大小 ≥0.05（不踩停车陷阱）。"""
        (tx, ty), _v = self._fake_target(t)
        bx, by = self._fake_base(t)
        dx, dy = tx - bx, ty - by
        d = math.hypot(dx, dy)
        msg = ChassisAdvice()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        if d > 1e-6:
            sp = 0.0 if d < 0.45 else 0.30
            msg.cmd_vx = float(dx / d * sp)
            msg.cmd_vy = float(dy / d * sp)
        msg.dist_to_target = float(d)
        msg.chosen_theta = 0.0
        msg.blocked = False
        msg.valid = True
        self._pub_advice.publish(msg)


def main(args: Optional[list] = None) -> int:
    """节点入口。"""
    rclpy.init(args=args)
    node = JudgeSimNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:  # Ctrl+C 是正常停机请求；finally 仍执行停止/关闭资源。
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
