"""等待本次实机标定和源图，写出不可变配置快照后交接给launch。"""
import json
import math
import os
import tempfile
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, UInt64
import yaml

from team43_common.qos_utils import latched_qos
from team43_common.config_loader import ConfigError
from .deployment_config import load_vehicle_config


def verified_match_params(path, epoch):
    _, params = load_vehicle_config(path)
    if epoch is None or params['kinematics.calibrated_epoch'] != epoch:
        raise ValueError('等待本次世界对齐：calibrated_epoch须与当前桥一致，不能仅改epoch数字')
    if params.get('kinematics.joint_limits_confirmed') is not True:
        raise ValueError('等待实测关节工作区间确认：kinematics.joint_limits_confirmed')
    for key in ('kinematics.yaw_limits_rad', 'kinematics.pitch_limits_rad'):
        bounds = params[key]
        if len(bounds) != 2 or not all(math.isfinite(float(v)) for v in bounds) or bounds[1]-bounds[0] <= .07:
            raise ValueError(key+'须为有依据的工作区间，并容纳两侧各0.035rad余量')
    if params.get('grid.source_frame') != 'lio_world' or params.get('grid.map_topic') != '/team43/slam_map':
        raise ValueError('比赛入口的源图必须来自本次/team43/slam_map，frame为lio_world')
    if params.get('localization.odom_frame_id') != 'lio_world' or params.get('localization.odom_child_frame_id') != 'body':
        raise ValueError('比赛入口使用Point-LIO的lio_world→body')
    if params.get('perception.marker_size_m') != .075 or params.get('perception.aruco_dict') != 'DICT_4X4_50':
        raise ValueError('现场已确定DICT_4X4_50及黑框边长0.075m')
    return params


class MatchSessionGate(Node):
    def __init__(self, **kwargs):
        super().__init__('match_session_gate', **kwargs)
        for name, value in [('vehicle_config', ''), ('snapshot', '')]:
            self.declare_parameter(name, value)
        self.epoch = None
        self.map_received = None
        self.ready = False
        self.last_reason = None
        self.pub = self.create_publisher(String, '/team43/match_session', latched_qos(1))
        self.create_subscription(UInt64, '/task1/gimbal_reset_epoch', self.on_epoch, latched_qos(1))
        self.create_subscription(OccupancyGrid, '/team43/slam_map', self.on_map, latched_qos(1))
        self.create_timer(.5, self.poll)

    def on_epoch(self, msg):
        self.epoch = int(msg.data)

    def on_map(self, msg):
        if (msg.header.frame_id == 'lio_world' and msg.info.width > 0 and msg.info.height > 0
                and len(msg.data) == msg.info.width*msg.info.height and any(v >= 0 for v in msg.data)):
            self.map_received = time.monotonic()

    def poll(self):
        if self.ready: return
        try:
            params = verified_match_params(self.get_parameter('vehicle_config').value, self.epoch)
            if self.map_received is None or time.monotonic()-self.map_received > 3.:
                raise ValueError('等待本次新鲜源地图；本场自动采用高度近似，查看/team43/cloud_mapping/status的扫描接收原因')
            path = Path(self.get_parameter('snapshot').value).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            # 目标只出现完整快照；写盘失败不留下一个使下次重试永久失败的半文件。
            fd, temporary = tempfile.mkstemp(prefix='.vehicle_', dir=path.parent)
            try:
                with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                    yaml.safe_dump({'/**': {'ros__parameters': params}}, stream, allow_unicode=True)
                os.link(temporary, path)  # 原子、排他：绝不覆盖另一次会话的配置
            finally:
                os.unlink(temporary)
            self.ready = True
            reason = '标定和源图就绪；按比赛入口参数启动处理、控制和评分'
        except (ConfigError, OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            reason = str(exc)
        msg = String()
        msg.data = json.dumps({'state': 'ready' if self.ready else 'waiting',
                               'epoch': self.epoch, 'reason': reason}, ensure_ascii=False)
        self.pub.publish(msg)
        if reason != self.last_reason:
            self.get_logger().info(msg.data)
            self.last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = MatchSessionGate()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=.2)
        return 0 if node.ready else 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
