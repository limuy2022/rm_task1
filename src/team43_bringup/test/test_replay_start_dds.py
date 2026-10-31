"""真实DDS旧锁存起点回归：不接硬件，只使用/test话题和临时输出。"""
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from team43_common.qos_utils import latched_qos
from team43_localization.map_frame_broadcaster_node import MapFrameBroadcasterNode
from team43_judge_io.judge_io_node import JudgeIoNode


@pytest.mark.parametrize('replay_first', [True, False])
def test_old_latched_start_then_explicit_start_in_either_topic_order(tmp_path, monkeypatch, replay_first):
    seen = {'map': [], 'judge': []}
    map_callback = MapFrameBroadcasterNode._on_origin_freeze
    judge_callback = JudgeIoNode._freeze_origin
    def map_spy(self, msg):
        seen['map'].append(msg.stamp.sec)
        return map_callback(self, msg)
    def judge_spy(self, msg):
        seen['judge'].append(msg.stamp.sec)
        return judge_callback(self, msg)
    monkeypatch.setattr(MapFrameBroadcasterNode, '_on_origin_freeze', map_spy)
    monkeypatch.setattr(JudgeIoNode, '_freeze_origin', judge_spy)
    args = ['--ros-args', '-p', 'judge.output_dir:='+str(tmp_path)]
    for src, dst in {'/judge/replay_start': '/test/replay_recovery/start',
                     '/judge/tick': '/test/replay_recovery/tick',
                     '/gimbal/origin_freeze': '/test/replay_recovery/freeze',
                     '/gimbal/base_pose_raw': '/test/replay_recovery/raw',
                     '/gimbal/base_pose': '/test/replay_recovery/base'}.items():
        args += ['-r', src+':='+dst]
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = []
    judge = None
    try:
        source = rclpy.create_node('replay_recovery_source'); nodes.append(source)
        publisher = source.create_publisher(Header, '/test/replay_recovery/start', latched_qos(1))
        old = Header(stamp=source.get_clock().now().to_msg()); old.stamp.sec -= 10
        publisher.publish(old)  # 在两个实际节点启动前留下锁存消息。
        map_node = rclpy.create_node('replay_recovery_map'); nodes.append(map_node)
        mapper = MapFrameBroadcasterNode(map_node)
        judge = JudgeIoNode('replay_recovery_submit'); nodes.append(judge)
        for node in nodes: executor.add_node(node)
        deadline = time.monotonic()+5.
        while not all(old.stamp.sec in seen[key] for key in seen) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        assert all(old.stamp.sec in seen[key] for key in seen), seen
        assert mapper._origin_stamp is None and not mapper.frozen
        assert not judge._origin_frozen
        fresh = Header(stamp=source.get_clock().now().to_msg())
        pose = Odometry()
        pose.header.stamp = fresh.stamp
        pose.header.frame_id = 'odom'; pose.child_frame_id = 'base_link'
        pose.pose.pose.orientation.w = 1.
        mapper._on_base_pose(pose)
        from team43_common.qos_utils import reliable_qos
        tick_pub = source.create_publisher(Header, '/test/replay_recovery/tick', reliable_qos())
        deadline = time.monotonic()+5.
        while tick_pub.get_subscription_count()<1 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        assert tick_pub.get_subscription_count() >= 1
        fresh.frame_id = 'record'
        if replay_first:
            publisher.publish(fresh)
            deadline = time.monotonic()+5.
            while not (mapper.frozen and judge._origin_frozen) and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.05)
        tick_pub.publish(Header(stamp=fresh.stamp, frame_id='0'))
        deadline = time.monotonic()+5.
        while judge._n_tick<1 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        assert judge._n_tick == 1
        if not replay_first:
            assert not mapper.frozen and mapper._origin_stamp is None
            assert not judge._origin_frozen
            publisher.publish(fresh)
        deadline = time.monotonic()+5.
        while not (mapper.frozen and judge._origin_frozen) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        assert mapper.frozen and judge._origin_frozen
        assert mapper._origin_stamp == judge._replay_stamp
        assert mapper._origin_stamp == fresh.stamp.sec+fresh.stamp.nanosec*1e-9
    finally:
        executor.shutdown()
        if judge is not None: judge._writer.close()
        for node in reversed(nodes): node.destroy_node()
        rclpy.shutdown()


def test_sim_clock_activation_refreshes_stale_origin_window_before_round():
    """公开use_sim_time切换后，系统时钟的启动门槛不能拒绝本轮仿真起点。"""
    from rclpy.parameter import Parameter
    rclpy.init(args=['--ros-args','-r','/tf_static:=/test/replay_clock/tf_static',
                    '-r','/gimbal/base_pose:=/test/replay_clock/base'])
    node=rclpy.create_node('replay_clock_epoch_test')
    try:
        mapper=MapFrameBroadcasterNode(node)
        assert mapper._replay_not_before>1.
        result=node.set_parameters([Parameter('use_sim_time',value=True)])
        assert all(r.successful for r in result)
        assert mapper._replay_not_before==0.
        pose=Odometry();pose.header.frame_id='odom';pose.child_frame_id='base_link'
        pose.pose.pose.orientation.w=1.
        mapper._on_base_pose(pose)
        mapper._on_origin_freeze(Header())
        assert mapper.frozen
        # 已接受本轮起点后不能因切钟再选另一原点；停止发布，等待重启新轮。
        node.set_parameters([Parameter('use_sim_time',value=False)])
        assert mapper._clock_fault and not mapper.frozen
    finally:
        node.destroy_node();rclpy.shutdown()
