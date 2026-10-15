"""隔离域的固定状态提交测试源；只发布/test/tick、/test/replay_start及合成状态。"""
import argparse
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=1800)
    args = parser.parse_args()
    if args.count < 1:
        parser.error('--count必须为正数')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('请设置ROS_LOCALHOST_ONLY=1和专用ROS_DOMAIN_ID')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Header
    from nav_msgs.msg import Odometry, OccupancyGrid
    from gimbal_interfaces.msg import TargetState, ChassisAdvice

    rclpy.init()
    node = Node('submission_test_source')
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tick = node.create_publisher(Header, '/test/tick', reliable)
    replay = node.create_publisher(Header, '/test/replay_start', latched)
    target = node.create_publisher(TargetState, '/estimation/target_state', reliable)
    base = node.create_publisher(Odometry, '/gimbal/base_pose', reliable)
    advice = node.create_publisher(ChassisAdvice, '/chassis/advice', reliable)
    grid = node.create_publisher(OccupancyGrid, '/map/grid', latched)
    start = time.monotonic()
    ready_at = None
    count = 0
    done_at = None

    def step():
        nonlocal ready_at, count, done_at
        now = time.monotonic()
        if done_at is not None:
            if now - done_at >= 1.2:
                node.get_logger().info(f'完成{count}拍；请检查CSV与地图')
                rclpy.shutdown()
            return
        publishers = (tick, replay, target, base, advice, grid)
        if not all(p.get_subscription_count() for p in publishers):
            if now - start > 30:
                raise RuntimeError('等待订阅超时；检查输出节点、域、话题映射和QoS')
            return
        stamp = node.get_clock().now().to_msg()
        t = TargetState(); t.header.stamp = stamp; t.header.frame_id = 'map'
        t.cur_x = 1.; t.cur_y = 2.; t.pred_x = 1.; t.pred_y = 2.
        t.valid = True; t.track_status = 1
        b = Odometry(); b.header.stamp = stamp; b.header.frame_id = 'map'
        b.child_frame_id = 'base_link'; b.pose.pose.orientation.w = 1.
        b.pose.pose.position.x = .5; b.pose.pose.position.y = -.25
        a = ChassisAdvice(); a.header.stamp = stamp; a.header.frame_id = 'map'
        a.cmd_vx = .3; a.cmd_vy = .4; a.valid = True
        g = OccupancyGrid(); g.header.stamp = stamp; g.header.frame_id = 'map'
        g.info.width = 4; g.info.height = 3; g.info.resolution = .5
        g.info.origin.position.x = -1.; g.info.origin.position.y = -.5
        g.info.origin.orientation.w = 1.
        g.data = [100,0,0,-1,0,100,0,0,0,0,100,0]
        target.publish(t); base.publish(b); advice.publish(a); grid.publish(g)
        # 固定状态先预热2秒，避免跨话题DDS顺序影响第0拍。
        if ready_at is None:
            ready_at = now
        if now - ready_at < 2:
            return
        h = Header(); h.stamp = stamp; h.frame_id = str(count)
        if count == 0:
            origin = Header(); origin.stamp = stamp; origin.frame_id = 'replay'
            replay.publish(origin)
        tick.publish(h)
        count += 1
        if count >= args.count:
            done_at = now
    node.create_timer(.1, step)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
