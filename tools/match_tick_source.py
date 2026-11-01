"""容器验收专用独立评分源：不与大图像共用DDS进程，只发/test消息。"""
import argparse
import os
from pathlib import Path
import time
import rclpy
from std_msgs.msg import Header
from team43_common.qos_utils import reliable_qos,latched_qos


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count',type=int,required=True)
    parser.add_argument('--transcript',type=Path,required=True)
    args=parser.parse_args()
    if (not Path('/.dockerenv').exists() or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
            or not os.environ.get('ROS_DOMAIN_ID') or not 1<=args.count<=9000):
        parser.error('仅允许设置独立ROS域的容器测试，count须1..9000')
    rclpy.init();node=rclpy.create_node('isolated_match_tick_source')
    tick=node.create_publisher(Header,'/test/match/tick',reliable_qos())
    origin=node.create_publisher(Header,'/test/match/start',latched_qos())
    start=time.monotonic()
    try:
        while not tick.get_subscription_count() or not origin.get_subscription_count():
            if time.monotonic()-start>15:raise RuntimeError('测试评分订阅未就绪')
            rclpy.spin_once(node,timeout_sec=.01)
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        with args.transcript.open('x') as stream:
            next_tick=time.monotonic()
            for index in range(args.count):
                while time.monotonic()<next_tick:
                    rclpy.spin_once(node,timeout_sec=max(0.,min(.01,next_tick-time.monotonic())))
                msg=Header(stamp=node.get_clock().now().to_msg(),frame_id=str(index))
                stamp=msg.stamp.sec+msg.stamp.nanosec*1e-9
                stream.write('%.6f\n'%stamp);stream.flush()
                if index==0:origin.publish(Header(stamp=msg.stamp,frame_id='record'))
                tick.publish(msg);next_tick+=.1
                if index%100==0:print(f'独立tick={index}/{args.count}',flush=True)
        end=time.monotonic()+1.2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
