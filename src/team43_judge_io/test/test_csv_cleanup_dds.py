"""评分端先清理CSV再发拍，以及运行中删文件的真实DDS回归。"""
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Header

from team43_common.qos_utils import reliable_qos
from team43_judge_io.judge_io_node import JudgeIoNode


def test_grader_deletes_csv_before_first_and_later_tick(tmp_path):
    path = tmp_path/'43.csv'
    path.write_text('previous incomplete round')
    rclpy.init(args=['--ros-args', '-p', 'judge.output_dir:='+str(tmp_path),
                    '-r', '/judge/tick:=/test/csv_cleanup/tick',
                    '-r', '/judge/replay_start:=/test/csv_cleanup/start',
                    '-r', '/gimbal/origin_freeze:=/test/csv_cleanup/freeze'])
    executor = SingleThreadedExecutor()
    judge = source = None
    try:
        judge = JudgeIoNode('csv_cleanup_submission')
        source = rclpy.create_node('csv_cleanup_source')
        publisher = source.create_publisher(Header, '/test/csv_cleanup/tick', reliable_qos())
        executor.add_node(judge); executor.add_node(source)
        assert path.read_text() == 'previous incomplete round'
        assert not judge._writer.is_open
        deadline = time.monotonic()+5.
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
        assert publisher.get_subscription_count() > 0
        for index in range(2):
            path.unlink()
            msg = Header(stamp=source.get_clock().now().to_msg(), frame_id=str(index))
            publisher.publish(msg)
            deadline = time.monotonic()+5.
            while judge._n_tick < index+1 and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.02)
            assert judge._n_tick == index+1
            rows = path.read_text().splitlines()
            assert len(rows) == 1
            assert len(rows[0].split(',')) == 9
            assert rows[0].split(',')[-1] == '%.6f' % (msg.stamp.sec+msg.stamp.nanosec*1e-9)
        assert judge._writer.stats()['external_reopens'] == 1
    finally:
        executor.shutdown()
        if judge is not None:
            judge._writer.close(); judge.destroy_node()
        if source is not None: source.destroy_node()
        rclpy.shutdown()
