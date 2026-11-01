#!/usr/bin/env python3
"""容器内真实DDS录制、分片、SQLite计数、逐消息读回与实际bag回放验证。"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Imu, Image, CameraInfo
from std_msgs.msg import UInt64
from team43_common.qos_utils import latched_qos
import rosbag2_py

ROOT=Path(__file__).resolve().parents[1]
LIDAR='/red_standard_robot1/livox/lidar';IMU='/red_standard_robot1/livox/imu'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if not Path('/.dockerenv').exists() or ROOT!=Path('/workspace'):parser.error('仅限/workspace容器')
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='85')
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('test_raw_record_source')
    lidar=node.create_publisher(PointCloud2,LIDAR,qos_profile_sensor_data)
    imu=node.create_publisher(Imu,IMU,10)
    report={'passed':False,'hardware_accessed':False,'checks':{}};process=None
    expected={LIDAR:{},IMU:{}}
    def wait(condition,timeout=12):
        end=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>end:raise RuntimeError('录制验收等待超时')
            rclpy.spin_once(node,timeout_sec=.005)
    def source_frames(count=100):
        for i in range(count):
            now=node.get_clock().now().to_msg()
            cloud=PointCloud2();cloud.header.stamp=now;cloud.header.frame_id=str(i)
            cloud.height=1;cloud.width=2;cloud.point_step=12;cloud.row_step=24
            cloud.fields=[PointField(name=n,offset=o,datatype=7,count=1) for n,o in [('x',0),('y',4),('z',8)]]
            cloud.data=struct.pack('<6f',i*.01,-.2,.3,.4,.5,.6)
            expected[LIDAR][str(i)]=cloud;lidar.publish(cloud)
            for j in range(4):
                msg=Imu();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id=str(i*4+j)
                msg.orientation.w=1.;msg.linear_acceleration.z=1.;msg.angular_velocity.x=i*.001
                expected[IMU][msg.header.frame_id]=msg;imu.publish(msg)
                end=time.monotonic()+.005
                wait(lambda:time.monotonic()>=end,1)
    def record_case(name,seconds=0,count=100,stop_signal=signal.SIGINT):
        nonlocal process
        target=output/name
        with (output/(name+'.log')).open('w') as stream:
            process=subprocess.Popen(['python3','-B',str(ROOT/'tools/record.py'),'--output-dir',str(target),
                                      '--seconds',str(seconds),'--split-seconds','1'],
                                     stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            wait(lambda:lidar.get_subscription_count()>=1 and imu.get_subscription_count()>=1)
            source_frames(count)
            time.sleep(.3)
            if seconds==0:process.send_signal(stop_signal)
            wait(lambda:process.poll() is not None,15)
            assert process.returncode==0,(output/(name+'.log')).read_text()
        meta=json.loads((target/'manifest.json').read_text())
        assert meta['success'] and meta['bag']['counts']=={LIDAR:count,IMU:count*4},meta
        assert (target/'config/lio.yaml').is_file()
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(target/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        seen={LIDAR:set(),IMU:set()}
        while reader.has_next():
            topic,data,_=reader.read_next();msg=deserialize_message(data,PointCloud2 if topic==LIDAR else Imu)
            assert msg==expected[topic][msg.header.frame_id],'读回消息与发布原始值不一致'
            seen[topic].add(msg.header.frame_id)
        assert {t:len(ids) for t,ids in seen.items()}=={LIDAR:count,IMU:count*4}
        return target,meta
    try:
        target,meta=record_case('manual_stop')
        assert len(meta['bag']['files'])>=2,'没有按时间分片'
        report['checks']['manual_sigint_split_raw_messages_exact_and_config_snapshot']=True
        # 实际ros2 bag play，重映射到只读测试话题，验证不是仅能读metadata。
        seen={LIDAR:[],IMU:[]}
        for topic,typ in [(LIDAR,PointCloud2),(IMU,Imu)]:
            node.create_subscription(typ,'/test/replay'+topic,lambda msg,t=topic:seen[t].append(msg),
                QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT))
        with (output/'play.log').open('w') as log:
            process=subprocess.Popen(['ros2','bag','play',str(target/'bag'),'--rate','1','--delay','1',
                '--remap',LIDAR+':=/test/replay'+LIDAR,IMU+':=/test/replay'+IMU],stdout=log,stderr=subprocess.STDOUT)
            wait(lambda:process.poll() is not None,15)
            end=time.monotonic()+.3;wait(lambda:time.monotonic()>=end)
        assert process.returncode==0
        for topic,count in [(LIDAR,100),(IMU,400)]:
            assert len(seen[topic])==count,(topic,len(seen[topic]))
            assert all(msg==expected[topic][msg.header.frame_id] for msg in seen[topic])
        report['checks']['actual_bag_playback_preserves_raw_headers_and_values']=True
        wait(lambda:lidar.get_subscription_count()==0 and imu.get_subscription_count()==0)
        record_case('timed_stop',seconds=5,count=30)
        report['checks']['timed_stop_saves_counts_and_metadata']=True
        with (output/'empty.log').open('w') as log:
            process=subprocess.Popen(['python3','-B',str(ROOT/'tools/record.py'),'--output-dir',str(output/'empty'),
                                      '--seconds','1'],stdout=log,stderr=subprocess.STDOUT)
            wait(lambda:process.poll() is not None,10)
        assert process.returncode==1 and not json.loads((output/'empty/manifest.json').read_text())['success']
        report['checks']['empty_capture_rejected_without_discarding_evidence']=True
        # 重复目录必须保留既有manifest和bag，不允许覆盖。
        original=(target/'manifest.json').read_bytes()
        process=subprocess.Popen(['python3','-B',str(ROOT/'tools/record.py'),'--output-dir',str(target)],
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        wait(lambda:process.poll() is not None)
        assert process.returncode!=0 and (target/'manifest.json').read_bytes()==original
        report['checks']['existing_record_not_overwritten']=True
        wait(lambda:lidar.get_subscription_count()==0 and imu.get_subscription_count()==0)
        image_pub=node.create_publisher(Image,'/image_raw',10)
        info_pub=node.create_publisher(CameraInfo,'/camera_info',10)
        gimbal_pub=node.create_publisher(Imu,'/red_standard_robot1/gimbal',qos_profile_sensor_data)
        epoch_pub=node.create_publisher(UInt64,'/task1/gimbal_reset_epoch',latched_qos(1))
        epoch_pub.publish(UInt64(data=43))
        target=output/'with_camera'
        with (output/'with_camera.log').open('w') as log:
            process=subprocess.Popen(['python3','-B',str(ROOT/'tools/record.py'),'--output-dir',str(target),
                                      '--with-camera','--seconds','5'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            wait(lambda:all(pub.get_subscription_count()>0 for pub in (lidar,imu,image_pub,info_pub,gimbal_pub)))
            source_frames(10)
            image=Image();image.header.stamp=node.get_clock().now().to_msg();image.height=2;image.width=2
            image.encoding='mono8';image.step=2;image.data=b'\x00\x01\x02\xff'
            info=CameraInfo();info.header=image.header;info.height=2;info.width=2
            image_pub.publish(image);info_pub.publish(info);gimbal_pub.publish(Imu())
            wait(lambda:process.poll() is not None,12)
        assert process.returncode==0,(output/'with_camera.log').read_text()
        meta=json.loads((target/'manifest.json').read_text())
        assert not meta['optional_topics_not_received'],meta
        assert all(meta['bag']['counts'][topic]>=1 for topic in
                   ('/image_raw','/camera_info','/red_standard_robot1/gimbal','/task1/gimbal_reset_epoch'))
        report['checks']['optional_camera_gimbal_and_preexisting_latched_epoch_recorded']=True
        wait(lambda:lidar.get_subscription_count()==0 and imu.get_subscription_count()==0)
        record_case('hangup_stop',count=10,stop_signal=signal.SIGHUP)
        report['checks']['terminal_hangup_gracefully_closes_bag']=True
        report['passed']=True
    except Exception as exc:
        import traceback
        report.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
