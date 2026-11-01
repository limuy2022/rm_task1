#!/usr/bin/env python3
"""比赛入口验收的独立Gazebo采集/等时回放源；真值绝不发布到生产定位链。"""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import time

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu, PointCloud2
import yaml


def capture(output):
    import simulate_gazebo_vision as vision
    from simulate_gazebo_lio import GazeboLio, envelope
    from geometry_msgs.msg import Twist
    vision.MARKER = np.array([2., .12, .925])
    vision.MARKER_CENTER = vision.MARKER + [-.0042, 0., 0.]
    vision.MARKER_SIZE = .075
    vision.CAMERA_IN_BODY = np.array([.0625, .03, .165])
    vision.CAMERA_RPY = np.zeros(3)

    class Capture(vision.GazeboVision):
        def start(self):
            GazeboLio.start(self)

        def drive(self, now):
            cmd = Twist()
            if self.motion_start is not None:
                dt = now-self.motion_start
                if 0 <= dt < 32:
                    leg, phase = int(dt//4), dt % 4
                    if leg % 4 < 2: cmd.linear.x = (.18 if leg % 4 == 0 else -.18)*envelope(phase)
                    else: cmd.linear.y = (.12 if leg % 4 == 2 else -.12)*envelope(phase)
            self.cmd_pub.publish(cmd)

        def evaluate(self, elapsed_wall):
            # 本阶段仅采原始输入，算法判据在真实比赛入口回放中检查。
            gt = np.asarray(self.truth)
            np.savetxt(output/'truth.csv', gt, delimiter=',')
            return {'passed':len(self.images)>400 and len(self.estimates)>300,
                    'images':len(self.images),'lidar':self.published_frames,
                    'motion_start':self.motion_start,'wall_seconds':elapsed_wall,
                    'marker_world':vision.MARKER_CENTER.tolist(), 'truth_published_to_production':False}

    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    trial = Capture(output, 600., .5, True, 16, 'corridor', .03)
    report = {'passed':False}
    try:
        report = trial.run()
    finally:
        report["exit_codes"] = trial.close()
        report["passed"] = report["passed"] and all(code == 0 for code in report["exit_codes"].values())
        if rclpy.ok():rclpy.shutdown()
        (output/'capture_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if not report['passed']:raise RuntimeError('Gazebo原始采集样本不足')


def make_configs(record, work):
    from ament_index_python.packages import get_package_share_directory
    vehicle = yaml.safe_load((Path(get_package_share_directory('team43_bringup'))/'config/vehicle_field.yaml').read_text())
    p = vehicle['/**']['ros__parameters']
    p.update({'localization.sensor_in_base_xyz':[0.,0.,0.],
              'localization.sensor_in_base_rpy':[0.,0.,0.]})
    (work/'vehicle_sim.yaml').write_text(yaml.safe_dump(vehicle))
    config = yaml.safe_load((record/'point_lio.yaml').read_text())
    p = config['/**']['ros__parameters']
    # 只适配平装仿真传感器的SI单位、地面高度与话题，不调滤波器参数。
    p.update(use_sim_time=False, approximate_floor_from_height=True, lidar_height_m=.7+float(p["extrinsic_T"][2]),
             input_audit_path=str(work/'replay_input_audit.csv'))
    (work/'lio_sim.yaml').write_text(yaml.safe_dump(config))
    return ['vehicle_config:='+str(work/'vehicle_sim.yaml'),
            'lio_config:='+str(work/'lio_sim.yaml'),'camera_config:='+str(record/'camera.yaml')]


def replay(record, control, manifest):
    from team43_common.qos_utils import reliable_qos
    mapping = {'/sim/gazebo/livox/lidar':(PointCloud2,'/red_standard_robot1/livox/lidar'),
               '/sim/gazebo/imu':(Imu,'/red_standard_robot1/livox/imu'),
               '/sim/gazebo/camera/image':(Image,'/image_raw')}
    meta = yaml.safe_load((record/'sensor_bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    rows = []
    connections = []
    for name in meta['relative_file_paths']:
        db = sqlite3.connect((record/'sensor_bag'/name).resolve().as_uri()+'?mode=ro&immutable=1',uri=True)
        connections.append(db)
        for stamp, ident, topic in db.execute('SELECT m.timestamp,m.id,t.name FROM messages m JOIN topics t ON t.id=m.topic_id ORDER BY m.timestamp'):
            if topic in mapping:
                data = db.execute('SELECT data FROM messages WHERE id=?',(ident,)).fetchone()[0]
                msg = deserialize_message(data, mapping[topic][0])
                ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                event_ns=ns
                if isinstance(msg,PointCloud2):
                    # 一帧要等最后一次真实扫描发生后才可发送，不能按帧头提前交付未来点。
                    event_ns=int(np.max(np.ndarray((msg.width*msg.height,),dtype='<f8',
                        buffer=msg.data,offset=18,strides=(26,))))
                rows.append((event_ns, ns, len(connections)-1, ident, topic))
    rows.sort()
    if not rows:raise ValueError('没有原始传感器消息')
    rclpy.init();node = rclpy.create_node('gazebo_raw_match_replay')
    publishers = {t:node.create_publisher(cls,dst,reliable_qos(1000)) for t,(cls,dst) in mapping.items()}
    deadline = time.monotonic()+30
    while not all(p.get_subscription_count() for p in publishers.values()):
        if time.monotonic()>deadline:raise RuntimeError('真实比赛输入订阅未全部建立')
        rclpy.spin_once(node,timeout_sec=.01)
    offset = time.time_ns()+500_000_000-rows[0][0]
    counts = {t:0 for t in mapping};lag=[];suppressed=[]
    manifest.write_text(json.dumps({'offset_ns':offset,'source_start_ns':rows[0][0],
                                    'source_end_ns':rows[-1][0],'equal_sensor_intervals':True})+'\n')
    try:
        for event_ns, source_ns, dbindex, ident, topic in rows:
            remaining = (event_ns+offset-time.time_ns())*1e-9
            if remaining>0:time.sleep(remaining)
            lag.append(max(0.,(time.time_ns()-event_ns-offset)*1e-9))
            data = connections[dbindex].execute('SELECT data FROM messages WHERE id=?',(ident,)).fetchone()[0]
            msg = deserialize_message(data,mapping[topic][0])
            msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(source_ns+offset,10**9)
            if isinstance(msg,PointCloud2):
                if msg.is_bigendian or msg.point_step!=26 or msg.height!=1:
                    raise ValueError('回放仅接受采集器生成的26字节小端Livox PointCloud2')
                fields={f.name:(f.offset,f.datatype,f.count) for f in msg.fields}
                if fields.get('timestamp')!=(18,8,1):
                    raise ValueError('点时间字段必须为offset18的绝对纳秒FLOAT64')
                # 官方PointCloud2内各点绝对纳秒时间与header一起平移，扫描间隔不变。
                raw = bytearray(msg.data)
                np.ndarray((msg.width*msg.height,), dtype='<f8',buffer=raw,offset=18,strides=(26,))[:] += offset
                msg.data = bytes(raw)
            if topic.endswith('/image') and control.exists():
                suppressed.append(source_ns)
                continue
            publishers[topic].publish(msg);counts[topic]+=1
    except KeyboardInterrupt:
        pass
    finally:
        manifest.write_text(json.dumps({'offset_ns':offset,'source_start_ns':rows[0][0],
            'source_end_ns':rows[-1][0],'equal_sensor_intervals':True,'counts':counts,
            'max_source_lag_s':max(lag) if lag else None,
            'suppressed_camera_source_ns':suppressed})+'\n')
        for db in connections:db.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['capture','replay'])
    parser.add_argument('--record-dir',type=Path,required=True)
    parser.add_argument('--control',type=Path)
    parser.add_argument('--manifest',type=Path)
    args = parser.parse_args()
    if not Path('/.dockerenv').exists() or Path.cwd()!=Path('/workspace'):
        parser.error('仅允许/workspace隔离容器')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID') in (None,'0'):
        parser.error('必须设置ROS_LOCALHOST_ONLY=1及独立非零ROS_DOMAIN_ID')
    if args.mode=='capture':
        if not os.environ.get('IGN_PARTITION'):parser.error('采集必须设置独立IGN_PARTITION')
        capture(args.record_dir.resolve())
    else:
        if args.control is None or args.manifest is None:parser.error('回放必须提供control和manifest路径')
        replay(args.record_dir.resolve(), args.control, args.manifest)
