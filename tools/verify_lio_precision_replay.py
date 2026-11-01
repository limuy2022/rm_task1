#!/usr/bin/env python3
"""同一Gazebo传感器bag对照记录参数/当前装车参数；真值只作离线评价。"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import yaml

from team43_common.qos_utils import reliable_qos


def input_summary(bag):
    meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    rows = []
    for name in meta['relative_file_paths']:
        with sqlite3.connect((bag/name).resolve().as_uri()+'?mode=ro&immutable=1',uri=True) as db:
            for order in ('ASC','DESC'):
                row = db.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id '
                    "WHERE t.name='/sim/gazebo/livox/lidar' ORDER BY m.timestamp "+order+' LIMIT 1').fetchone()
                if row:rows.append(row)
    rows.sort(key=lambda row:row[0])
    a,b = [deserialize_message(row[1],PointCloud2).header.stamp for row in (rows[0],rows[-1])]
    header_span = b.sec+b.nanosec*1e-9-a.sec-a.nanosec*1e-9
    if header_span<=0:raise ValueError('录制没有有效传感器时间跨度')
    counts = {row['topic_metadata']['name']:row['message_count'] for row in meta['topics_with_message_count']}
    return {'storage_span_s':(rows[-1][0]-rows[0][0])*1e-9,'sensor_span_s':header_span,
            'expected_lidar':counts['/sim/gazebo/livox/lidar'],'expected_imu':counts['/sim/gazebo/imu']}


def run_variant(name,binary,record,out,summary,rate,parameters=None):
    work = out/name;work.mkdir()
    env = dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='96',
               TEAM43_LIO_PLANE_AUDIT=str(work/'plane_quality.json'))
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='96')
    config = yaml.safe_load((record/'point_lio.yaml').read_text())
    if parameters is not None:config['/**']['ros__parameters'] = dict(parameters)
    config['/**']['ros__parameters']['input_audit_path'] = str(work/'input_audit.csv')
    (work/'params.yaml').write_text(yaml.safe_dump(config))
    qos = {topic:{'reliability':'reliable','durability':'volatile','history':'keep_last','depth':1000}
           for topic in ('/sim/gazebo/livox/lidar','/sim/gazebo/imu')}
    (work/'qos.yaml').write_text(yaml.safe_dump(qos))
    backend=player=None;rclpy.init(args=[]);node = rclpy.create_node('precision_replay_observer')
    poses = []
    node.create_subscription(Odometry,'/test/precision/aft_mapped_to_init',
        lambda m:poses.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                              m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]),reliable_qos(1000))
    codes = {}
    try:
        with (work/'lio.log').open('w') as log,(work/'play.log').open('w') as play_log:
            backend = subprocess.Popen([str(binary),'--ros-args','--params-file',str(work/'params.yaml'),
                '-r','__ns:=/test/precision'],stdout=log,stderr=subprocess.STDOUT,env=env)
            deadline = time.monotonic()+15
            while not all(node.get_subscriptions_info_by_topic(t) for t in qos):
                if backend.poll() is not None:raise RuntimeError('LIO提前退出')
                if time.monotonic()>deadline:raise RuntimeError('LIO订阅发现超时')
                rclpy.spin_once(node,timeout_sec=.02)
            # 存储时间可能是录制墙钟；按实际传感器跨度换算播放速率，消息本身不改。
            playback_rate = summary['storage_span_s']/summary['sensor_span_s']*rate
            player = subprocess.Popen(['ros2','bag','play',str(record/'sensor_bag'),'--clock','50',
                '--rate',str(playback_rate),'--delay','1','--qos-profile-overrides-path',str(work/'qos.yaml'),
                '--topics','/sim/gazebo/livox/lidar','/sim/gazebo/imu'],
                stdout=play_log,stderr=subprocess.STDOUT,env=env)
            deadline = time.monotonic()+summary['sensor_span_s']/rate+45
            while player.poll() is None:
                if time.monotonic()>deadline:raise RuntimeError('回放超时')
                if backend.poll() is not None:raise RuntimeError('回放期间LIO退出')
                rclpy.spin_once(node,timeout_sec=.01)
            if player.returncode:raise RuntimeError('播放器异常退出')
            deadline = time.monotonic()+2
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
            backend.send_signal(signal.SIGINT);codes['lio']=backend.wait(timeout=10)
            codes['player']=player.returncode
    finally:
        for process in (player,backend):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
    with (work/'poses.csv').open('w') as stream:csv.writer(stream).writerows(poses)
    truth = np.loadtxt(record/'truth.csv',delimiter=',');p = np.asarray(poses)
    if len(p)<100:raise ValueError('定位输出不足')
    xy = np.c_[np.interp(p[:,0],truth[:,0],truth[:,1]),np.interp(p[:,0],truth[:,0],truth[:,2])]
    error = np.linalg.norm(p[:,1:3]-xy,axis=1)
    audit = list(csv.DictReader((work/'input_audit.csv').open()))[-1]
    complete = (int(audit['lidar_received'])==summary['expected_lidar'] and
                int(audit['imu_received'])==summary['expected_imu'])
    result = {'position_max_m':float(error.max()),'position_rmse_m':float(np.sqrt(np.mean(error**2))),
              'poses':len(p),'identical_input_counts':complete,'input_audit':audit,'exit_codes':codes,
              'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
              'map_resolution':config['/**']['ros__parameters']['map_resolution'],
              'space_downsample_leaf_size':config['/**']['ros__parameters']['space_downsample_leaf_size'],
              'precision_passed':bool(error.max()<.2 and np.sqrt(np.mean(error**2))<.15)}
    if (work/'plane_quality.json').exists():result['plane_quality']=json.loads((work/'plane_quality.json').read_text())
    (work/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-dir',required=True,type=Path)
    parser.add_argument('--baseline-binary',required=True,type=Path)
    parser.add_argument('--fixed-binary',required=True,type=Path)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--sensor-rate',type=float,default=.5)
    args = parser.parse_args()
    if not Path('/.dockerenv').exists() or Path.cwd()!=Path('/workspace'):
        parser.error('仅限/workspace隔离Docker')
    if not 0<args.sensor_rate<=1:parser.error('sensor-rate须在(0,1]')
    record=args.record_dir.resolve();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    summary=input_summary(record/'sensor_bag');report={'passed':False,'input':summary,'truth_published':False}
    try:
        baseline=run_variant('baseline',args.baseline_binary.resolve(),record,out,summary,args.sensor_rate)
        from ament_index_python.packages import get_package_share_directory
        from team43_gazebo.sensor_config import simulation_lio_parameters
        source=yaml.safe_load((Path(get_package_share_directory('team43_lio_backend'))/'config/small_point_lio_mid360.yaml').read_text())['/**']['ros__parameters']
        parameters=simulation_lio_parameters(source,'/sim/gazebo/livox/lidar','/sim/gazebo/imu','unused_audit_path')
        fixed=run_variant('fixed',args.fixed_binary.resolve(),record,out,summary,args.sensor_rate,parameters)
        report.update(baseline=baseline,fixed=fixed)
        report['passed'] = (fixed['precision_passed'] and fixed['identical_input_counts'] and baseline['identical_input_counts']
                            and all(v==0 for r in (baseline,fixed) for v in r['exit_codes'].values()))
    except Exception as exc:
        import traceback
        report.update(error=repr(exc),traceback=traceback.format_exc())
    (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
