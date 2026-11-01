#!/usr/bin/env python3
"""容器内执行比赛一键全链：合成传感器＋真实Point-LIO/视觉/TF/控制/提交＋串口PTY。

仅替换Livox硬件驱动进程，绝不加载SDK或连接雷达；正式评分话题统一重映射到/test。
使用本轮装车配置和真实start_match/set_target脚本，不修改生产启动默认值。
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import tempfile
import time
import tty

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, OccupancyGrid
from team43_interfaces.msg import MarkerObservation, TargetState, ChassisAdvice
from team43_common.qos_utils import latched_qos, reliable_qos
from verify_visual_tracking_dds import Probe, checksum

ROOT = Path(__file__).resolve().parents[1]


def fixture_workspace(work):
    """只在/tmp创建ROS包覆盖层；SDK可执行程序必定被无硬件替身替换。"""
    prefix=work/'overlay'
    index=prefix/'share/ament_index/resource_index/packages';index.mkdir(parents=True)
    for name in ('team43_bringup','livox_ros_driver2'):(index/name).write_text('')
    share=prefix/'share/team43_bringup'
    shutil.copytree(get_package_share_directory('team43_bringup'), share, symlinks=False)
    (prefix/'lib').mkdir()
    (prefix/'lib/team43_bringup').symlink_to(Path(get_package_prefix('team43_bringup'))/'lib/team43_bringup')
    driver=prefix/'lib/livox_ros_driver2/livox_ros_driver2_node';driver.parent.mkdir()
    driver.write_text('''#!/usr/bin/python3
import rclpy
from rclpy.node import Node
rclpy.init()
node=Node('test_livox_no_hardware')
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
''');driver.chmod(0o755)
    real_launch=Path(get_package_share_directory('team43_bringup'))/'launch/match.launch.py'
    (share/'launch/match.launch.py').write_text('''from launch import LaunchDescription
from launch_ros.actions import SetRemap
import importlib.util
_spec=importlib.util.spec_from_file_location('actual_match_launch',%r)
_module=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_module)
def generate_launch_description():
    actual=_module.generate_launch_description()
    return LaunchDescription([SetRemap(src='/judge/tick',dst='/test/match/tick'),
        SetRemap(src='/judge/replay_start',dst='/test/match/start'),*actual.entities])
''' % str(real_launch))
    project=work/'project';(project/'tools').mkdir(parents=True)
    (project/'maps').mkdir()
    for name in ('43_map.pgm','43_map.yaml'):shutil.copy2(ROOT/'maps'/name,project/'maps'/name)
    for name in ('env.bash','start_match.bash','set_target.bash','set_target.py',
                 'start_mapping.bash','save_mapping.bash','save_mapping.py','record.bash','record.py'):
        shutil.copy2(ROOT/'tools'/name,project/'tools'/name)
    (project/'.livox').mkdir();(project/'.livox/env.bash').write_text('# 无硬件测试，不加载SDK驱动进程。\n')
    (project/'install').mkdir()
    (project/'install/local_setup.bash').write_text(
        'source '+str(Path(get_package_prefix('team43_bringup')).parent/'setup.bash')+'\n'
        'export AMENT_PREFIX_PATH='+str(prefix)+':"$AMENT_PREFIX_PATH"\n')
    # 启动前独立确认覆盖层生效，杜绝误执行真实驱动。
    found=subprocess.check_output(['bash','-c','source '+str(project/'install/local_setup.bash')+
                                   ' && ros2 pkg prefix livox_ros_driver2'],text=True).strip()
    if found!=str(prefix):raise RuntimeError('硬件替身未生效，拒绝运行比赛启动器')
    return project, prefix


class MatchProbe(Probe):
    def __init__(self, master, launch):
        super().__init__(master,launch)
        from array import array
        from verify_perception_offline import render_marker
        self.image=render_marker(3,(.06,.03,1.2),marker_size_m=self.marker_size)
        self.image_data=array('B',self.image.tobytes())
        self.targets=[];self.bases=[];self.grids=[];self.advice=[]
        self.joint_timing=[];self.base_timing=[]
        self.command_wall_offset=time.time()-time.monotonic()
        self.create_subscription(JointState,'/gimbal/joint_states',
            lambda m:self.joint_timing.append([time.time(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9]),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/gimbal/base_pose_raw',
            lambda m:self.base_timing.append([time.time(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9]),qos_profile_sensor_data)
        self.create_subscription(MarkerObservation,'/perception/marker_cam',self.observations.append,qos_profile_sensor_data)
        self.create_subscription(TargetState,'/estimation/target_state',self.targets.append,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/gimbal/base_pose',self.bases.append,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map/grid',self.grids.append,latched_qos(1))
        self.create_subscription(ChassisAdvice,'/chassis/advice',self.advice.append,qos_profile_sensor_data)
        self.tick_pub=self.create_publisher(Header,'/test/match/tick',reliable_qos())
        self.start_pub=self.create_publisher(Header,'/test/match/start',latched_qos(1))
        self.lidar_pub=self.create_publisher(PointCloud2,'/red_standard_robot1/livox/lidar',qos_profile_sensor_data)
        self.lidar_imu_pub=self.create_publisher(Imu,'/red_standard_robot1/livox/imu',qos_profile_sensor_data)
        import yaml
        cfg=yaml.safe_load((Path(get_package_share_directory('team43_lio_backend'))/'config/small_point_lio_mid360.yaml').read_text())['/**']['ros__parameters']
        up=np.asarray(cfg['output_world_up'],float);up/=np.linalg.norm(up);self.up=up
        v=np.cross(up,[0.,0.,1.]);K=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
        rotation=np.eye(3)+K+K@K/(1+up[2])
        floor=float(up@np.asarray(cfg['extrinsic_T'])-cfg['lidar_height_m'])
        points=[]
        for a in range(-12,13):
            for b in range(11):
                for xyz in ((2.,a*.15,floor+b*.18),(-2.,a*.15,floor+b*.18),
                            (a*.15,2.,floor+b*.18),(a*.15,-2.,floor+b*.18)):
                    points.append(xyz)
        for a in range(-12,13):
            for b in range(-12,13):points.append((a*.15,b*.15,floor))
        self.points=np.asarray(points)@rotation-np.asarray(cfg['extrinsic_T'])
        self.last_lidar=self.last_lidar_imu=0.
        self.tick_stamps=[];self.visible_latencies=[];self.rows_seen=[]

    def pump_sensors(self):
        now=time.monotonic()
        if now-self.last_lidar_imu>=.005:
            imu=Imu();imu.header.stamp=self.get_clock().now().to_msg()
            imu.header.frame_id='livox_frame'
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,self.up)
            self.lidar_imu_pub.publish(imu);self.last_lidar_imu=now
        if now-self.last_lidar>=.1:
            end=self.get_clock().now().nanoseconds;begin=end-100_000_000
            cloud=PointCloud2();cloud.header.stamp.sec,cloud.header.stamp.nanosec=divmod(begin,1_000_000_000)
            cloud.header.frame_id='livox_frame';cloud.height=1;cloud.width=len(self.points)
            cloud.point_step=26;cloud.row_step=26*cloud.width
            cloud.fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in
                          [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]
            cloud.data=b''.join(struct.pack('<ffffBBd',*p,100.,0x10,0,float(begin+i*99_999_000/(len(self.points)-1)))
                                for i,p in enumerate(self.points))
            self.lidar_pub.publish(cloud);self.last_lidar=now

    def pump(self):
        self.pump_sensors()
        super().pump()

    def run_command(self, command, log):
        with log.open('w') as stream:
            process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT)
            self.wait(lambda: process.poll() is not None,30.)
            if process.returncode:raise AssertionError('命令失败：'+log.read_text()[-3000:])

    def score_round(self, count, csv_path):
        # 评分端删文件发生在参赛程序启动之后，不能要求选手此时重启。
        csv_path.parent.mkdir(parents=True,exist_ok=True)
        csv_path.write_text('old round to be removed\n');csv_path.unlink()
        self.run_for(.1)
        assert not csv_path.exists(),'首拍前不应提前占住CSV'
        # 裁判和观察器为独立进程；大图像/雷达源不得共享评分DDS发送队列。
        transcript=csv_path.parent/'tick_stamps.txt'
        delays=csv_path.parent/'visible_delays.json'
        observer_log=csv_path.parent/'observer.json'
        observer=source=None
        try:
            with observer_log.open('w') as log:
                observer=subprocess.Popen(['python3','-B',str(ROOT/'tools/observe_submission.py'),str(csv_path),
                    '--live-state','--count',str(count),'--timeout',str(count*.1+30),
                    '--timings-json',str(delays)],stdout=log,stderr=subprocess.STDOUT)
                source=subprocess.Popen(['python3','-B',str(ROOT/'tools/match_tick_source.py'),
                    '--count',str(count),'--transcript',str(transcript)])
                self.wait(lambda:source.poll() is not None and observer.poll() is not None,count*.1+35)
            assert source.returncode==0,'独立评分源退出异常'
            observed=json.loads(observer_log.read_text())
            assert observer.returncode==0 and observed['passed'],observed
            stamps=transcript.read_text().splitlines()
            rows=csv_path.read_text().splitlines()
            assert len(rows)==len(stamps)==count,'源拍数与实际CSV行数不一致'
            for line,stamp in zip(rows,stamps):
                fields=line.split(',')
                assert len(fields)==9 and all(math.isfinite(float(v)) for v in fields),line
                assert fields[8]==stamp,'CSV未使用相应tick的原始stamp'
            self.rows_seen=rows
            self.visible_latencies=json.loads(delays.read_text())
            self.tick_stamps=[float(stamp) for stamp in stamps]
            self.run_for(1.2)
            assert len(csv_path.read_text().splitlines())==count
        finally:
            for process in (source,observer):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:process.kill();process.wait()


def run_static_source(master, control, audit):
    """传感器与MCU独立于观察器进程；实际发送间隔保存以便排查中断。"""
    from sensor_msgs.msg import Image
    rclpy.init()
    source=MatchProbe(master,None)
    for subscription in tuple(source.subscriptions):source.destroy_subscription(subscription)
    times={'lidar':[],'imu':[],'mcu':[],'image':[]}
    seq=0; last_mcu=last_image=0.;begin=time.monotonic()
    try:
        while True:
            before_lidar,before_imu=source.last_lidar,source.last_lidar_imu
            source.pump_sensors()
            if source.last_lidar!=before_lidar:times['lidar'].append(time.time())
            if source.last_lidar_imu!=before_imu:times['imu'].append(time.time())
            now=time.monotonic()
            if now-last_mcu>=.02:
                seq+=1
                data=b'\xaa\xff\x01\x1a'+struct.pack('<HI3h3h4h',seq&65535,
                    int((now-begin)*1e6)&0xffffffff,0,0,1365,0,0,0,32760,0,0,0)
                os.write(master,data+checksum(data));last_mcu=now;times['mcu'].append(time.time())
            if now-last_image>=.05 and not control.exists():
                message=Image();message.header.stamp=source.get_clock().now().to_msg()
                message.height,message.width=source.image.shape
                message.encoding='mono8';message.step=message.width;message.data=source.image_data
                source.image_pub.publish(message);last_image=now;times['image'].append(time.time())
            time.sleep(.001)
    except KeyboardInterrupt:
        pass
    finally:
        data={kind:{'count':len(stamps),'max_gap_s':max(np.diff(stamps)) if len(stamps)>1 else None,
                    'first_wall_s':stamps[0] if stamps else None,'last_wall_s':stamps[-1] if stamps else None}
              for kind,stamps in times.items()}
        audit.write_text(json.dumps(data)+'\n')
        source.destroy_node()
        if rclpy.ok():rclpy.shutdown()


class IndependentMatchProbe(MatchProbe):
    def __init__(self,master,launch,work,output,domain):
        super().__init__(master,launch)
        for publisher in (self.lidar_pub,self.lidar_imu_pub,self.image_pub):self.destroy_publisher(publisher)
        self.image_control=work/'pause_camera'
        self.camera_fault_events=[]
        self.source_log=(output/'static_source.log').open('w')
        self.source=subprocess.Popen(['python3','-B',str(Path(__file__).resolve()),
            '--source-fd',str(master),'--source-control',str(self.image_control),
            '--source-audit',str(output/'static_source_timing.json'),'--ros-domain-id',str(domain)],
            pass_fds=(master,),stdout=self.source_log,stderr=subprocess.STDOUT)

    def pump_sensors(self):
        if self.send_images:self.image_control.unlink(missing_ok=True)
        else:self.image_control.touch(exist_ok=True)
        if self.source.poll() is not None:raise RuntimeError('独立合成源提前退出')

    def pump(self):
        self.pump_sensors()
        visible=self.send_images;self.send_images=False;self.last_imu=float('inf')
        try:Probe.pump(self)
        finally:self.send_images=visible

    def close_source(self):
        if self.source.poll() is None:
            self.source.send_signal(signal.SIGINT)
            try:self.source.wait(timeout=5.)
            except subprocess.TimeoutExpired:self.source.kill();self.source.wait()
        self.source_log.close()


class GazeboMatchProbe(MatchProbe):
    """生产全链输入来自独立进程回放的Gazebo原始数据，保留真实串口桥。"""
    def __init__(self, master, launch, record, work, output):
        super().__init__(master, launch)
        self.record = record
        self.inject_camera_fault=False
        self.camera_fault_paused=False
        self.camera_fault_events=[]
        self.create_subscription(Header,'/test/match/tick',self.on_score_tick,reliable_qos())
        for publisher in (self.lidar_pub,self.lidar_imu_pub,self.image_pub):
            self.destroy_publisher(publisher)
        self.image_control = work/'pause_camera'
        self.manifest = output/'source_manifest.json'
        self.source_log = (output/'sensor_replay.log').open('w')
        self.source = subprocess.Popen(['python3','-B',str(ROOT/'tools/afternoon_gazebo_inputs.py'),
            'replay','--record-dir',str(record),'--control',str(self.image_control),
            '--manifest',str(self.manifest)],stdout=self.source_log,stderr=subprocess.STDOUT)

    def on_score_tick(self, message):
        if self.inject_camera_fault:
            paused=100 <= int(message.frame_id) < 120
            if paused!=self.camera_fault_paused:
                self.camera_fault_events.append([int(message.frame_id),time.time(),paused])
            self.camera_fault_paused=paused

    def pump_sensors(self):
        if self.send_images and not self.camera_fault_paused:self.image_control.unlink(missing_ok=True)
        else:self.image_control.touch(exist_ok=True)
        if self.source.poll() is not None:
            raise RuntimeError('Gazebo原始输入回放提前结束，采集时长不足或源异常')

    def pump(self):
        # Probe负责PTY与观察，不再发布合成图像；原始图像由独立进程发送。
        self.pump_sensors()
        visible = self.send_images
        self.send_images = False
        try:Probe.pump(self)
        finally:self.send_images = visible

    def precision(self):
        source = json.loads(self.manifest.read_text())
        delta = source['offset_ns']*1e-9
        truth = np.loadtxt(self.record/'truth.csv',delimiter=',')
        rows = np.asarray([[float(v) for v in row.split(',')] for row in self.rows_seen])
        stamps = rows[:,8]-delta
        assert truth[0,0]<=stamps[0]<=stamps[-1]<=truth[-1,0], '评分时段超出Gazebo原始数据'
        origin = np.array([np.interp(stamps[0],truth[:,0],truth[:,k]) for k in (1,2)])
        yaw = np.interp(stamps[0],truth[:,0],truth[:,4])
        rot = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
        expected_base = (np.c_[np.interp(stamps,truth[:,0],truth[:,1]),
                               np.interp(stamps,truth[:,0],truth[:,2])]-origin)@rot.T
        marker = np.array(json.loads((self.record/'capture_result.json').read_text())['marker_world'][:2])
        expected_target = rot@(marker-origin)
        # 第0拍异步起点/TF回调可能尚未融合；所有行仍须按时写出。精度单独统计1秒后。
        elapsed=stamps-stamps[0]
        selected = elapsed>1.
        if self.inject_camera_fault:
            # 100..119拍停止图像；另给1秒恢复。只排除精度，逐拍截止检查不排除。
            selected &= (elapsed<10.) | (elapsed>13.)
            outage=(elapsed>10.8)&(elapsed<11.8)
            assert np.count_nonzero(outage)>=5
            assert np.all(np.linalg.norm(rows[outage,6:8],axis=1)==0.),'相机断流时建议未归零'
            assert np.any(np.linalg.norm(rows[outage,2:4],axis=1)==0.),'相机断流没有降级目标状态'
            assert np.all(np.linalg.norm(rows[elapsed>13.,2:4],axis=1)>0.),'图像恢复后目标未恢复'
        base_error = np.linalg.norm(rows[selected,4:6]-expected_base[selected],axis=1)
        target_error = np.linalg.norm(rows[selected,2:4]-expected_target,axis=1)
        prediction_error = np.linalg.norm(rows[selected,:2]-expected_target,axis=1)
        metrics = {'base_max_m':float(base_error.max()),'base_rmse_m':float(np.sqrt(np.mean(base_error**2))),
                   'target_max_m':float(target_error.max()),'target_rmse_m':float(np.sqrt(np.mean(target_error**2))),
                   'prediction_max_m':float(prediction_error.max()),
                   'truth_motion_span_m':float(np.linalg.norm(np.ptp(expected_base,axis=0))),
                   'zero_target_rows':int(np.sum(np.linalg.norm(rows[:,2:4],axis=1)==0.)),
                   'all_rows_deadline_checked':True,'precision_warmup_s':1.,'camera_fault_ticks':[100,120] if self.inject_camera_fault else None}
        assert metrics['truth_motion_span_m']>.3, metrics
        assert metrics['base_max_m']<.2 and metrics['base_rmse_m']<.1, metrics
        assert metrics['target_max_m']<.35 and metrics['target_rmse_m']<.15, metrics
        assert metrics['prediction_max_m']<.4,metrics
        return metrics

    def close_source(self):
        if self.source.poll() is None:
            self.source.send_signal(signal.SIGINT)
            try:self.source.wait(timeout=5.)
            except subprocess.TimeoutExpired:self.source.kill();self.source.wait()
        self.source_log.close()



def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--source-fd',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--source-control',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--source-audit',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--count',type=int,default=30)
    parser.add_argument('--inject-camera-fault',action='store_true',help='Gazebo回放时第100..119拍切断相机，检验CSV持续和恢复')
    parser.add_argument('--ros-domain-id',type=int,default=88,choices=range(1,101))
    parser.add_argument('--gazebo-record',type=Path,help='可选真实Gazebo原始采集目录；不再注入合成雷达/图像')
    args=parser.parse_args()
    if not Path('/.dockerenv').exists() or ROOT!=Path('/workspace'):
        parser.error('只允许在挂载到/workspace的隔离Docker容器运行')
    if not 1<=args.count<=9000:parser.error('count须1..9000')
    if args.inject_camera_fault and (not args.gazebo_record or args.count<150):
        parser.error('断流演练需要Gazebo记录与至少150拍')
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(args.ros_domain_id))
    if args.source_fd is not None:
        if args.source_control is None or args.source_audit is None:parser.error('独立源缺少控制/审核路径')
        run_static_source(args.source_fd,args.source_control,args.source_audit)
        return 0
    if args.output_dir is None:parser.error('必须提供output-dir')
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=False)
    work=Path(tempfile.mkdtemp(prefix='match_workflow_'))
    report=dict(passed=False,hardware_accessed=False,kind='production_match_workflow',checks={},
                limitations=['Livox硬件驱动为无SDK进程替身，雷达/IMU/图像为合成输入',
                             '云台固定反馈，串口只接新建PTY，不证明真实电机闭环或NUC截止时间'])
    launch=probe=None;master=slave=None;shutdown_armed=False
    try:
        project,prefix=fixture_workspace(work)
        master,slave=os.openpty();tty.setraw(slave);os.set_blocking(master,False)
        log=(output/'match.log').open('w')
        extra=[]
        if args.gazebo_record:
            from afternoon_gazebo_inputs import make_configs
            extra=make_configs(args.gazebo_record.resolve(),work)
            report['limitations'][0]='Gazebo三维点云、IMU和渲染图像等时回放；平装仿真外参，硬件驱动替身'
            report['kind']='gazebo_raw_production_match_workflow'
        launch=subprocess.Popen(['bash',str(project/'tools/start_match.bash'),'port:='+os.ttyname(slave),
                                 'output_dir:='+str(work/'score'),*extra],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init()
        probe=(GazeboMatchProbe(master,launch,args.gazebo_record.resolve(),work,output)
               if args.gazebo_record else IndependentMatchProbe(master,launch,work,output,args.ros_domain_id))
        if args.gazebo_record:probe.inject_camera_fault=args.inject_camera_fault
        target='0' if args.gazebo_record else '3'
        moving=lambda commands:any((y!=0. or p!=0.) if args.gazebo_record else (y<0<p)
                                   for _,y,p in commands)
        probe.wait(lambda:len(probe.statuses)>5,30.)
        assert not any(y!=0. or p!=0. for _,y,p in probe.commands),'裁判未通知ID就发出了非零运动指令'
        report['checks']['start_waits_for_target_without_manual_confirmation']=True
        probe.run_command(['bash',str(project/'tools/set_target.bash'),target],output/'set_target.log')
        probe.wait(lambda:moving(probe.commands),20.)
        status=json.loads(probe.statuses[-1].data)
        assert status['continuous'] is True and status['operating_mode']=='match',status
        report['checks']['actual_set_target_script_enables_default_continuous_tracking']=True
        probe.wait(lambda:probe.tick_pub.get_subscription_count()>=1 and probe.start_pub.get_subscription_count()>=2,10.)
        probe.score_round(args.count,work/'score/43.csv')
        feedback_faults=[json.loads(s.data) for s in probe.statuses
            if probe.tick_stamps[0]<=json.loads(s.data).get('status_stamp',0.)<=probe.tick_stamps[-1]
            and json.loads(s.data).get('fault')]
        assert not feedback_faults,'评分过程中出现锁存保护故障：'+str(feedback_faults[:1])
        report['checks']['no_latched_feedback_fault_during_scoring']=True
        assert probe.bases and any(t.valid for t in probe.targets),'没有产生有效比赛坐标状态'
        assert any(a.valid for a in probe.advice),'没有有效方向建议'
        assert max(probe.visible_latencies)<=.08,'完整链路下存在超过80ms才可见的CSV行'
        tail=np.asarray([[float(v) for v in row.split(',')] for row in probe.rows_seen[-min(10,args.count):]])
        if args.gazebo_record:
            report['gazebo_precision']=probe.precision()
            if args.inject_camera_fault:
                report['checks']['camera_loss_keeps_tick_csv_and_recovers']=True
        else:
            assert np.max(np.linalg.norm(tail[:,2:4]-[1.2625,-.03],axis=1))<.1,'目标坐标没有按本轮装车配置落到比赛起点系'
            assert np.max(np.linalg.norm(tail[:,4:6],axis=1))<.05,'静止底盘位置未归零'
        assert not probe.grids,'固定交付模式不应产生运行地图'
        assert any(a.valid and np.hypot(a.cmd_vx,a.cmd_vy)>.02 for a in probe.advice)
        import yaml
        assert (work/'score/43_map.pgm').read_bytes() == (project/'maps/43_map.pgm').read_bytes(),'复制PGM被覆盖'
        assert yaml.safe_load((work/'score/43_map.yaml').read_text()) == yaml.safe_load((project/'maps/43_map.yaml').read_text()),'复制YAML参数被改变'
        report['checks']['selected_map_copied_without_runtime_map_or_overwrite']=True
        report['checks']['replay_origin_target_base_advice_csv']=True
        first=len(probe.commands);probe.send_images=False;probe.check_stop(first)
        report['checks']['camera_loss_stops_real_cpp_bridge']=True
        probe.send_images=True;first=len(probe.commands)
        probe.wait(lambda:moving(probe.commands[first:]))
        probe.arm(False);first=len(probe.commands);probe.run_for(.3)
        assert all(y==p==0. for _,y,p in probe.commands[first:])
        report['checks']['operator_disarm_stops_motion']=True
        # 再次开启，检验比赛结束时直接Ctrl+C也能向真实C++串口发出停止帧。
        probe.arm(True);first=len(probe.commands)
        probe.wait(lambda:moving(probe.commands[first:]))
        shutdown_armed=True
        report.update(passed=True,ticks=args.count,csv_rows=len(probe.rows_seen),
                      target_messages=len(probe.targets),valid_target_messages=sum(bool(t.valid) for t in probe.targets),
                      base_messages=len(probe.bases),map_messages=len(probe.grids),serial_commands=len(probe.commands),
                      max_visible_latency_s=max(probe.visible_latencies),
                      p99_visible_latency_s=float(np.percentile(probe.visible_latencies,99)))
    except Exception as exc:
        import traceback
        report.update(error=repr(exc),traceback=traceback.format_exc())
        if probe is not None:report['last_statuses']=[json.loads(s.data) for s in probe.statuses[-5:]]
    finally:
        if launch is not None:
            if launch.poll() is None:launch.send_signal(signal.SIGINT)
            try:report['launch_exit_code']=launch.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGKILL);launch.wait();report['launch_exit_code']=launch.returncode
            log.close()
            if shutdown_armed:
                tail=bytearray(probe.serial)
                while True:
                    try:tail.extend(os.read(master,65536))
                    except (BlockingIOError,OSError):break
                frames=[]
                while len(tail)>=14:
                    frame=bytes(tail[:14]);del tail[:14]
                    if frame[:4]!=b'\xaa\xee\x30\x08' or checksum(frame[:12])!=frame[12:]:
                        report['passed']=False;report['shutdown_serial_error']='坏停止帧';break
                    frames.append(struct.unpack('<ff',frame[4:12]))
                stopped=(not tail and len(frames)>=3 and all(y==p==0. for y,p in frames[-3:]))
                report['checks']['ctrl_c_while_armed_emits_serial_zero_frames']=stopped
                report['shutdown_tail_frames']=frames
                report['passed']=report['passed'] and stopped
            report['passed']=report['passed'] and report['launch_exit_code']==0
            report['clean_shutdown']='process has died' not in (output/'match.log').read_text()
            report['passed']=report['passed'] and report['clean_shutdown']
        if isinstance(probe,(GazeboMatchProbe,IndependentMatchProbe)):
            probe.close_source()
            report['camera_fault_events']=probe.camera_fault_events
            if args.inject_camera_fault and probe.manifest.exists():
                source=json.loads(probe.manifest.read_text())
                injected=len(source.get('suppressed_camera_source_ns',[]))>=15
                report['checks']['camera_fault_actually_injected']=injected
                report['passed']=report['passed'] and injected
        if probe is not None:
            (output/'feedback_timing.json').write_text(json.dumps({'joints':probe.joint_timing,'base':probe.base_timing,
                'serial_commands':[[t+probe.command_wall_offset,y,p] for t,y,p in probe.commands],
                'statuses':[json.loads(s.data) for s in probe.statuses]},ensure_ascii=False)+'\n')
            probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for fd in (master,slave):
            if fd is not None:os.close(fd)
        for name in ('vehicle_sim.yaml','lio_sim.yaml','replay_input_audit.csv'):
            if (work/name).exists():shutil.copy2(work/name,output/name)
        if (work/'score').exists():
            import yaml
            preserved=all((work/'score'/name).exists() for name in ('43_map.pgm','43_map.yaml'))
            if preserved:
                preserved=((work/'score/43_map.pgm').read_bytes()==(project/'maps/43_map.pgm').read_bytes() and
                    yaml.safe_load((work/'score/43_map.yaml').read_text())==yaml.safe_load((project/'maps/43_map.yaml').read_text()))
            report['checks']['map_copy_survives_shutdown']=preserved
            report['passed']=report['passed'] and preserved
            shutil.copytree(work/'score',output/'score',dirs_exist_ok=True)
        if (work/'project/records').exists():shutil.copytree(work/'project/records',output/'records',dirs_exist_ok=True)
        shutil.rmtree(work)
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
