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
from std_msgs.msg import Header, String
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
    for name in ('start_match.bash','set_target.bash','set_target.py',
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
    def __init__(self, master, launch, corridor=False):
        super().__init__(master,launch)
        from array import array
        from verify_perception_offline import render_marker
        self.image=render_marker(3,(.06,.03,1.2),marker_size_m=self.marker_size)
        self.image_data=array('B',self.image.tobytes())
        self.targets=[];self.bases=[];self.grids=[];self.advice=[];self.map_status=[]
        self.create_subscription(MarkerObservation,'/perception/marker_cam',self.observations.append,qos_profile_sensor_data)
        self.create_subscription(TargetState,'/estimation/target_state',self.targets.append,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/gimbal/base_pose',self.bases.append,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map/grid',self.grids.append,latched_qos(1))
        self.create_subscription(ChassisAdvice,'/chassis/advice',self.advice.append,qos_profile_sensor_data)
        self.create_subscription(String,'/team43/cloud_mapping/status',self.map_status.append,10)
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
        if corridor:
            points=[]
            for y in np.linspace(-6.,6.,121):
                for z in np.linspace(floor,floor+1.8,11):
                    points.extend(((1.25,y,z),(-1.25,y,z)))
            for x in np.linspace(-1.25,1.25,26):
                for z in np.linspace(floor,floor+1.8,11):
                    points.extend(((x,-6.,z),(x,6.,z)))
            for x in np.linspace(-1.25,1.25,26):
                for y in np.linspace(-6.,6.,61):points.append((x,y,floor))
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



def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--count',type=int,default=30)
    parser.add_argument('--map-mode',choices=('live','corridor','none','copy'),default='live')
    args=parser.parse_args()
    if not Path('/.dockerenv').exists() or ROOT!=Path('/workspace'):
        parser.error('只允许在挂载到/workspace的隔离Docker容器运行')
    if not 1<=args.count<=1800:parser.error('count须1..1800')
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='88')
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
        launch=subprocess.Popen(['bash',str(project/'tools/start_match.bash'),'port:='+os.ttyname(slave),
                                 *(['map_mode:='+args.map_mode] if args.map_mode!='copy' else []),
                                 'output_dir:='+str(work/'score')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();probe=MatchProbe(master,launch,corridor=args.map_mode=='corridor')
        probe.wait(lambda: len(probe.statuses)>5 and (args.map_mode!='live' or
                   any(json.loads(s.data).get('accepted_scans',0)>5 for s in probe.map_status)),30.)
        assert not any(y!=0. or p!=0. for _,y,p in probe.commands),'裁判未通知ID就发出了非零运动指令'
        report['checks']['start_waits_for_target_without_manual_confirmation']=True
        probe.run_command(['bash',str(project/'tools/set_target.bash'),'3'],output/'set_target.log')
        probe.wait(lambda:any(y<0<p for _,y,p in probe.commands),20.)
        status=json.loads(probe.statuses[-1].data)
        assert status['continuous'] is True and status['operating_mode']=='match',status
        report['checks']['actual_set_target_script_enables_default_continuous_tracking']=True
        probe.wait(lambda:probe.tick_pub.get_subscription_count()>=1 and probe.start_pub.get_subscription_count()>=2,10.)
        probe.score_round(args.count,work/'score/43.csv')
        assert probe.bases and any(t.valid for t in probe.targets),'没有产生有效比赛坐标状态'
        assert any(a.valid for a in probe.advice),'没有有效方向建议'
        assert max(probe.visible_latencies)<=.08,'完整链路下存在超过80ms才可见的CSV行'
        tail=np.asarray([[float(v) for v in row.split(',')] for row in probe.rows_seen[-min(10,args.count):]])
        assert np.max(np.linalg.norm(tail[:,2:4]-[1.2625,-.03],axis=1))<.1,'目标坐标没有按本轮装车配置落到比赛起点系'
        assert np.max(np.linalg.norm(tail[:,4:6],axis=1))<.05,'静止底盘位置未归零'
        if args.map_mode in ('live','corridor'):
            probe.wait(lambda: bool(probe.grids),15.)
            assert (work/'score/43_map.pgm').is_file() and (work/'score/43_map.yaml').is_file(),'正式地图未保存'
        else:
            assert not probe.grids,'无图模式不应伪造地图'
            report['checks']['csv_and_nonzero_advice_without_any_map']=bool(any(a.valid and np.hypot(a.cmd_vx,a.cmd_vy)>.02 for a in probe.advice))
            assert report['checks']['csv_and_nonzero_advice_without_any_map']
            if args.map_mode=='copy':
                import yaml
                assert (work/'score/43_map.pgm').read_bytes() == (project/'maps/43_map.pgm').read_bytes(),'复制PGM被覆盖'
                assert yaml.safe_load((work/'score/43_map.yaml').read_text()) == yaml.safe_load((project/'maps/43_map.yaml').read_text()),'复制YAML参数被改变'
                report['checks']['selected_map_copied_and_not_overwritten']=True
        report['checks']['replay_origin_target_base_advice_csv']=True
        report['checks']['map_matches_requested_mode']=bool(probe.grids) == (args.map_mode in ('live','corridor'))
        first=len(probe.commands);probe.send_images=False;probe.check_stop(first)
        report['checks']['camera_loss_stops_real_cpp_bridge']=True
        probe.send_images=True;first=len(probe.commands)
        probe.wait(lambda:any(y<0<p for _,y,p in probe.commands[first:]))
        probe.arm(False);first=len(probe.commands);probe.run_for(.3)
        assert all(y==p==0. for _,y,p in probe.commands[first:])
        report['checks']['operator_disarm_stops_motion']=True
        # 再次开启，检验比赛结束时直接Ctrl+C也能向真实C++串口发出停止帧。
        probe.arm(True);first=len(probe.commands)
        probe.wait(lambda:any(y<0<p for _,y,p in probe.commands[first:]))
        shutdown_armed=True
        report.update(passed=True,map_mode=args.map_mode,ticks=args.count,csv_rows=len(probe.rows_seen),
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
        if probe is not None:probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for fd in (master,slave):
            if fd is not None:os.close(fd)
        if (work/'score').exists():shutil.copytree(work/'score',output/'score',dirs_exist_ok=True)
        if (work/'project/records').exists():shutil.copytree(work/'project/records',output/'records',dirs_exist_ok=True)
        shutil.rmtree(work)
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
