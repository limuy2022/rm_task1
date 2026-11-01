#!/usr/bin/env python3
"""Gazebo真实雷达/IMU→LIO→平行墙模型与实时防撞建议；真值只用于评价。"""
import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import time
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from PIL import Image,ImageDraw

from team43_interfaces.msg import TargetState,ChassisAdvice
from team43_common.qos_utils import latched_qos,sensor_qos
from team43_gazebo.process_shutdown import stop_processes
from simulate_gazebo_lio import GazeboLio,NS,ROOT,envelope,seconds,point_lio_image_build


class CorridorTrial(GazeboLio):
    def __init__(self,output,timeout,factor):
        super().__init__(output,timeout,factor,False,16)
        # 场景唯一来源为已安装的worlds/corridor.sdf，不在脚本中另造房间。
        self.base_pub = self.create_publisher(Odometry,NS+'/corridor/base',sensor_qos())
        self.target_pub = self.create_publisher(TargetState,NS+'/corridor/target',sensor_qos())
        self.guard_pub = self.create_publisher(PointCloud2,NS+'/corridor/guard_cloud',sensor_qos(2))
        self.create_subscription(PointCloud2,NS+'/lio/cloud_registered',self.on_registered,sensor_qos(5))
        self.create_subscription(ChassisAdvice,NS+'/corridor/advice',self.on_advice,sensor_qos(100))
        self.create_subscription(OccupancyGrid,NS+'/corridor/model',self.on_model,latched_qos(1))
        self.create_subscription(String,NS+'/corridor/status',self.on_model_status,latched_qos(1))
        self.tf_pub = self.create_publisher(TFMessage,NS+'/tf_static',latched_qos())
        self.advice = [];self.models = [];self.model_reports = []
        self.registered_count = self.relay_count = 0

    def on_model(self,msg):self.models.append(msg)

    def on_model_status(self,msg):self.model_reports.append(json.loads(msg.data))

    def elapsed(self):
        return -1. if self.motion_start is None else self.get_clock().now().nanoseconds*1e-9-self.motion_start

    def on_registered(self,msg):
        self.registered_count += 1
        # 只中断保护输入；真实LIO、底盘反馈和目标继续，专门验证保护过期归零。
        if not 37. <= self.elapsed() < 40.:
            self.guard_pub.publish(msg);self.relay_count += 1

    def on_estimate(self,msg):
        super().on_estimate(msg)
        base = copy.deepcopy(msg);base.header.frame_id = 'map';base.child_frame_id = 'base_link'
        self.base_pub.publish(base)

    def on_advice(self,msg):
        self.advice.append([seconds(msg.header.stamp),self.elapsed(),msg.cmd_vx,msg.cmd_vy,
                            int(msg.valid),int(msg.blocked)])

    def start(self):
        self.launch_ros('corridor_model','team43_grid_mapper','corridor_map_node',
            {'floor_z':-.7,'width_m':2.4937575280215114,'run_dir':str(self.output/'model')},
            {'/red_standard_robot1/cloud_registered':NS+'/lio/cloud_registered',
             '/team43/slam_map':NS+'/corridor/model','/team43/corridor/status':NS+'/corridor/status'})
        self.launch_ros('corridor_advisor','team43_chassis_advisor','chassis_advisor_node',
            {'chassis.dir_mode':'direct','chassis.enable_avoidance':False,
             'chassis.obstacle_guard_enabled':True,'chassis.obstacle_floor_valid':True,
             'chassis.obstacle_floor_z':-.7},
            {'/red_standard_robot1/cloud_registered':NS+'/corridor/guard_cloud',
             '/gimbal/base_pose':NS+'/corridor/base','/estimation/target_state':NS+'/corridor/target',
             '/chassis/advice':NS+'/corridor/advice'})
        # 实验map与LIO初始系相同，测试不加载真值位姿，也不测试裁判起点/视觉链。
        tf = TransformStamped();tf.header.frame_id = 'map';tf.child_frame_id = 'lio_world'
        tf.transform.rotation.w = 1.;self.tf_pub.publish(TFMessage(transforms=[tf]))
        super().start()

    def drive(self,now):
        elapsed = -1. if self.motion_start is None else now-self.motion_start
        cmd = Twist()
        if 3 <= elapsed < 8:cmd.linear.y = .14*envelope(elapsed-3,5.)
        elif 12 <= elapsed < 17:cmd.linear.y = -.14*envelope(elapsed-12,5.)
        elif 17 <= elapsed < 33:cmd.linear.x = .22*envelope(elapsed-17,16.)
        self.cmd_pub.publish(cmd)  # 独立规定运动，模拟主办方控制；不执行建议速度。
        target = TargetState();target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map';target.valid = True;target.track_status = 1
        if 3 <= elapsed < 17:target.cur_x,target.cur_y = 0.,4.
        else:target.cur_x,target.cur_y = 6.,0.
        target.pred_x,target.pred_y = target.cur_x,target.cur_y
        target.last_obs_age = 0.;self.target_pub.publish(target)

    def run(self):
        self.start();start = time.monotonic();last = -1
        while time.monotonic()-start < self.timeout:
            self.pump();now = self.get_clock().now().nanoseconds*1e-9
            if now-self.last_command >= .02:
                self.drive(now);self.last_command = now
            if self.motion_start is None and len(self.estimates)>=20 and len(self.truth)>=100:
                self.motion_start = now+2.
            if int(now//5)!=last:
                last = int(now//5)
                print(f'sim={now:.1f}s elapsed={self.elapsed():.1f} LIO={len(self.estimates)} advice={len(self.advice)} model={len(self.models)}',flush=True)
            if self.elapsed() >= 44.:break
            if now>15 and not self.estimates:raise RuntimeError('15秒仿真时间内LIO无输出')
        else:raise RuntimeError('走廊仿真超时')
        self.cmd_pub.publish(Twist())
        return self.evaluate()

    def evaluate(self):
        a = np.asarray(self.advice);gt = np.asarray(self.truth);est = np.asarray(self.estimates)
        if not len(a):raise RuntimeError('建议速度无输出')
        gt_at = np.c_[np.interp(a[:,0],gt[:,0],gt[:,1]),np.interp(a[:,0],gt[:,0],gt[:,2])]
        # 以Gazebo真实墙面评价所有有效建议的1.5秒扫掠，而不是再用被测点云自证。
        path = gt_at[:,None,:]+np.linspace(0,1.5,51)[None,:,None]*a[:,None,2:4]
        clearance = np.minimum(4.-np.abs(path[:,:,0]),1.25-np.abs(path[:,:,1])).min(axis=1)
        valid = a[:,4]>0
        side = (a[:,1]>9.)&(a[:,1]<11.5)&valid
        end = (a[:,1]>34.)&(a[:,1]<36.5)&valid
        dropout = (a[:,1]>38.)&(a[:,1]<39.8)
        recovery = (a[:,1]>41.)&(a[:,1]<43.5)&valid
        straight = (a[:,1]>.5)&(a[:,1]<2.5)&valid
        model = self.output/'model/corridor_model.json'
        fitted = json.loads(model.read_text()) if model.exists() else {}
        match = np.c_[np.interp(est[:,0],gt[:,0],gt[:,1]),np.interp(est[:,0],gt[:,0],gt[:,2])]
        error = np.linalg.norm(est[:,1:3]-match,axis=1)
        checks = {
            'real_gazebo_and_lio':self.raw_scans>100 and len(est)>100,
            'model_generated':bool(self.models) and bool(fitted),
            'model_width_matches_independent_geometry':abs(fitted.get('width_m',0.)-2.5)<.10,
            'model_has_no_fake_end_walls':bool(self.models) and np.count_nonzero(np.asarray(self.models[-1].data).reshape(self.models[-1].info.height,-1)[:,0]==100)==2,
            'actual_motion_approached_side_wall':float(gt[:,2].max())>.55,
            'actual_motion_approached_end_wall':float(gt[:,1].max())>3.2,
            'lio_position_max':float(error.max())<.20,
            'clear_forward_suggestion':np.count_nonzero(straight)>=5 and bool(np.any(a[straight,2]>.02)),
            'side_wall_suggestions_present':np.count_nonzero(side)>=5,
            'end_wall_suggestions_present':np.count_nonzero(end)>=5,
            # 允许5cm定位/栅格化评估误差，生产保护半径仍为40cm，没有调小。
            'all_valid_sweeps_keep_clearance':bool(np.any(valid)) and float(clearance[valid].min())>=.35,
            'side_wall_changes_raw_target_direction':bool(np.any(side)) and bool(np.all(a[side,3]*1.5 <= 1.25-gt_at[side,1]-.35)),
            'end_wall_limits_forward_motion':bool(np.any(end)) and bool(np.all(a[end,2]*1.5 <= 4.-gt_at[end,0]-.35)),
            'cloud_dropout_stops_suggestions':np.count_nonzero(dropout)>=5 and bool(np.all(np.linalg.norm(a[dropout,2:4],axis=1)<1e-8)),
            'cloud_recovery_resumes_suggestions':np.count_nonzero(recovery)>=5 and bool(np.any(np.linalg.norm(a[recovery,2:4],axis=1)>.02)),
        }
        for name,rows in (('advice.csv',np.c_[a,gt_at,clearance]),('truth.csv',gt),('lio.csv',est)):
            with (self.output/name).open('w') as f:csv.writer(f).writerows(rows.tolist())
        self.preview_corridor(gt,a,gt_at)
        return {'passed':all(checks.values()),'checks':checks,'model':fitted,'metrics':{
            'advice_messages':len(a),'valid_advice_messages':int(valid.sum()),
            'minimum_true_wall_clearance_m':float(clearance[valid].min()) if np.any(valid) else None,
            'lio_position_max_error_m':float(error.max()),'lio_position_rmse_m':float(np.sqrt(np.mean(error**2))),
            'raw_scans':self.raw_scans,'lio_poses':len(est),'registered_clouds':self.registered_count,
            'protected_clouds_forwarded':self.relay_count,'side_samples':int(side.sum()),'end_samples':int(end.sum()),
            'dropout_samples':int(dropout.sum()),'recovery_samples':int(recovery.sum())}}

    def preview_corridor(self,gt,a,xy):
        image = Image.new('RGB',(1100,460),'white');draw = ImageDraw.Draw(image)
        def px(x,y):return (int(550+x*115),int(240-y*115))
        draw.rectangle([px(-4,1.25),px(4,-1.25)],outline='black',width=4)
        draw.line([px(*p) for p in gt[:,1:3]],fill='green',width=3)
        for i in range(0,len(a),max(1,len(a)//100)):
            if a[i,4]:draw.line([px(*xy[i]),px(*(xy[i]+a[i,2:4]*1.5))],fill='red',width=1)
        draw.text((20,15),'Gazebo corridor 8 x 2.5m | black: actual walls | green: prescribed motion | red: 1.5s advice',fill='black')
        draw.text((20,35),'Raw Gazebo LiDAR + IMU -> real Small Point-LIO -> production corridor model / obstacle guard',fill='black')
        image.save(self.output/'overview.png')

    def close(self):
        if rclpy.ok():self.cmd_pub.publish(Twist())
        codes = stop_processes(self.processes,self.output)
        for log in self.logs:log.close()
        self.destroy_node()
        return codes


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',required=True,type=Path)
    p.add_argument('--timeout',type=float,default=900.)
    p.add_argument('--real-time-factor',type=float,default=.25)
    args = p.parse_args()
    if (not Path('/.dockerenv').exists() or ROOT!=Path('/workspace') or
            os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID') in (None,'0')
            or not os.environ.get('IGN_PARTITION')):p.error('仅限/workspace隔离Docker、独立ROS域与IGN_PARTITION')
    if not 0<args.real_time_factor<=1 or not 30<=args.timeout<=3600:p.error('仿真倍率或超时无效')
    from ament_index_python.packages import get_package_share_directory
    os.environ['RM_LIO_WORLD'] = str(Path(get_package_share_directory('team43_gazebo'))/'worlds/corridor.sdf')
    output = args.output_dir.resolve();output.mkdir(parents=True,exist_ok=False)
    rclpy.init();app = CorridorTrial(output,args.timeout,args.real_time_factor)
    report = {'kind':'gazebo_corridor_model_and_realtime_obstacle_guard','hardware_accessed':False,
              'ground_truth_used_as_odometry_input':False,'point_lio_image_build':point_lio_image_build(),
              'world_source':str(app.world_source),'world_source_sha256':app.world_source_sha256,
              'generated_world_sha256':hashlib.sha256((output/'world.sdf').read_bytes()).hexdigest(),
              'limitations':['规则16线雷达，不是MID360扫描模式','悬浮平台，不模拟轮胎或实际制动',
                 '目标为合成有效TargetState；没有运行ArUco视觉或评分CSV',
                 'map在本试验与LIO初始系重合；不验收裁判replay_start链',
                 '轨迹独立规定，建议不控制平台；检验建议扫掠路径而非闭环避障',
                 '无额外注入噪声，2.5m走廊几何为测试假设，不是实物测绘']}
    try:report.update(app.run())
    except (Exception,KeyboardInterrupt) as exc:
        report.update(passed=False,error=repr(exc))
        (output/'failure.log').write_text(traceback.format_exc())
    finally:
        report['node_exit_codes'] = app.close()
        if rclpy.ok():rclpy.shutdown()
    report['clean_shutdown'] = all(code==0 for code in report['node_exit_codes'].values())
    report['passed'] = report['passed'] and report['clean_shutdown']
    report['Gazebo_executed'] = app.raw_scans>0 and len(app.truth)>0
    report['Point_LIO_executed'] = len(app.estimates)>0
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
