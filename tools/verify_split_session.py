#!/usr/bin/env python3
"""实际上午入口保存→进程全停→换位换朝向→下午旧图重定位/跟踪/评分，容器与PTY专用。"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import tty
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from team43_common.qos_utils import sensor_qos, latched_qos
from team43_common import frames
from team43_grid_mapper.saved_map import rotation, angle
from team43_grid_mapper.saved_map import SavedMap
from verify_match_workflow import fixture_workspace, MatchProbe, ROOT


class MorningProbe(MatchProbe):
    def pump(self):
        if self.launch.poll() is not None:raise RuntimeError('上午入口提前退出')
        self.pump_sensors();rclpy.spin_once(self,timeout_sec=.002)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--count',type=int,default=30)
    parser.add_argument('--domain-id',type=int,default=89,choices=range(1,101))
    args=parser.parse_args()
    if not Path('/.dockerenv').exists() or ROOT!=Path('/workspace'):parser.error('仅限/workspace容器')
    if not 1<=args.count<=9000:parser.error('count须1..9000')
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(args.domain_id))
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=False)
    work=Path(tempfile.mkdtemp(prefix='split_match_'))
    report={'passed':False,'hardware_accessed':False,'checks':{}}
    launch=probe=log=None;master=slave=None
    try:
        project,_=fixture_workspace(work)
        master,slave=os.openpty();tty.setraw(slave);os.set_blocking(master,False)
        log=(output/'morning.log').open('w')
        launch=subprocess.Popen(['bash',str(project/'tools/start_mapping.bash')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();probe=MorningProbe(master,launch)
        probe.wait(lambda:any(json.loads(s.data).get('accepted_scans',0)>=30 for s in probe.map_status),30.)
        probe.run_command(['bash',str(project/'tools/save_mapping.bash')],output/'save.log')
        assert (project/'maps/current/43_slam_map.yaml').is_file()
        assert not probe.commands,'上午建图不应有串口运动指令'
        report['checks']['morning_actual_entry_and_saved_map_handoff']=True
        launch.send_signal(signal.SIGINT);launch.wait(timeout=12);assert launch.returncode==0
        log.close();probe.destroy_node();probe=None
        report['checks']['morning_can_stop_all_processes']=True
        log=(output/'afternoon.log').open('w')
        launch=subprocess.Popen(['bash',str(project/'tools/start_match.bash'),'port:='+os.ttyname(slave),
                                 'map_mode:=saved','map_yaml:='+str(project/'maps/current/43_slam_map.yaml'),
                                 'output_dir:='+str(work/'score')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        probe=MatchProbe(master,launch)
        # 新会话对应原图中平移[.55,-.35]、旋转.65rad的位置，传感器不在上午原位。
        import yaml
        from ament_index_python.packages import get_package_share_directory
        cfg=yaml.safe_load((Path(get_package_share_directory('team43_lio_backend'))/'config/small_point_lio_mid360.yaml').read_text())['/**']['ros__parameters']
        up=probe.up;v=np.cross(up,[0.,0.,1.]);K=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
        level=np.eye(3)+K+K@K/(1+up[2]);offset=np.asarray(cfg['extrinsic_T'])
        old=(probe.points+offset)@level.T
        new=old.copy();new[:,:2]=(old[:,:2]-[.55,-.35])@rotation(.65)
        probe.points=new@level-offset
        states=[];odom=[]
        probe.create_subscription(String,'/team43/relocalization/status',lambda msg:states.append(json.loads(msg.data)),latched_qos(1))
        probe.create_subscription(Odometry,'/red_standard_robot1/aft_mapped_to_init',odom.append,sensor_qos(5))
        hint_pub=probe.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        probe.wait(lambda:len(odom)>15 and hint_pub.get_subscription_count()>0,30.)
        # 方形房间存在多解，操作员在RViz提供近似底盘位置/车头方向；仍由点云验证。
        vehicle=yaml.safe_load((Path(get_package_share_directory('team43_bringup'))/'config/vehicle_field.yaml').read_text())['/**']['ros__parameters']
        msg=odom[-1];q=msg.pose.pose.orientation;p=msg.pose.pose.position
        R=frames.quat_to_mat([q.w,q.x,q.y,q.z])@frames.rpy_to_mat(vehicle['localization.sensor_in_base_rpy']).T
        base=np.array([p.x,p.y,p.z])-R@np.array(vehicle['localization.sensor_in_base_xyz'])
        base_old=rotation(.65)@base[:2]+[.55,-.35]
        yaw=.65+math.atan2(R[1,0],R[0,0])
        hint=PoseWithCovarianceStamped();hint.header.frame_id='survey_world';hint.header.stamp=probe.get_clock().now().to_msg()
        hint.pose.pose.position.x=float(base_old[0]+.12);hint.pose.pose.position.y=float(base_old[1]-.08)
        hint.pose.pose.orientation.z=math.sin((yaw+.05)/2);hint.pose.pose.orientation.w=math.cos((yaw+.05)/2)
        hint_pub.publish(hint)
        probe.wait(lambda:any(s.get('aligned') for s in states),45.)
        state=next(s for s in reversed(states) if s.get('aligned'))
        got=np.array(state['transform'])
        assert np.linalg.norm(got[:2]-[.55,-.35])<.18 and abs(angle(got[2]-.65))<.08,state
        report['checks']['different_pose_relocalized_against_saved_map']=True
        report['transform']=got.tolist();report['expected_transform']=[.55,-.35,.65]
        probe.run_command(['bash',str(project/'tools/set_target.bash'),'3'],output/'set_target.log')
        probe.wait(lambda:any(y<0<p for _,y,p in probe.commands),20.)
        probe.wait(lambda:probe.start_pub.get_subscription_count()>=3 and probe.tick_pub.get_subscription_count()>=1)
        msg=odom[-1];q=msg.pose.pose.orientation;p=msg.pose.pose.position
        R=frames.quat_to_mat([q.w,q.x,q.y,q.z])@frames.rpy_to_mat(vehicle['localization.sensor_in_base_rpy']).T
        base=np.array([p.x,p.y,p.z])-R@np.array(vehicle['localization.sensor_in_base_xyz'])
        base_yaw=math.atan2(R[1,0],R[0,0])
        probe.score_round(args.count,work/'score/43.csv')
        assert probe.grids and (work/'score/43_map.yaml').is_file()
        assert any(a.valid for a in probe.advice)
        # 必须检查尾部，不能让开赛前几秒的有效建议掩盖静态地图5秒后过期。
        assert sum(a.valid and math.hypot(a.cmd_vx,a.cmd_vy)>.02 for a in probe.advice[-100:])>=20, '静态旧图运行后不再产生有效建议'
        assert len(probe.rows_seen)==args.count and max(probe.visible_latencies)<.08
        source=SavedMap(project/'maps/current/43_slam_map.yaml')
        formal=SavedMap(work/'score/43_map.yaml')
        expected_origin=rotation(-base_yaw)@(rotation(-got[2])@(source.origin[:2]-got[:2])-base[:2])
        assert np.array_equal(formal.grid,source.grid),'正式图没有保留上午格子'
        assert np.linalg.norm(formal.origin[:2]-expected_origin)<.1,(formal.origin,expected_origin)
        assert abs(angle(formal.origin[2]-(source.origin[2]-got[2]-base_yaw)))<.04
        report['checks']['saved_cells_and_match_frame_origin_preserved']=True
        report['checks']['afternoon_tracking_ticks_old_map_export_and_advice']=True
        report.update(ticks=args.count,rows=len(probe.rows_seen),max_visible_latency_s=max(probe.visible_latencies),passed=True)
    except Exception as exc:
        import traceback
        report.update(error=repr(exc),traceback=traceback.format_exc())
        if probe is not None:
            report['observed_rows_before_failure']=len(probe.rows_seen)
            report['max_observed_latency_s']=max(probe.visible_latencies,default=0.)
    finally:
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:launch.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
        if launch is not None:report['last_launch_exit_code']=launch.returncode;report['passed'] &= launch.returncode==0
        if log is not None:log.close()
        if probe is not None:probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for name,source in [('maps',project/'maps'),('records',project/'records'),('score',work/'score')]:
            if source.exists():shutil.copytree(source,output/name,dirs_exist_ok=True)
        for fd in (master,slave):
            if fd is not None:os.close(fd)
        shutil.rmtree(work)
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
