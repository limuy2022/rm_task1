#!/usr/bin/env python3
"""在容器内录下隔离域合成原始输入，关闭录制后在新ROS域重放并重建地图。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Trigger
from team43_common.qos_utils import sensor_qos,latched_qos
from team43_grid_mapper.saved_map import SavedMap

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--source-domain',type=int,default=89,choices=range(1,101))
    parser.add_argument('--record-project',type=Path,help='可选：测试临时部署目录中的真实record.bash入口')
    args=parser.parse_args()
    if not Path('/.dockerenv').exists() or ROOT!=Path('/workspace'):parser.error('仅限/workspace容器')
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=False)
    report={'passed':False,'hardware_accessed':False};processes=[];logs=[];node=None
    try:
        env=dict(os.environ,ROS_DOMAIN_ID=str(args.source_domain),ROS_LOCALHOST_ONLY='1')
        record_command=(['bash',str(args.record_project/'tools/record.bash')] if args.record_project
                        else ['python3','-B',str(ROOT/'tools/record.py')])
        with (output/'record.log').open('w') as log:
            subprocess.run([*record_command,'--seconds','10',
                            '--output-dir',str(output/'raw')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        os.environ.update(ROS_DOMAIN_ID='91',ROS_LOCALHOST_ONLY='1')
        env=dict(os.environ)
        def launch(name,command):
            log=(output/(name+'.log')).open('w');logs.append(log)
            proc=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(proc);return proc
        lio=output/'raw/config/lio.yaml'
        (output/'replay_mapper.yaml').write_text('/**:\n  ros__parameters:\n    use_sim_time: true\n')
        launch('lio',['ros2','launch','team43_lio_backend','lio.launch.py','use_sim_time:=true','params_file:='+str(lio)])
        launch('mapper',['ros2','launch','team43_bringup','cloud_mapping.launch.py','lio_params_file:='+str(lio),
            'params_file:='+str(output/'replay_mapper.yaml'),'output_dir:='+str(output/'rebuilt')])
        rclpy.init();node=rclpy.create_node('offline_rebuild_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
        poses=[];grids=[]
        node.create_subscription(Odometry,'/red_standard_robot1/aft_mapped_to_init',poses.append,sensor_qos(50))
        node.create_subscription(OccupancyGrid,'/team43/slam_map',grids.append,latched_qos(1))
        client=node.create_client(Trigger,'/team43/save_slam_map')
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('离线建图服务未就绪')
        play=launch('play',['ros2','bag','play',str(output/'raw/bag'),'--clock','50','--rate','.5','--delay','2'])
        end=time.monotonic()+45
        while play.poll() is None:
            if time.monotonic()>end:raise RuntimeError('离线回放超时')
            rclpy.spin_once(node,timeout_sec=.01)
        assert play.returncode==0
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert len(poses)>20 and grids,'回放没有产生足够的LIO位姿和地图'
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        assert future.done() and future.result().success
        saved=SavedMap(output/'rebuilt/43_slam_map.yaml')
        report.update(passed=True,lio_poses=len(poses),maps_received=len(grids),
                      occupied_cells=int((saved.grid==100).sum()),free_cells=int((saved.grid==0).sum()),
                      checks={'raw_record_complete':True,'new_lio_process_from_bag':True,'new_map_saved_from_raw_bag':True})
        if args.record_project:report['checks']['actual_record_bash_env_and_arguments']=True
    except Exception as exc:
        import traceback
        report.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        report['exit_codes']=[p.returncode for p in processes]
        report['passed'] &= all(p.returncode==0 for p in processes)
        for log in logs:log.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
