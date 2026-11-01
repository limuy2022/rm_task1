"""完整录制建图原始ROS消息；配置快照、分片、正常收尾与独立SQLite计数核验。"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import hashlib
import sqlite3
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
LIDAR='/red_standard_robot1/livox/lidar'
IMU='/red_standard_robot1/livox/imu'


def inspect_bag(path, required=(LIDAR,IMU)):
    import rosbag2_py
    path=Path(path).resolve()
    meta=rosbag2_py.Info().read_metadata(str(path),'sqlite3')
    declared={row.topic_metadata.name:int(row.message_count) for row in meta.topics_with_message_count}
    actual={}
    for name in meta.relative_file_paths:
        db=(path/name).resolve()
        if db.parent!=path or not db.is_file():raise ValueError('录制分片缺失或路径无效：'+str(db))
        with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as conn:
            for topic,count in conn.execute('SELECT topics.name, COUNT(messages.id) FROM topics '
                    'LEFT JOIN messages ON messages.topic_id=topics.id GROUP BY topics.name'):
                actual[topic]=actual.get(topic,0)+int(count)
    if actual!=declared:raise ValueError('SQLite实际消息数与metadata不一致')
    missing=[name for name in required if actual.get(name,0)==0]
    if missing:raise ValueError('没有录到必需原始输入：'+', '.join(missing))
    return {'counts':actual,'files':list(meta.relative_file_paths),
            'duration_s':meta.duration.nanoseconds*1e-9,
            'message_count':int(meta.message_count)}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=0.,help='墙钟录制时长；0为持续录制，Ctrl+C正常结束')
    parser.add_argument('--output-dir',type=Path,help='新的记录目录；默认records/raw_日期_时间')
    parser.add_argument('--with-camera',action='store_true',help='额外录制原图、相机信息、云台IMU和epoch；须已有对应驱动，数据量较大')
    parser.add_argument('--split-seconds',type=int,default=60,choices=range(1,3601),metavar='1..3600',help='分片时长，默认60秒；同时按1GiB分片')
    parser.add_argument('--lio-config',type=Path,help='实际使用的自定义LIO YAML；不填则备份安装默认文件')
    args=parser.parse_args(argv)
    if not 0<=args.seconds<=86400:parser.error('seconds须为0..86400的有限数')
    from ament_index_python.packages import get_package_share_directory
    import yaml
    output=(args.output_dir or ROOT/'records'/('raw_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).expanduser().resolve()
    if output.exists():parser.error('输出目录已存在，请换一个新目录，避免覆盖录制')
    output.mkdir(parents=True,exist_ok=False)
    topics=[LIDAR,IMU]
    if args.with_camera:topics+=['/image_raw','/camera_info','/red_standard_robot1/gimbal','/task1/gimbal_reset_epoch']
    configs=output/'config';configs.mkdir()
    lio=args.lio_config or Path(get_package_share_directory('team43_lio_backend'))/'config/small_point_lio_mid360.yaml'
    sources={'lio.yaml':lio,'vehicle.yaml':Path(get_package_share_directory('team43_bringup'))/'config/vehicle_field.yaml',
             'camera.yaml':Path(get_package_share_directory('team43_bringup'))/'config/camera_team43.yaml',
             'cloud_mapping.yaml':Path(get_package_share_directory('team43_grid_mapper'))/'config/cloud_mapping.yaml',
             'livox.json':ROOT/'config/livox_mid360.json'}
    copied={}
    for name,source in sources.items():
        if source.is_file():shutil.copy2(source,configs/name);copied[name]=str(source)
        elif name=='lio.yaml':raise ValueError('实际LIO配置不存在：'+str(source))
    # 明确与best_effort和reliable传感器都兼容；不改消息、不改header或采样时间。
    qos={topic:{'reliability':'best_effort','durability':'volatile','history':'keep_last','depth':1000} for topic in topics}
    if args.with_camera:
        qos['/task1/gimbal_reset_epoch'].update(reliability='reliable',durability='transient_local',depth=1)
    (output/'record_qos.yaml').write_text(yaml.safe_dump(qos))
    command=['ros2','bag','record','-s','sqlite3','-o',str(output/'bag'),
             '--max-bag-size','1073741824','--max-bag-duration',str(args.split_seconds),
             '--max-cache-size','67108864','--qos-profile-overrides-path',str(output/'record_qos.yaml'),*topics]
    manifest={'success':False,'topics_requested':topics,'copied_config_sources':copied,
              'config_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in configs.iterdir()},
              'config_note':'LIO配置为所指定或安装默认文件；自定义启动覆盖必须同时保留，不冒充运行期参数实测。',
              'started':datetime.now().isoformat(),'command':command,
              'limitations':['完整保存已收到的消息；不能仅凭数量证明现场没有丢包或未扫区域完整覆盖']}
    stop=threading.Event();old={sig:signal.signal(sig,lambda *_:stop.set()) for sig in
        (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    process=None;reader=None;log=None;failure=None
    begin=time.monotonic()
    try:
        print('只读录制，不启动驱动或运动；请保持现有雷达程序运行。',flush=True)
        print('目录：'+str(output)+'\n原始话题：'+', '.join(topics),flush=True)
        print('Ctrl+C结束后等待校验完成，再复制整个目录；不要只复制单个db3。',flush=True)
        free=shutil.disk_usage(output).free
        print('可用空间：%.2f GiB'%(free/1024**3),flush=True)
        if free<2*1024**3:print('注意：可用空间不足2GiB，请关注剩余空间。',flush=True)
        log=(output/'recorder.log').open('w')
        process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL,text=True,start_new_session=True,bufsize=1)
        def relay():
            for line in process.stdout:
                log.write(line);log.flush();print(line,end='',flush=True)
        reader=threading.Thread(target=relay,daemon=True);reader.start()
        while process.poll() is None and not stop.is_set():
            if args.seconds and time.monotonic()-begin>=args.seconds:stop.set();break
            if shutil.disk_usage(output).free<256*1024**2:
                failure='剩余空间低于256MiB，停止录制以完成收尾';stop.set();break
            stop.wait(.2)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                failure='录制器30秒内未正常关闭；保留原始文件，不保证完整'
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        reader.join(timeout=3)
        log_text=(output/'recorder.log').read_text()
        manifest['recorder_reported_loss']=any(text in log_text for text in
            ('Cache buffers lost messages per topic:', 'Cache buffers were unflushed with'))
        if manifest['recorder_reported_loss']:failure=failure or '录制器报告缓存丢失或未完全写出，查看recorder.log'
        manifest['recorder_exit_code']=process.returncode
        if not stop.is_set():failure=failure or '录制器意外提前退出'
        if process.returncode not in (0,-signal.SIGINT,130):failure=failure or '录制器异常退出'
        manifest['bag']=inspect_bag(output/'bag')
        optional_missing=[t for t in topics[2:] if manifest['bag']['counts'].get(t,0)==0]
        manifest['optional_topics_not_received']=optional_missing
        manifest['all_requested_topics_received']=not optional_missing
        if optional_missing:print('额外请求但未收到（可能对应驱动未运行）：'+', '.join(optional_missing),flush=True)
        if failure:raise RuntimeError(failure)
        manifest['success']=True
        print('原始数据保存并校验完成：'+json.dumps(manifest['bag'],ensure_ascii=False),flush=True)
    except Exception as exc:
        manifest['error']=str(exc);print('录制未通过完整性检查：'+str(exc),file=sys.stderr,flush=True)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGKILL);process.wait()
        if reader is not None:reader.join(timeout=3)
        if log is not None:log.close()
        for sig,handler in old.items():signal.signal(sig,handler)
        manifest['elapsed_wall_s']=time.monotonic()-begin;manifest['finished']=datetime.now().isoformat()
        (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return 0 if manifest['success'] else 1


if __name__=='__main__':raise SystemExit(main())
