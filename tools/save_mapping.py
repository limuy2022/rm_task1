"""请求有效源图落盘，保存独立快照并原子更新下午默认地图链接。"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from team43_grid_mapper.saved_map import SavedMap


def archive(source, destination):
    source=Path(source).resolve();destination=Path(destination).expanduser().resolve()
    destination.mkdir(parents=True,exist_ok=True)
    # 先复制到私有目录再验证，避免源节点自动保存时跨文件读取。
    with tempfile.TemporaryDirectory(prefix='.survey-',dir=destination) as tmp:
        snapshot=Path(tmp)
        for name in ('43_slam_map.pgm','43_slam_map.yaml'):shutil.copy2(source/name,snapshot/name)
        SavedMap(snapshot/'43_slam_map.yaml')
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()}
        (snapshot/'manifest.json').write_text(json.dumps({'created':datetime.now().isoformat(),
            'source':str(source),'sha256':hashes,'frame':'survey_world：上午源图独立会话'},ensure_ascii=False,indent=2)+'\n')
        target=destination/('survey_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copytree(snapshot,target)
    link=destination/'current';pending=destination/('.current_'+target.name)
    try:
        pending.symlink_to(target.name,target_is_directory=True);os.replace(pending,link)
    finally:
        if pending.is_symlink():pending.unlink()
    return target


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--maps-dir',required=True)
    args=parser.parse_args();rclpy.init();node=Node('save_survey_map')
    try:
        client=node.create_client(Trigger,'/team43/save_slam_map')
        if not client.wait_for_service(timeout_sec=8):raise RuntimeError('上午建图尚未运行，没有保存服务')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None:raise RuntimeError('地图保存超时')
        response=future.result()
        if not response.success:raise RuntimeError('地图保存失败：'+response.message)
        target=archive(response.message,args.maps_dir)
        print('上午地图已保存并验证：'+str(target),flush=True)
        print('下午默认地图：'+str(Path(args.maps_dir)/'current/43_slam_map.yaml'),flush=True)
        print('现在可以在上午主终端按Ctrl+C；保留整个maps目录并备份。',flush=True)
        return 0
    except (RuntimeError,OSError,ValueError) as exc:
        print('地图交接未完成：'+str(exc),flush=True);return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
