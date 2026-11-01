"""请求有效源图落盘，保存独立采集快照；不改交付地图。"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml
import numpy as np
from team43_grid_mapper.pgm_io import read_pgm,pgm_to_grid


def archive(source, destination):
    source=Path(source).resolve();destination=Path(destination).expanduser().resolve()
    destination.mkdir(parents=True,exist_ok=True)
    # 先复制到私有目录再验证，避免源节点自动保存时跨文件读取。
    with tempfile.TemporaryDirectory(prefix='.survey-',dir=destination) as tmp:
        snapshot=Path(tmp)
        for name in ('43_slam_map.pgm','43_slam_map.yaml'):shutil.copy2(source/name,snapshot/name)
        meta=yaml.safe_load((snapshot/'43_slam_map.yaml').read_text())
        image,maximum=read_pgm(str(snapshot/meta['image']))
        grid=pgm_to_grid(image,meta['occupied_thresh'],meta['free_thresh'],meta['negate'],maximum)
        if np.count_nonzero(grid==100)<20 or np.count_nonzero(grid==0)<20:
            raise ValueError('源图缺少有效占用或空闲观测')
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()}
        (snapshot/'manifest.json').write_text(json.dumps({'created':datetime.now().isoformat(),
            'source':str(source),'sha256':hashes,'source_frame':'lio_world'},ensure_ascii=False,indent=2)+'\n')
        target=destination/('survey_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copytree(snapshot,target)
    return target


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--archive-dir',required=True)
    args=parser.parse_args();rclpy.init();node=Node('save_survey_map')
    try:
        client=node.create_client(Trigger,'/team43/save_slam_map')
        if not client.wait_for_service(timeout_sec=8):raise RuntimeError('上午建图尚未运行，没有保存服务')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None:raise RuntimeError('地图保存超时')
        response=future.result()
        if not response.success:raise RuntimeError('地图保存失败：'+response.message)
        target=archive(response.message,args.archive_dir)
        print('上午地图已保存并验证：'+str(target),flush=True)
        print('这是独立采集快照，不自动替换maps中的交付地图；可停止上午建图并备份。',flush=True)
        return 0
    except (RuntimeError,OSError,ValueError) as exc:
        print('地图交接未完成：'+str(exc),flush=True);return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
