"""直接在真 judge_io_node 方法与真 ROS 消息上执行的离线回归。

节点构造器用 ``__new__`` 绕过，内部字段手工注入；执行的是生产代码。
不验证 DDS/ROS 通信。
"""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
import sys
root = Path(__file__).resolve().parents[3]
for package in ('team43_common', 'team43_judge_io', 'team43_grid_mapper'):
    sys.path.insert(0, str(root/'src'/package))

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid

from team43_judge_io.judge_io_node import JudgeIoNode

def test_map_origin_yaw_preserved(tmp_path):
    log = NS(error=lambda *a: None, warning=lambda *a: None)
    obj = JudgeIoNode.__new__(JudgeIoNode)
    obj._log = log
    q = Quaternion(w=math.cos(.4), x=0., y=0., z=math.sin(.4))
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = 2; msg.info.height = 1; msg.info.resolution = .5
    msg.info.origin.position.x = -1.; msg.info.origin.position.y = 2.
    msg.info.origin.orientation = q
    msg.data = [0, 100]
    JudgeIoNode._on_grid(obj, msg)
    assert abs(obj._grid_yaw - .8) < 1e-12
    assert obj._grid.tolist() == [[0],[100]]
    obj._map_image_name='43_map.pgm';obj._params={'judge.map_occupied_thresh':.65,'judge.map_free_thresh':.196}
    obj._map_saved_final=False;obj._n_tick=1;obj._map_pgm='';obj._log=NS(error=lambda *a:None,info=lambda *a:None)
    p=tmp_path/'map.yaml'
    obj._map_pgm=str(tmp_path/'map.pgm');obj._map_yaml=str(p)
    assert JudgeIoNode._save_map(obj,'final')
    np.testing.assert_allclose(yaml.safe_load(p.read_text())['origin'],[-1,2,.8])
