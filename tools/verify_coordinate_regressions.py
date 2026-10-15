"""执行选定节点方法的离线回归；替代消息仅用于逻辑，不验证DDS/ROS。"""
import ast
import math
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

root = Path(__file__).resolve().parents[1] / 'src'

def method(path, name, env):
    tree = ast.parse(path.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[fn], type_ignores=[])
    exec(compile(module, str(path), 'exec'), env)
    return env[name]

log = NS(error=lambda *a: None, warning=lambda *a: None)
judge = root / 'gimbal_judge_io/gimbal_judge_io/judge_io_node.py'
env = {'Any': object, 'np': np, 'math': math}
on_grid = method(judge, '_on_grid', env)
obj = NS(_log=log)
q = NS(w=math.cos(.4), x=0., y=0., z=math.sin(.4))
msg = NS(info=NS(width=2, height=1, resolution=.5,
         origin=NS(position=NS(x=-1., y=2.), orientation=q)), data=[0,100])
on_grid(obj, msg)
assert abs(obj._grid_yaw - .8) < 1e-12
assert obj._grid.tolist() == [[0],[100]]
obj._map_image_name='47_map.pgm';obj._params={'judge.map_occupied_thresh':.65,'judge.map_free_thresh':.196}
fallback = method(judge, '_write_yaml_fallback', env)
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'map.yaml';fallback(obj,str(p))
    assert 'origin: [-1, 2, 0.8]' in p.read_text()
print('PASS map origin yaw preserved in ingestion and fallback YAML')

path=root/'gimbal_localization/gimbal_localization/map_frame_broadcaster_node.py'
clock=NS(now=lambda:100.)
env={'Odometry':object,'math':math,'TU':clock,'quat_yaw':lambda *q:0.,
     'LU':NS(throttled=lambda *a:None)}
on_base=method(path,'_on_base_pose',env)
obj=NS(_log=log,_origin_mode='judge_replay_start',_origin_timeout_s=5.,
       _t_first_valid=0.,_n_pose=0,_frozen=False,_freeze_requested=False,
       _freeze=lambda *a: (_ for _ in ()).throw(AssertionError('premature freeze')))
msg=NS(pose=NS(pose=NS(position=NS(x=3.,y=4.),orientation=NS(x=0,y=0,z=0,w=1))),
       twist=NS(twist=NS(linear=NS(x=0.,y=0.))))
on_base(obj,msg)
assert obj._origin_mode=='judge_replay_start' and not obj._frozen
print('PASS wait beyond 5s does not freeze or change origin mode')
for p in root.glob('**/*.py'):
    ast.parse(p.read_bytes(),filename=str(p))
print('PASS syntax; NOT VERIFIED: ROS communication, live timestamps or calibration')
