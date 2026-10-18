"""执行选定节点方法的离线回归；替代消息仅用于逻辑，不验证DDS/ROS。"""
import ast
import math
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import sys
for package in ('gimbal_common', 'grid_mapper'):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'/package))

root = Path(__file__).resolve().parents[1] / 'src'

def method(path, name, env):
    tree = ast.parse(path.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[fn], type_ignores=[])
    exec(compile(module, str(path), 'exec'), env)
    return env[name]

log = NS(error=lambda *a: None, warning=lambda *a: None)
judge = root / 'gimbal_judge_io/gimbal_judge_io/judge_io_node.py'
from gimbal_common import frames
env = {'Any': object, 'np': np, 'math': math, 'frames': frames}
on_grid = method(judge, '_on_grid', env)
obj = NS(_log=log)
q = NS(w=math.cos(.4), x=0., y=0., z=math.sin(.4))
msg = NS(header=NS(frame_id='map'), info=NS(width=2, height=1, resolution=.5,
         origin=NS(position=NS(x=-1., y=2.), orientation=q)), data=[0,100])
on_grid(obj, msg)
assert abs(obj._grid_yaw - .8) < 1e-12
assert obj._grid.tolist() == [[0],[100]]
obj._map_image_name='47_map.pgm';obj._params={'judge.map_occupied_thresh':.65,'judge.map_free_thresh':.196}
obj._map_saved_final=False;obj._n_tick=1;obj._map_pgm='';obj._log=NS(error=lambda *a:None,info=lambda *a:None)
save_map = method(judge, '_save_map', dict(env, time=__import__('time')))
with tempfile.TemporaryDirectory() as d:
    p=Path(d)/'map.yaml'
    obj._map_pgm=str(Path(d)/'map.pgm');obj._map_yaml=str(p)
    assert save_map(obj,'final')
    import yaml
    np.testing.assert_allclose(yaml.safe_load(p.read_text())['origin'],[-1,2,.8])
print('PASS map origin yaw preserved in ingestion and saved YAML')

from verify_timing_offline import TimingTests
case=TimingTests('test_no_timeout_origin_fallback_or_new_round_rebase')
case.test_no_timeout_origin_fallback_or_new_round_rebase()
print('PASS replay origin has no timeout/first-tick fallback')
for p in root.glob('**/*.py'):
    ast.parse(p.read_bytes(),filename=str(p))
print('PASS syntax; NOT VERIFIED: ROS communication, live timestamps or calibration')
