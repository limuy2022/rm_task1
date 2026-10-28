"""通用仿真启动助手必须启动调用方要求的节点，不能固定为定位程序。"""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


@pytest.mark.parametrize('package,executable', [
    ('team43_grid_mapper', 'cloud_mapping_node'),
    ('team43_perception', 'marker_pnp_node'),
])
def test_launch_ros_preserves_requested_node(tmp_path, package, executable):
    source = Path(__file__).resolve().parents[3] / 'tools/simulate_gazebo_lio.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GazeboLio')
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'launch_ros')
    scope = {'yaml': yaml, 'NS': '/sim/gazebo'}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
    launched = []
    owner = SimpleNamespace(output=tmp_path, launch=lambda name, command: launched.append(command))
    scope['launch_ros'](owner, 'example', package, executable, {}, {})
    assert launched[0][:4] == ['ros2', 'run', package, executable]
