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


@pytest.mark.parametrize('lidar_format', ['pointcloud2', 'custom'])
def test_gazebo_livox_adapter_preserves_snapshot_coordinates_and_times(lidar_format):
    """四次独立快照的坐标与时间不能在格式适配时被插值、重置或重复加起点。"""
    import numpy as np
    from sensor_msgs.msg import PointCloud2, PointField
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    from team43_grid_mapper.cloud_grid import pointcloud2_xyz

    source = Path(__file__).resolve().parents[3] / 'tools/simulate_gazebo_lio.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GazeboLio')
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_cloud')
    scope = dict(np=np, PointCloud2=PointCloud2, PointField=PointField, CustomMsg=CustomMsg,
                 CustomPoint=CustomPoint, pointcloud2_xyz=pointcloud2_xyz,
                 stamp_ns=lambda stamp: stamp.sec*1_000_000_000+stamp.nanosec)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), scope)
    published = []
    owner = SimpleNamespace(last_raw_stamp=-1, out_of_order={'raw_cloud': 0}, raw_scans=0,
                            raw_bad=0, measurement_samples=[], cloud_batches=[], batch_stamps=[],
                            lidar_format=lidar_format, scan_lines=16,
                            lidar_pub=SimpleNamespace(publish=published.append), published_lidar_stamps=[],
                            offset_spans=[], published_frames=0, published_points=0)
    coordinates = []
    for i in range(4):
        raw = PointCloud2(height=1, width=2, point_step=12, row_step=24)
        raw.header.stamp.sec = 100
        raw.header.stamp.nanosec = i*25_000_000
        raw.fields = [PointField(name=name, offset=j*4, datatype=7, count=1)
                      for j, name in enumerate(('x', 'y', 'z'))]
        xyz = np.array([[i+1., .2, .3], [i+1., -.4, .5]], dtype='<f4')
        raw.data = xyz.tobytes()
        coordinates.extend(xyz)
        scope['on_cloud'](owner, raw)
    assert len(published) == 1
    out = published[0]
    if lidar_format == 'pointcloud2':
        actual_xyz = pointcloud2_xyz(out)
        actual_ns = np.ndarray((8,), dtype='<f8', buffer=bytes(out.data), offset=18, strides=(26,))
    else:
        actual_xyz = np.array([[p.x, p.y, p.z] for p in out.points])
        actual_ns = np.array([out.timebase+p.offset_time for p in out.points])
    np.testing.assert_allclose(actual_xyz, coordinates, atol=0, rtol=0)
    np.testing.assert_array_equal(actual_ns, 100_000_000_000+np.repeat(np.arange(4)*25_000_000, 2))
    assert owner.offset_spans == [75_000_000]
    assert owner.published_frames == 1 and owner.published_points == 8
