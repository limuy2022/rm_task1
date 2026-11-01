"""上午有效地图交接；保存失败不能改变下午当前地图。"""
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest
from team43_grid_mapper.grid_builder import GridMap
from team43_grid_mapper.pgm_io import write_map
from team43_grid_mapper.saved_map import SavedMap

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('survey_archive',ROOT/'tools/save_mapping.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_archive_preserves_morning_map_and_atomically_repoints_current(tmp_path):
    grid=np.zeros((30,40),np.int8);grid[:2,:]=100;grid[-3:,-4:]=-1
    gm=GridMap.from_arrays(grid,.05,[-1,-2],inflation_radius_m=0.)
    source=tmp_path/'source'
    write_map(str(source),gm,'43_slam_map.pgm','43_slam_map.yaml')
    dest=tmp_path/'maps';first=module.archive(source,dest)
    assert (dest/'current').resolve()==first
    assert np.array_equal(SavedMap(first/'43_slam_map.yaml').grid,grid)
    assert len(json.loads((first/'manifest.json').read_text())['sha256'])==2
    # 上午原始records被移走仍能加载快照；重复保存保留第一份快照。
    second=module.archive(source,dest)
    assert first!=second and first.is_dir() and (dest/'current').resolve()==second
    (source/'43_slam_map.pgm').unlink()
    assert SavedMap(dest/'current/43_slam_map.yaml').grid.shape==(30,40)
    with pytest.raises(OSError):module.archive(source,dest)
    assert (dest/'current').resolve()==second


def test_survey_launch_contains_no_motion_or_judge_process(tmp_path):
    from launch import LaunchContext
    from launch_ros.actions import Node
    from team43_bringup.livox_config import main
    spec=importlib.util.spec_from_file_location('survey_launch',ROOT/'src/team43_bringup/launch/survey.launch.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    driver=tmp_path/'livox.json';main(['--output',str(driver)])
    ctx=LaunchContext();ctx.launch_configurations.update(livox_config=str(driver),
        output_dir=str(tmp_path/'source'),lio_config=str(ROOT/'src/team43_lio_backend/config/small_point_lio_mid360.yaml'))
    actions=mod._setup(ctx)
    names={a.node_executable for a in actions if isinstance(a,Node)}
    assert names=={'livox_ros_driver2_node','small_point_lio_node','cloud_mapping_node'}
