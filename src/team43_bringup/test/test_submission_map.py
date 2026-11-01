"""选图复制保留原始几何，正式地图写出禁用后不能覆盖。"""
from pathlib import Path

import importlib.util
import pytest
import yaml

from team43_bringup.submission_map import copy_submission_map

ROOT = Path(__file__).resolve().parents[3]


def test_selected_map_copy_preserves_pixels_origin_and_source(tmp_path):
    source = ROOT/'maps/43_map.yaml'
    before = source.read_bytes()
    report = copy_submission_map(source,tmp_path)
    assert (tmp_path/'43_map.pgm').read_bytes() == (ROOT/'maps/43_map.pgm').read_bytes()
    meta = yaml.safe_load((tmp_path/'43_map.yaml').read_text())
    assert meta['image'] == '43_map.pgm'
    assert meta['origin'] == [-10,-10,0]
    assert meta['frame_id'] == 'lio_world'
    assert meta['resolution'] == .05
    assert report['coordinate_transform_applied'] is False
    assert source.read_bytes() == before
    assert {p.name for p in tmp_path.iterdir()} == {'43_map.pgm','43_map.yaml'}


def test_missing_map_preserves_previous_output(tmp_path):
    copy_submission_map(ROOT/'maps/43_map.yaml',tmp_path)
    before = {p.name:p.read_bytes() for p in tmp_path.iterdir()}
    with pytest.raises(FileNotFoundError):copy_submission_map(tmp_path/'missing.yaml',tmp_path)
    assert {p.name:p.read_bytes() for p in tmp_path.iterdir()} == before


def test_disabled_judge_map_writer_cannot_replace_copied_files(tmp_path):
    from nav_msgs.msg import OccupancyGrid
    spec = importlib.util.spec_from_file_location('copy_timing_harness',ROOT/'tools/verify_timing_offline.py')
    TIMING = importlib.util.module_from_spec(spec);spec.loader.exec_module(TIMING)
    app = TIMING.judge_node(str(tmp_path))
    copy_submission_map(ROOT/'maps/43_map.yaml',tmp_path)
    before = {p.name:p.read_bytes() for p in tmp_path.iterdir() if p.suffix in ('.pgm','.yaml')}
    app._params['judge.map_write_enabled'] = False
    msg = OccupancyGrid();msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 2;msg.info.resolution = .05
    msg.info.origin.orientation.w = 1.;msg.data = [100]*4
    app._on_grid(msg)
    assert not app._save_map('test')
    assert {p.name:p.read_bytes() for p in tmp_path.iterdir() if p.suffix in ('.pgm','.yaml')} == before
    app._writer.close()
