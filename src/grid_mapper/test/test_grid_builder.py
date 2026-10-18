"""grid_builder / field_config / pgm_io 单测（不依赖 rclpy）。

覆盖 DESIGN §7-T04 验收点：
- 栅格化三态正确（边界外 -1 / 障碍 100 / 其余 0）
- world↔cell 互转往返一致
- **PGM 行序翻转正确**（构造非对称图案，验证上下不颠倒）
- PGM 三态取值：占用 0 / 空闲 254 / 未知 205
- 膨胀只进 ray_grid，不写 pgm，且未知区不被膨胀
"""

from __future__ import annotations

import os

import numpy as np
import pytest
import yaml

from gimbal_common.config_loader import ConfigError
from grid_mapper.field_config import FieldConfig
from grid_mapper.grid_builder import (FREE, OCCUPIED, UNKNOWN, GridMap, inflate)
from grid_mapper.pgm_io import (PGM_FREE, PGM_OCCUPIED, PGM_UNKNOWN,
                                grid_to_pgm_image, grid_to_pgm_bytes,
                                pgm_to_grid, read_pgm, write_map, write_map_yaml,
                                write_pgm)


# ---------------------------------------------------------------------------
# 夹具
# ---------------------------------------------------------------------------
def _field(resolution: float = 0.1, inflation: float = 0.0) -> FieldConfig:
    """4×4m 场地，origin (-2,-2)；边界是 2×2m 方框（外部为未知）；中心有一个 0.3m 方块障碍。"""
    return FieldConfig.from_dict({
        'grid': {
            'resolution': resolution,
            'width_m': 4.0,
            'height_m': 4.0,
            'origin_xyz': [-2.0, -2.0, 0.0],
            'inflation_radius_m': inflation,
        },
        'boundary': [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
        'obstacles': [
            {'type': 'rect', 'center_xy': [0.0, 0.0], 'size_xy': [0.3, 0.3], 'yaw': 0.0},
        ],
    })


# ---------------------------------------------------------------------------
# 1. 三态栅格化
# ---------------------------------------------------------------------------
def test_tristate_rasterization():
    """边界内空闲 = 0；障碍内 = 100；边界外 = -1。"""
    gm = GridMap.build(_field())
    assert gm.width == 40 and gm.height == 40
    assert gm.resolution == pytest.approx(0.1)

    # 障碍覆盖 |x|<=0.15, |y|<=0.15 → cell 中心 -0.15/-0.05/0.05/0.15
    assert gm.at((19, 19)) == OCCUPIED
    assert gm.at((18, 21)) == OCCUPIED
    # 边界内非障碍
    assert gm.at((25, 25)) == FREE
    assert gm.at((15, 15)) == FREE
    # 边界外（cell 30 → c = 1.05；cell 5 → c = -1.45，均 |c| > 1.0）
    assert gm.at((30, 30)) == UNKNOWN
    assert gm.at((5, 5)) == UNKNOWN

    st = gm.stats()
    assert st['n_occupied'] == 16          # 4×4 个障碍 cell
    assert st['n_unknown'] > 0
    assert st['n_free'] > 0
    assert st['n_occupied'] + st['n_free'] + st['n_unknown'] == 40 * 40


def test_circle_and_segment_obstacles():
    """circle / segment 型障碍按几何判定覆盖。"""
    cfg = FieldConfig.from_dict({
        'grid': {'resolution': 0.1, 'width_m': 4.0, 'height_m': 4.0,
                 'origin_xyz': [-2.0, -2.0, 0.0], 'inflation_radius_m': 0.0},
        'boundary': [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]],
        'obstacles': [
            {'type': 'circle', 'center_xy': [0.0, 0.0], 'radius': 0.25},
            {'type': 'segment', 'p0_xy': [-1.0, 1.0], 'p1_xy': [1.0, 1.0], 'width': 0.2},
        ],
    })
    gm = GridMap.build(cfg)
    assert gm.at(gm.world_to_cell((0.0, 0.0))) == OCCUPIED        # 圆心
    assert gm.at(gm.world_to_cell((0.0, 1.0))) == OCCUPIED        # 线段中点
    assert gm.at(gm.world_to_cell((-1.5, -1.5))) == FREE          # 空地
    assert gm.at(gm.world_to_cell((0.0, 0.6))) == FREE            # 离墙 0.4m > 0.1


def test_world_cell_roundtrip():
    """world → cell → world 往返：cell 中心与原点的偏差 ≤ res/2，且索引一致。"""
    gm = GridMap.build(_field())
    rng = np.random.default_rng(20261009)
    for _ in range(500):
        p = np.array([rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)])
        i, j = gm.world_to_cell(p)
        assert gm.in_bounds((i, j))
        c = gm.cell_to_world((i, j))
        assert np.all(np.abs(c - p) <= 0.5 * gm.resolution + 1e-12)
        # 再走一遍 world_to_cell 必须稳定（幂等）
        assert gm.world_to_cell(c) == (i, j)


def test_cell_to_world_center_formula():
    """cell 中心 = origin + (i+0.5, j+0.5) * res。"""
    gm = GridMap(resolution=0.05, origin_xy=(-6.0, -6.0),
                 grid=np.zeros((240, 240), dtype=np.int8))
    c = gm.cell_to_world((0, 0))
    assert c[0] == pytest.approx(-6.0 + 0.025)
    assert c[1] == pytest.approx(-6.0 + 0.025)
    c = gm.cell_to_world((239, 239))
    assert c[0] == pytest.approx(-6.0 + 239.5 * 0.05)
    assert c[1] == pytest.approx(-6.0 + 239.5 * 0.05)


def test_default_grid_is_240x240_origin_minus6():
    """DESIGN §6.11：0.05m/cell，12×12m → 240×240，origin [-6,-6,0]。"""
    gm = GridMap.build(FieldConfig.default())
    assert gm.shape == (240, 240)
    assert gm.resolution == pytest.approx(0.05)
    assert gm.origin_xy[0] == pytest.approx(-6.0)
    assert gm.origin_xy[1] == pytest.approx(-6.0)


# ---------------------------------------------------------------------------
# 2. 膨胀（只进 ray_grid）
# ---------------------------------------------------------------------------
def test_inflation_only_in_ray_grid():
    """膨胀半径 0.2m = 2 cells：2 cells 内变 100，3 cells 外仍为 0；未知区保持 -1。"""
    gm = GridMap.build(_field(resolution=0.1, inflation=0.2))
    center = gm.world_to_cell((0.0, 0.0))          # 障碍中心 cell
    assert gm.ray_grid[center[0], center[1]] == OCCUPIED
    # 距中心 2 cells → 应被膨胀
    assert gm.ray_grid[center[0] + 3, center[1]] == OCCUPIED
    # 距中心 4 cells（0.4m > 0.2m）→ 仍空闲
    assert gm.ray_grid[center[0] + 6, center[1]] == FREE
    # grid 本身不含膨胀
    assert gm.grid[center[0] + 3, center[1]] == FREE
    # 未知区（边界外）不被膨胀，保持 -1
    out = gm.world_to_cell((1.5, 1.5))
    assert gm.grid[out[0], out[1]] == UNKNOWN
    assert gm.ray_grid[out[0], out[1]] == UNKNOWN


def test_inflate_preserves_unknown():
    """inflate() 对 -1 cell 原样保留（未知区不膨胀、不变空闲）。"""
    g = np.full((10, 10), UNKNOWN, dtype=np.int8)
    g[5, 5] = OCCUPIED
    out = inflate(g, 0.2, 0.1)
    assert out[5, 5] == OCCUPIED
    assert out[0, 0] == UNKNOWN
    assert out[7, 5] == UNKNOWN           # 未知 cell 即使靠近障碍也保持 -1
    assert out[9, 5] == UNKNOWN


def test_inflate_dilates_free_cells():
    """空闲区：中心距 ≤ 2 cells（0.2m）被膨胀为 100，3 cells 外仍空闲。"""
    g = np.full((10, 10), FREE, dtype=np.int8)
    g[5, 5] = OCCUPIED
    out = inflate(g, 0.2, 0.1)
    assert out[5, 5] == OCCUPIED
    assert out[6, 5] == OCCUPIED          # 1 cell
    assert out[7, 5] == OCCUPIED          # 2 cells = 0.2m（边界，取 ≤）
    assert out[8, 5] == FREE              # 3 cells = 0.3m > 0.2m
    assert out[0, 0] == FREE


# ---------------------------------------------------------------------------
# 3. PGM 行序翻转（核心）
# ---------------------------------------------------------------------------
def _asymmetric_grid() -> np.ndarray:
    """构造一个**非对称**栅格：底行(j=0)占用、第 1 行未知、第 2 行未知、顶行(j=3)空闲。"""
    g = np.full((4, 4), UNKNOWN, dtype=np.int8)
    g[:, 0] = OCCUPIED        # y 最小 → 图像最底行
    g[:, 3] = FREE            # y 最大 → 图像最顶行
    return g


def test_pgm_row_order_flipped():
    """PGM 第 0 行 = 图像顶行 = y 最大。底行占用必须出现在图像**最后**一行。"""
    g = _asymmetric_grid()
    img = grid_to_pgm_image(g)
    assert img.shape == (4, 4)
    # 顶行（r=0）对应 j = H-1 = 3 → 空闲 254
    assert np.all(img[0, :] == PGM_FREE)
    # 底行（r=3）对应 j = 0 → 占用 0
    assert np.all(img[3, :] == PGM_OCCUPIED)
    # 中间两行是未知 205
    assert np.all(img[1, :] == PGM_UNKNOWN)
    assert np.all(img[2, :] == PGM_UNKNOWN)
    # 列方向（x）不翻转：把第 0 列设为占用，第 0 列仍应在图像第 0 列
    g2 = np.full((4, 4), FREE, dtype=np.int8)
    g2[0, :] = OCCUPIED
    img2 = grid_to_pgm_image(g2)
    assert np.all(img2[:, 0] == PGM_OCCUPIED)
    assert np.all(img2[:, 3] == PGM_FREE)


def test_pgm_write_read_roundtrip(tmp_path):
    """写 pgm → 读回 → 反演，必须与原始三态栅格**完全一致**（行序不颠倒）。"""
    g = _asymmetric_grid()
    p = str(tmp_path / 'map.pgm')
    write_pgm(p, g)

    # 文件头
    with open(p, 'rb') as f:
        head = f.read(16)
    assert head.startswith(b'P5\n4 4\n255\n')

    img, maxval = read_pgm(p)
    assert maxval == 255
    assert img.shape == (4, 4)
    assert np.all(img[0, :] == PGM_FREE)
    assert np.all(img[3, :] == PGM_OCCUPIED)

    back = pgm_to_grid(img, maxval=maxval)
    assert back.shape == g.shape
    assert np.array_equal(back, g)


def test_pgm_tristate_values(tmp_path):
    """PGM 三态取值：占用 0 / 空闲 254 / 未知 205。"""
    g = np.array([[OCCUPIED, FREE], [UNKNOWN, OCCUPIED]], dtype=np.int8)
    p = str(tmp_path / 't.pgm')
    write_pgm(p, g)
    img, _ = read_pgm(p)
    # grid[i, j]；g[0,0]=100 → 图像底行(r=1)第 0 列
    assert img[1, 0] == PGM_OCCUPIED        # 0
    assert img[1, 1] == PGM_UNKNOWN         # g[1,0] = -1 → r = H-1-0 = 1
    assert img[0, 0] == PGM_FREE            # g[0,1] = 0 → r = 0
    assert img[0, 1] == PGM_OCCUPIED        # g[1,1] = 100
    data = grid_to_pgm_bytes(g)
    assert set(np.frombuffer(data, dtype=np.uint8).tolist()) <= {0, 205, 254}


def test_map_yaml_content(tmp_path):
    """map.yaml 必须是 nav2_map_server 可加载的标准字段。"""
    p = str(tmp_path / 'map.yaml')
    write_map_yaml(p, image='map.pgm', resolution=0.05, origin_xyz=(-6.0, -6.0, 0.0))
    txt = open(p, 'r', encoding='utf-8').read()
    assert 'image: map.pgm' in txt
    assert 'mode: trinary' in txt
    assert 'resolution: 0.05' in txt
    assert 'origin: [-6, -6, 0]' in txt
    assert 'negate: 0' in txt
    assert 'occupied_thresh: 0.65' in txt
    assert 'free_thresh: 0.196' in txt


def test_write_map_creates_both_files(tmp_path):
    """write_map 一次产出 map.pgm + map.yaml，且 pgm 尺寸正确。"""
    gm = GridMap.build(_field())
    out = str(tmp_path / 'maps')
    paths = write_map(out, gm, origin_xyz=(-2.0, -2.0, 0.0))
    assert os.path.isfile(paths['pgm'])
    assert os.path.isfile(paths['yaml'])
    img, _ = read_pgm(paths['pgm'])
    assert img.shape == (40, 40)
    # 膨胀不写 pgm：ray_grid 的占用数应 ≥ grid 的占用数
    back = pgm_to_grid(img)
    assert np.array_equal(back, gm.grid)


# ---------------------------------------------------------------------------
# 4. OccupancyGrid 导出顺序
# ---------------------------------------------------------------------------
def test_occupancy_data_order():
    """导出顺序必须是 index = x + y*width（ROS OccupancyGrid 约定）。"""
    g = np.zeros((3, 2), dtype=np.int8)       # W=3, H=2
    g[2, 1] = OCCUPIED
    gm = GridMap.from_arrays(g, 0.05, (0.0, 0.0))
    data = gm.to_occupancy_data()
    assert len(data) == 6
    assert data[1 * 3 + 2] == OCCUPIED        # x=2, y=1
    assert sum(1 for v in data if v == OCCUPIED) == 1


def test_from_occupancy_grid_roundtrip():
    """从 OccupancyGrid 数据构造 → 导出，数据保持一致。"""
    g = np.full((5, 4), FREE, dtype=np.int8)
    g[1, 2] = OCCUPIED
    g[0, 0] = UNKNOWN
    data = GridMap.from_arrays(g, 0.05, (-1.0, -1.0)).to_occupancy_data()
    gm2 = GridMap.from_occupancy_grid(5, 4, 0.05, (-1.0, -1.0), data)
    assert np.array_equal(gm2.grid, g)


# ---------------------------------------------------------------------------
# 5. topic 模式（外部 SLAM 地图）：origin/resolution 原样保留 + ray_grid 才膨胀
# ---------------------------------------------------------------------------
def test_from_occupancy_grid_keeps_external_origin():
    """外部地图的 resolution/origin 必须原样保留，不能被 grid.* 参数改写。"""
    data = [0] * (4 * 3)
    gm = GridMap.from_occupancy_grid(4, 3, 0.05, (10.0, -20.0), data)
    assert gm.resolution == pytest.approx(0.05)
    assert gm.origin_xy[0] == pytest.approx(10.0)
    assert gm.origin_xy[1] == pytest.approx(-20.0)
    assert gm.shape == (4, 3)


def test_from_occupancy_grid_inflates_ray_grid_only():
    """topic 模式：ray_grid 含膨胀（供避障），grid 保持原始三态（供方向项）。"""
    w = h = 21
    g = np.full((w, h), FREE, dtype=np.int8)
    g[10, 10] = OCCUPIED
    data = GridMap.from_arrays(g, 0.1, (0.0, 0.0)).to_occupancy_data()

    plain = GridMap.from_occupancy_grid(w, h, 0.1, (0.0, 0.0), data)
    assert np.array_equal(plain.ray_grid, plain.grid), '未给半径时不应膨胀'

    infl = GridMap.from_occupancy_grid(w, h, 0.1, (0.0, 0.0), data,
                                       inflation_radius_m=0.2)
    # grid 一点没变（方向项用未膨胀图）
    assert np.array_equal(infl.grid, plain.grid)
    assert infl.at((10, 10), use_ray=True) == OCCUPIED
    assert infl.at((12, 10), use_ray=True) == OCCUPIED     # 2 cells = 0.2 m
    assert infl.at((13, 10), use_ray=True) == FREE         # 3 cells = 0.3 m
    assert infl.at((12, 10), use_ray=False) == FREE        # grid 仍是空闲
    assert infl.inflation_radius_m == pytest.approx(0.2)


def test_resolve_map_origin_prefers_grid_origin():
    """map.yaml 的 origin 取栅格自身的 origin；与参数不一致时给告警。"""
    from grid_mapper.grid_geometry import resolve_map_origin

    gm = GridMap.from_arrays(np.zeros((4, 4), dtype=np.int8), 0.05, (1.5, -2.5))
    # 参数与栅格 origin 一致 → 取栅格值、无告警
    org, warn = resolve_map_origin(gm, [1.5, -2.5, 0.0])
    assert abs(org[0] - 1.5) < 1e-12 and abs(org[1] + 2.5) < 1e-12
    assert warn == ''

    # topic 模式典型情况：参数是 -6,-6 而外部地图 origin 是别的 → 必须用栅格的值 + 告警
    gm2 = GridMap.from_occupancy_grid(4, 4, 0.05, (-6.0, -6.0),
                                      [0] * 16)
    org2, warn2 = resolve_map_origin(gm2, [-6.0, -6.0, 0.0])
    assert abs(org2[0] + 6.0) < 1e-12
    assert warn2 == ''

    gm3 = GridMap.from_occupancy_grid(4, 4, 0.05, (0.0, 0.0), [0] * 16)
    org3, warn3 = resolve_map_origin(gm3, [-6.0, -6.0, 0.0])
    assert abs(org3[0]) < 1e-12 and abs(org3[1]) < 1e-12, '必须用栅格 origin'
    assert '不一致' in warn3


def test_missing_or_broken_field_yaml_cannot_become_a_default_map(tmp_path):
    with pytest.raises(ConfigError):
        FieldConfig.from_yaml(str(tmp_path / 'missing.yaml'))
    broken = tmp_path / 'broken.yaml'
    broken.write_text('grid: [unterminated', encoding='utf-8')
    with pytest.raises(yaml.YAMLError):
        FieldConfig.from_yaml(str(broken))
    broken.write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError):
        FieldConfig.from_yaml(str(broken))


@pytest.mark.parametrize('bad', [
    None,
    {'grid': []},
    {'grid': {}, 'boundary': [[0, 0], [1, 0]]},
    {'grid': {'resolution': 0}, 'boundary': [[0, 0], [1, 0], [0, 1]]},
    {'grid': {}, 'boundary': [[0, 0], [1, 0], [0, 1]], 'obstacles': [
        {'type': 'rectangle_typo', 'center_xy': [0, 0], 'size_xy': [1, 1]}]},
    {'grid': {}, 'boundary': [[0, 0], [1, 0], [0, 1]], 'obstacles': [
        {'type': 'rect', 'center_xy': [0], 'size_xy': [1, 1]}]},
])
def test_invalid_field_geometry_cannot_silently_clear_obstacles(bad):
    with pytest.raises((ValueError, TypeError, KeyError)):
        FieldConfig.from_dict(bad)

