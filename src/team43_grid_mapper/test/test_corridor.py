"""平行墙只产出几何模型，不推断纵向全局位置。"""
import math

import numpy as np
import pytest

from team43_grid_mapper.corridor import fit_parallel_walls, corridor_grid


def walls(yaw=.4, shift=(1.,-2.)):
    s = np.linspace(-5.,5.,400)
    p = np.vstack((np.c_[s,np.full_like(s,-1.25)],np.c_[s,np.full_like(s,1.25)]))
    rng = np.random.default_rng(43)
    p += rng.normal(0,.01,p.shape)
    R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    return p@R.T+shift


@pytest.mark.parametrize('yaw',[.1,.8,1.6,2.8])
def test_width_invariant_to_pose_and_longitudinal_shift(yaw):
    fit = fit_parallel_walls(walls(yaw),expected_width=2.5)
    assert fit['width_m'] == pytest.approx(2.5,abs=.08)
    assert abs((fit['yaw_rad']-yaw+math.pi/2)%math.pi-math.pi/2) < .04
    grid,origin = corridor_grid(fit)
    assert grid.shape[0] == 800
    assert np.count_nonzero(grid[0] == 100) == 2  # 没有封闭端墙。
    assert np.count_nonzero(grid[-1] == 100) == 2
    assert np.count_nonzero(grid == 0) > 0
    assert len(origin) == 3


def test_single_wall_and_wrong_width_do_not_create_map():
    with pytest.raises(ValueError):
        fit_parallel_walls(walls()[:400])
    with pytest.raises(ValueError):
        fit_parallel_walls(walls(),expected_width=4.)
