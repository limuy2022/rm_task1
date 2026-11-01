"""实测点障碍保护：侧墙、端墙、包围、离散误差和旋转参考系。"""
import math

import numpy as np
import pytest

from team43_chassis_advisor.obstacle_guard import guard_velocity


def wall_x(x):
    return np.c_[np.full(161,x),np.linspace(-4.,4.,161)]


def assert_swept_clear(cmd,points):
    positions = np.linspace(0.,1.5,101)[:,None]*cmd
    distance = np.linalg.norm(points[None,:,:]-positions[:,None,:],axis=2)
    assert distance.min() >= .4-1e-8


def test_approaching_end_wall_slows_or_turns_without_crossing():
    points = wall_x(.65)
    cmd,angle,blocked = guard_velocity([.6,0.],points)
    assert not blocked and np.linalg.norm(cmd) > 0
    assert np.linalg.norm(cmd) <= .6
    assert_swept_clear(cmd,points)


def test_diagonal_side_wall_and_rotation_invariance():
    points = wall_x(.5)
    cmd,_,blocked = guard_velocity([.4,.4],points)
    assert not blocked
    assert_swept_clear(cmd,points)
    # 斜向近墙时保留沿墙移动，不向墙穿越。
    assert cmd[1] > 0
    assert cmd[0]*1.5 < .1
    R = np.array([[0.,-1.],[1.,0.]])
    rotated,_,_ = guard_velocity(R@np.array([.4,.4]),points@R.T)
    assert_swept_clear(rotated,points@R.T)


def test_already_near_wall_never_moves_toward_it():
    points = wall_x(.35)
    cmd,_,_ = guard_velocity([.5,0.],points)
    assert cmd[0] <= 1e-8
    away,_,blocked = guard_velocity([-.3,0.],points)
    assert not blocked and away[0] < 0


def test_enclosed_or_empty_cloud_returns_zero_without_speed_floor():
    a = np.linspace(0.,2*math.pi,100,endpoint=False)
    points = np.c_[.3*np.cos(a),.3*np.sin(a)]
    for p in (points,np.empty((0,2))):
        cmd,_,blocked = guard_velocity([.3,0.],p)
        assert blocked
        np.testing.assert_array_equal(cmd,[0.,0.])


def test_clear_forward_path_keeps_speed_and_bad_points_rejected():
    points = np.vstack((np.c_[np.linspace(-3.,3.,61),np.full(61,1.25)],
                        np.c_[np.linspace(-3.,3.,61),np.full(61,-1.25)]))
    cmd,angle,blocked = guard_velocity([.3,0.],points)
    np.testing.assert_allclose(cmd,[.3,0.]);assert angle == 0 and not blocked
    with pytest.raises(ValueError):
        guard_velocity([.3,0.],[[np.nan,0.]])
