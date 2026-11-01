"""跨会话匹配：平移旋转恢复、对称多解拒绝、文件与格子方向。"""
import math
import numpy as np
import pytest
from team43_grid_mapper.grid_builder import GridMap
from team43_grid_mapper.pgm_io import write_map
from team43_grid_mapper.saved_map import SavedMap, rotation, angle


def scene(tmp_path, symmetric=False):
    grid=np.zeros((100,80),np.int8)
    grid[:2,:]=100;grid[-2:,:]=100;grid[:,:2]=100;grid[:,-2:]=100
    if not symmetric:
        grid[20:25,12:38]=100;grid[60:84,50:55]=100;grid[70:75,10:20]=100
    gm=GridMap.from_arrays(grid,.05,[-2.5,-2.],inflation_radius_m=0.)
    paths=write_map(str(tmp_path),gm)
    model=SavedMap(paths['yaml'])
    cells=np.argwhere(grid==100)
    points=(cells+.5)*.05+[-2.5,-2.]
    return model,points


def test_global_relocalizes_after_restart_in_new_position_and_heading(tmp_path):
    model,points=scene(tmp_path)
    pose=np.array([.65,-.45,.72])
    current=(points-pose[:2]) @ rotation(pose[2])
    result=model.match(current)
    assert result['accepted'],result
    assert np.linalg.norm(np.array(result['pose'])[:2]-pose[:2])<.12
    assert abs(angle(result['pose'][2]-pose[2]))<.06


def test_symmetric_room_refuses_global_guess_but_accepts_verified_pose_hint(tmp_path):
    model,points=scene(tmp_path,symmetric=True)
    pose=np.array([.3,-.2,.4]);current=(points-pose[:2]) @ rotation(pose[2])
    assert not model.match(current)['accepted']
    result=model.match(current,hint=pose+[.2,-.15,.08])
    assert result['accepted'],result
    assert np.linalg.norm(np.array(result['pose'])[:2]-pose[:2])<.12


def test_bad_or_sparse_cloud_does_not_create_alignment(tmp_path):
    model,_=scene(tmp_path)
    with pytest.raises(ValueError):model.match(np.zeros((10,2)))
    with pytest.raises(ValueError):model.match(np.full((50,2),np.nan))
    # 不同场景的圆形回波不应被方形墙体接纳。
    theta=np.linspace(0,2*math.pi,200)
    assert not model.match(np.column_stack((np.cos(theta),np.sin(theta)))*1.3)['accepted']


def test_saved_map_preserves_unknown_cells_and_rejects_empty_map(tmp_path):
    model,_=scene(tmp_path)
    assert model.grid[0,0]==100 and model.grid[10,10]==0
    empty=GridMap.from_arrays(np.full((20,20),-1,np.int8),.05,[0,0],inflation_radius_m=0.)
    paths=write_map(str(tmp_path/'empty'),empty)
    with pytest.raises(ValueError,match='足够'):SavedMap(paths['yaml'])
