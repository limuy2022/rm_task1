#!/usr/bin/env python3
"""Run real advisor callbacks/kernels using in-memory ROS substitutes, not DDS."""
from __future__ import annotations
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
ROOT=Path(__file__).resolve().parents[1]
for name in ('gimbal_common','chassis_advisor'):
    sys.path.insert(0,str(ROOT/'src'/name))
import chassis_advisor.chassis_advisor_node as node_module
from chassis_advisor.chassis_advisor_node import ChassisAdvisorNode, DEFAULTS
from chassis_advisor.cost_raycast import CostRaycast,RayGridAdapter,rotz2
from chassis_advisor.free_dir import FreeDirSolver,_ray_blocked
from chassis_advisor.velocity_law import VelocityLaw


def grid_message(arr,origin=(0.,0.),yaw=0.,stamp=100.,frame='map'):
    sec=int(stamp);ns=round((stamp-sec)*1e9)
    return NS(header=NS(frame_id=frame,stamp=NS(sec=sec,nanosec=ns)),
        info=NS(width=arr.shape[0],height=arr.shape[1],resolution=.1,
            origin=NS(position=NS(x=origin[0],y=origin[1],z=0.),
                      orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),
        data=arr.T.reshape(-1).tolist())


def base_message(xy,stamp=100.,frame='map'):
    return NS(header=NS(frame_id=frame,stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
              child_frame_id='base_link',pose=NS(pose=NS(position=NS(x=xy[0],y=xy[1],z=0.),
                                  orientation=NS(x=0.,y=0.,z=0.,w=1.))))


def target_message(xy,stamp=100.,frame='map',valid=True,age=0.):
    return NS(header=NS(frame_id=frame,stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
              valid=valid,track_status=1,cur_x=xy[0],cur_y=xy[1],pred_x=xy[0],pred_y=xy[1],
              last_obs_age=age)


class AdvisorOfflineTests(unittest.TestCase):
    def setUp(self):
        self.time=100.
        self.original_now=node_module.now
        node_module.now=lambda:self.time
        self.app=ChassisAdvisorNode.__new__(ChassisAdvisorNode)
        a=self.app
        a._params=dict(DEFAULTS)
        a._map_frame='map';a._has_base=a._has_target=a._has_grid=a._has_clean_grid=False
        a._base_stamp=a._target_stamp=a._base_received=a._target_received=-math.inf
        a._observation_stamp=-math.inf
        a._base_xy=a._goal_xy=a._cur_xy=np.zeros(2)
        a._base_yaw=0.;a._track_status=0;a._t_lost=None;a._t_last=self.time-.02
        a._dist=a._theta=0.;a._blocked=False;a._ray_adapter=a._clean_adapter=None
        a._ray=CostRaycast();a._free=FreeDirSolver();a._law=VelocityLaw()
        a._log=NS(warning=lambda *args:None,error=lambda *args:None)
        a.get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(self.time*1e9)))
        self.results=[]
        a._publish=lambda cmd,dist,theta,blocked,valid:self.results.append(
            NS(cmd=np.asarray(cmd),dist=dist,theta=theta,blocked=blocked,valid=valid))

    def tearDown(self):node_module.now=self.original_now

    @staticmethod
    def asymmetric_wall(mirror=False):
        arr=np.zeros((45,40),dtype=np.int8)
        arr[15:17,15:25]=100
        if mirror:arr=arr[:,::-1].copy()
        return arr

    def feed(self,arr=None,origin=(0.,0.),yaw=0.,base=(1.05,1.55),goal=(3.05,1.55)):
        if arr is not None:
            self.app._on_clean_grid(grid_message(arr,origin,yaw,self.time))
            self.app._on_ray_grid(grid_message(arr,origin,yaw,self.time))
        self.app._on_base_pose(base_message(base,self.time))
        self.app._on_target(target_message(goal,self.time))
        self.app._on_timer()
        return self.results[-1]

    def test_empty_map_straight_and_stop_within_radius(self):
        result=self.feed(np.zeros((45,40),np.int8))
        self.assertTrue(result.valid);self.assertFalse(result.blocked)
        np.testing.assert_allclose(result.cmd,[.3,0],atol=1e-12)
        self.time+=.01
        self.app._on_target(target_message((1.3,1.55),self.time));self.app._on_timer()
        np.testing.assert_allclose(self.results[-1].cmd,0)
        self.assertTrue(self.results[-1].valid)

    def test_asymmetric_wall_chooses_nearest_free_side_both_directions(self):
        for mirror in (False,True):
            self.setUp_case_time()
            arr=self.asymmetric_wall(mirror)
            base=(1.05,1.55) if not mirror else (1.05,2.45)
            goal=(3.05,base[1])
            result=self.feed(arr,base=base,goal=goal)
            self.assertTrue(result.valid);self.assertFalse(result.blocked)
            self.assertLess(result.cmd[1],0) if not mirror else self.assertGreater(result.cmd[1],0)
            angle=math.atan2(result.cmd[1],result.cmd[0])
            self.assertFalse(_ray_blocked(arr,(0,0),.1,base,angle,1.,False))

    def setUp_case_time(self):
        self.time+=.01
        self.app._law.reset()

    def test_rotated_nonsymmetric_grid_rotates_advice(self):
        arr=self.asymmetric_wall()
        ordinary=self.feed(arr)
        self.time+=.01
        yaw=1.1;origin=np.array([2.,-3.]);R=rotz2(yaw)
        result=self.feed(arr,origin,yaw,origin+R@np.array([1.05,1.55]),origin+R@np.array([3.05,1.55]))
        self.assertTrue(result.valid);self.assertFalse(result.blocked)
        np.testing.assert_allclose(result.cmd/np.linalg.norm(result.cmd),
                                   R@ordinary.cmd/np.linalg.norm(ordinary.cmd),atol=1e-10)

    def test_rotated_quantized_ray_map_rotates_advice(self):
        self.app._params['chassis.dir_mode']='quantized'
        arr=self.asymmetric_wall()
        ordinary=self.feed(arr)
        self.time+=.01
        yaw=math.pi/2;origin=np.array([-2.,3.]);R=rotz2(yaw)
        result=self.feed(arr,origin,yaw,origin+R@np.array([1.05,1.55]),origin+R@np.array([3.05,1.55]))
        self.assertTrue(result.valid);self.assertFalse(result.blocked)
        np.testing.assert_allclose(result.cmd/np.linalg.norm(result.cmd),
                                   R@ordinary.cmd/np.linalg.norm(ordinary.cmd),atol=1e-12)

    def test_blocked_zero_is_not_turned_into_through_wall_floor(self):
        self.feed(np.zeros((45,40),np.int8))
        self.time+=.01
        result=self.feed(np.full((45,40),100,np.int8))
        self.assertTrue(result.valid);self.assertTrue(result.blocked)
        np.testing.assert_array_equal(result.cmd,[0,0])
        np.testing.assert_array_equal(self.app._law.cmd_out,[0,0])
        np.testing.assert_array_equal(self.app._apply_v_floor(np.zeros(2)),[0,0])

    def test_missing_or_bad_frame_map_is_invalid(self):
        result=self.feed()
        self.assertFalse(result.valid);np.testing.assert_array_equal(result.cmd,[0,0])
        self.time+=.01;arr=np.zeros((45,40),np.int8)
        self.feed(arr)
        self.app._on_clean_grid(grid_message(arr,stamp=self.time,frame='lio_map'))
        self.app._on_timer()
        self.assertFalse(self.results[-1].valid)
        self.assertFalse(self.app._has_clean_grid)

    def test_invalid_target_and_nonfinite_current_stop_immediately(self):
        self.feed(np.zeros((45,40),np.int8))
        self.app._on_target(target_message((3,1.55),self.time,valid=False))
        self.app._on_timer();self.assertFalse(self.results[-1].valid)
        self.time+=.01
        self.app._on_target(target_message((math.nan,1.55),self.time))
        self.app._on_timer();self.assertFalse(self.results[-1].valid)
        np.testing.assert_array_equal(self.results[-1].cmd,[0,0])

    def test_disconnect_and_duplicate_old_source_do_not_renew_valid_advice(self):
        arr=np.zeros((45,40),np.int8);self.feed(arr)
        self.time+=.3
        # Duplicate data arriving now must keep its old measurement stamp/time.
        self.app._on_base_pose(base_message((1.05,1.55),100.))
        self.app._on_target(target_message((3.05,1.55),100.))
        self.app._on_timer()
        self.assertFalse(self.results[-1].valid)
        np.testing.assert_array_equal(self.results[-1].cmd,[0,0])
        self.time+=.01
        result=self.feed(arr)
        self.assertTrue(result.valid)

    def test_fresh_output_with_expired_observation_is_invalid(self):
        self.feed(np.zeros((45,40),np.int8))
        self.time+=.01
        self.app._on_target(target_message((3.05,1.55),self.time,age=.6))
        self.app._on_timer()
        self.assertFalse(self.results[-1].valid)

    def test_observation_expiry_between_messages_stops_before_message_timeout(self):
        self.feed(np.zeros((45,40),np.int8))
        self.time+=.01
        self.app._on_target(target_message((3.05,1.55),self.time,age=.45))
        self.app._on_timer();self.assertTrue(self.results[-1].valid)
        self.time+=.06
        self.app._on_timer();self.assertFalse(self.results[-1].valid)
        np.testing.assert_array_equal(self.results[-1].cmd,[0,0])

    def test_wrong_base_frame_future_stamp_and_invalid_map_quaternion(self):
        self.feed(np.zeros((45,40),np.int8))
        self.app._on_base_pose(base_message((1,2),self.time+.5))
        self.app._on_timer();self.assertFalse(self.results[-1].valid)
        self.time+=.01;self.feed(np.zeros((45,40),np.int8))
        self.app._on_base_pose(base_message((1,2),self.time,frame='odom'))
        self.app._on_timer();self.assertFalse(self.results[-1].valid)
        msg=grid_message(np.zeros((45,40),np.int8),stamp=self.time)
        msg.info.origin.orientation.w=0
        self.app._on_clean_grid(msg)
        self.assertFalse(self.app._has_clean_grid)

    def test_unknown_boundary_is_avoided_when_conservative_mode_is_selected(self):
        self.app._params['chassis.dir_block_unknown']=True
        self.app._free=FreeDirSolver(block_unknown=True)
        arr=np.zeros((45,40),np.int8)
        result=self.feed(arr,base=(4.05,1.55),goal=(6.05,1.55))
        self.assertTrue(result.valid);self.assertFalse(result.blocked)
        angle=math.atan2(result.cmd[1],result.cmd[0])
        self.assertGreater(abs(angle),math.radians(60))
        self.assertFalse(_ray_blocked(arr,(0,0),.1,(4.05,1.55),angle,1.,True))

    def test_origin_yaw_cell_conversion_and_ray_cast_agree(self):
        arr=self.asymmetric_wall();origin=np.array([2.,3.]);yaw=.8;R=rotz2(yaw)
        adapter=RayGridAdapter(arr,origin,.1,yaw)
        for ij in ((0,0),(44,39),(15,17)):
            self.assertEqual(adapter.world_to_cell(adapter.cell_to_world(ij)),ij)
        base=origin+R@np.array([1.05,1.55])
        caster=CostRaycast(grid=adapter)
        hits=caster.cast(adapter,base,[yaw],L=1.)
        self.assertTrue(hits[0].hit_occ)
        self.assertAlmostEqual(hits[0].d_occ,.5,10)


if __name__=='__main__':unittest.main(verbosity=2)
