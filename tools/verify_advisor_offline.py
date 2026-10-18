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


def grid_message(arr,origin=(0.,0.),yaw=0.,stamp=100.,frame='map',resolution=.1):
    sec=int(stamp);ns=round((stamp-sec)*1e9)
    return NS(header=NS(frame_id=frame,stamp=NS(sec=sec,nanosec=ns)),
        info=NS(width=arr.shape[0],height=arr.shape[1],resolution=resolution,
            origin=NS(position=NS(x=origin[0],y=origin[1],z=0.),
                      orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),
        data=arr.T.reshape(-1).tolist())


def base_message(xy,stamp=100.,frame='map',yaw=0.):
    return NS(header=NS(frame_id=frame,stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
              child_frame_id='base_link',pose=NS(pose=NS(position=NS(x=xy[0],y=xy[1],z=0.),
                                  orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))))


def target_message(xy,stamp=100.,frame='map',valid=True,age=0.,prediction=None):
    prediction=xy if prediction is None else prediction
    return NS(header=NS(frame_id=frame,stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
              valid=valid,track_status=1,cur_x=xy[0],cur_y=xy[1],pred_x=prediction[0],pred_y=prediction[1],
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
        a._map_stamps={'analytic':-math.inf,'quantized':-math.inf}
        a._map_received={'analytic':-math.inf,'quantized':-math.inf}
        a._base_xy=a._goal_xy=a._cur_xy=np.zeros(2)
        a._base_yaw=0.;a._track_status=0;a._t_lost=None;a._t_last=self.time-.02
        a._dist=a._theta=0.;a._blocked=False;a._ray_adapter=a._clean_adapter=None
        a._ray=CostRaycast();a._free=FreeDirSolver(block_unknown=a._params['chassis.dir_block_unknown']);a._law=VelocityLaw()
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

    def test_target_behind_produces_map_negative_x_not_arbitrary_forward(self):
        # 这只是地图坐标系里的速度建议，不下发底盘。目标在后方仍朝目标。
        result=self.feed(np.zeros((70,60),np.int8),base=(3.05,3.05),goal=(1.05,3.05))
        self.assertTrue(result.valid)
        self.assertLess(result.cmd[0],0.)
        self.assertAlmostEqual(result.cmd[1],0.,12)
        self.assertLessEqual(np.linalg.norm(result.cmd),self.app._params['chassis.v_max'])

    def test_speed_limit_applies_to_vector_norm_in_all_quadrants(self):
        arr=np.zeros((90,80),np.int8);base=np.array([4.05,4.05])
        self.app._params['chassis.v_max']=.12
        for angle in np.linspace(-math.pi,math.pi,9):
            self.time+=.02
            unit=np.array([math.cos(angle),math.sin(angle)])
            result=self.feed(arr,base=base,goal=base+2*unit)
            self.assertTrue(result.valid)
            self.assertLessEqual(np.linalg.norm(result.cmd),.12+1e-12)
            self.assertGreaterEqual(np.linalg.norm(result.cmd),.02)
            np.testing.assert_allclose(result.cmd/np.linalg.norm(result.cmd),unit,atol=1e-12)

    def test_approach_slowdown_does_not_get_flattened_by_far_speed_floor(self):
        arr=np.zeros((90,80),np.int8);base=np.array([3.05,3.05])
        speeds=[]
        for distance in (2.,.8,.6,.51,.46,.44):
            self.time+=.02
            self.app._law.reset([.5,0.])
            result=self.feed(arr,base=base,goal=base+[distance,0.])
            self.assertTrue(result.valid)
            speeds.append(float(np.linalg.norm(result.cmd)))
        self.assertAlmostEqual(speeds[0],.5,12)
        self.assertTrue(all(a>=b for a,b in zip(speeds,speeds[1:])),speeds)
        self.assertLess(speeds[2],.3) # 近处确实减速，不被旧 .30 下限覆盖。
        self.assertGreaterEqual(speeds[3],.02) # 仍相距 .51m 时不能误报停车。
        self.assertGreaterEqual(speeds[4],.02)
        self.assertEqual(speeds[5],0.)

    def test_predictive_heading_uses_current_distance_for_stop(self):
        arr=np.zeros((90,80),np.int8);base=(3.05,3.05)
        self.feed(arr,base=base,goal=(5.05,3.05))
        self.app._params['chassis.dir_use_pred']=True
        self.time+=.02
        # 当前目标在前方2米，预测点恰在车中心：不能产生任意朝向或假停车。
        self.app._on_base_pose(base_message(base,self.time))
        self.app._on_target(target_message((5.05,3.05),self.time,prediction=base))
        self.app._on_timer();result=self.results[-1]
        self.assertTrue(result.valid);self.assertGreater(result.cmd[0],.02)
        self.assertAlmostEqual(result.cmd[1],0.,12)
        self.assertAlmostEqual(result.dist,2.,12)
        self.time+=.02
        # 换成预测点在左侧，真正改变方向，而非仅保存预测值。
        self.app._on_target(target_message((5.05,3.05),self.time,prediction=(3.05,5.05)))
        self.app._on_timer();result=self.results[-1]
        self.assertAlmostEqual(result.cmd[0],0.,12);self.assertGreater(result.cmd[1],.02)
        self.time+=.02
        # 当前已经很近时，即使预测点远，也按官方当前距离停车。
        self.app._on_target(target_message((3.25,3.05),self.time,prediction=(3.05,5.05)))
        self.app._on_timer();np.testing.assert_array_equal(self.results[-1].cmd,[0,0])

    def test_unknown_area_and_outside_boundary_not_assumed_free_by_default(self):
        arr=np.full((90,80),-1,np.int8)
        arr[20:41,20:41]=0
        result=self.feed(arr,base=(3.95,3.05),goal=(5.95,3.05))
        self.assertTrue(result.valid);self.assertFalse(result.blocked)
        angle=math.atan2(result.cmd[1],result.cmd[0])
        self.assertFalse(_ray_blocked(arr,(0,0),.1,(3.95,3.05),angle,1.,True))
        self.assertGreater(abs(angle),math.radians(75))
        # 全周围未知，中心仅1格已知：不制造一条穿未知区的方向。
        self.time+=.02;arr[:]=-1;arr[39,30]=0
        result=self.feed(arr,base=(3.95,3.05),goal=(5.95,3.05))
        self.assertTrue(result.blocked);np.testing.assert_array_equal(result.cmd,[0,0])

    def test_base_frame_option_rotates_map_advice_by_base_yaw(self):
        arr=np.zeros((90,80),np.int8);base=(3.05,3.05)
        self.feed(arr,base=base,goal=(5.05,3.05))
        self.app._params['chassis.cmd_frame']='base_link'
        self.time+=.02
        self.app._on_base_pose(base_message(base,self.time,yaw=math.pi/2))
        self.app._on_target(target_message((5.05,3.05),self.time))
        self.app._on_timer();result=self.results[-1]
        self.assertAlmostEqual(result.cmd[0],0.,12)
        self.assertLess(result.cmd[1],-.02)

    def test_grid_resolution_is_read_from_message_not_assumed_point_one(self):
        arr=np.zeros((90,80),np.int8)
        self.app._on_clean_grid(grid_message(arr,resolution=.05))
        self.app._on_ray_grid(grid_message(arr,resolution=.05))
        result=self.feed(base=(1.5,1.5),goal=(3.5,1.5))
        self.assertTrue(result.valid);self.assertGreater(result.cmd[0],0.)
        self.assertAlmostEqual(self.app._clean_adapter.resolution,.05)

    def test_live_map_optional_timeout_and_duplicate_does_not_renew_it(self):
        arr=np.zeros((90,80),np.int8)
        self.app._params['chassis.map_timeout_s']=.1
        self.feed(arr,base=(3.05,3.05),goal=(5.05,3.05))
        self.time+=.15
        self.app._on_clean_grid(grid_message(arr,stamp=100.))
        self.app._on_base_pose(base_message((3.05,3.05),self.time))
        self.app._on_target(target_message((5.05,3.05),self.time))
        self.app._on_timer();self.assertFalse(self.results[-1].valid)
        np.testing.assert_array_equal(self.results[-1].cmd,[0,0])
        self.app._on_clean_grid(grid_message(arr,stamp=self.time))
        self.app._on_timer();self.assertTrue(self.results[-1].valid)

    def test_latched_static_map_stays_valid_and_late_older_map_cannot_roll_it_back(self):
        arr=np.zeros((90,80),np.int8)
        self.feed(arr,base=(3.05,3.05),goal=(5.05,3.05))
        self.time+=20.
        result=self.feed(base=(3.05,3.05),goal=(5.05,3.05))
        self.assertTrue(result.valid) # 静态场地地图不会仅因年龄失效。
        self.app._on_clean_grid(grid_message(np.full_like(arr,100),stamp=99.))
        self.app._on_timer();self.assertTrue(self.results[-1].valid)
        self.assertFalse(self.results[-1].blocked)


if __name__=='__main__':unittest.main(verbosity=2)
