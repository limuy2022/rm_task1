#!/usr/bin/env python3
"""Offline checks for 3D sensor/base odometry and rotated OccupancyGrid IO.

Run from any directory: .venv/bin/python tools/verify_navigation_offline.py
Executes real node classes and real ROS message classes; node constructors are
bypassed via ``__new__`` with injected fields. No sensor, hardware command,
DDS traffic or protected directory is accessed.
"""
from __future__ import annotations
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ('team43_common', 'team43_localization', 'team43_grid_mapper'):
    sys.path.insert(0, str(ROOT/'src'/package))
from builtin_interfaces.msg import Time as Stamp
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import TransformException

from team43_localization import base_pose_provider_node as provider_module
from team43_localization.base_pose_source import OdomTopicSource
from team43_localization.odometry_geometry import (  # noqa:E402
    rigid_transform, quaternion_matrix, matrix_quaternion, convert_odometry,
)
from team43_grid_mapper.grid_builder import GridMap  # noqa:E402
from team43_grid_mapper.grid_geometry import compose_grid_origin, planar_quaternion_yaw  # noqa:E402
from team43_grid_mapper import grid_mapper_node as adapter  # noqa:E402
from team43_grid_mapper.pgm_io import write_map, read_grid  # noqa:E402
from team43_grid_mapper.png_draw import world_to_pixel  # noqa:E402


def odom(xyz=(0.,0.,0.), xyzw=(0.,0.,0.,1.), stamp=(12,123456789),
         linear=(0.,0.,0.), angular=(0.,0.,0.), parent='odom', child='base_link'):
    message = Odometry()
    message.header.frame_id = parent
    message.header.stamp.sec, message.header.stamp.nanosec = int(stamp[0]), int(stamp[1])
    message.child_frame_id = child
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = \
        map(float, xyz)
    message.pose.pose.orientation.x, message.pose.pose.orientation.y, \
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = map(float, xyzw)
    message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = \
        map(float, linear)
    message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = \
        map(float, angular)
    return message


def independent_rotation(rpy):
    r,p,y=rpy
    Rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz@Ry@Rx


class NavigationOfflineTests(unittest.TestCase):
    def test_sensor_lever_arm_after_full_3d_rotation(self):
        rpy=(0.4,-0.5,1.1)
        R=independent_rotation(rpy)
        base=np.array([1.2,-2.3,0.35])
        lever=np.array([0.31,-0.19,0.48])
        Rbs=independent_rotation((-0.1,0.2,-0.3))
        sensor=base+R@lever
        message=odom(sensor,matrix_quaternion(R@Rbs))
        sample=convert_odometry(message,rigid_transform(lever,(-.1,.2,-.3)),np.eye(4))
        np.testing.assert_allclose(sample.T_output_base[:3,3],base,atol=1e-12)
        np.testing.assert_allclose(sample.T_output_base[:3,:3],R,atol=1e-12)
        self.assertAlmostEqual(sample.yaw,rpy[2],12)
        # A planar-only lever arm subtraction is observably wrong on this pose.
        planar=independent_rotation((0,0,rpy[2]))
        self.assertGreater(np.linalg.norm(sensor-planar@lever-base),.1)

    def test_rotation_near_pi_roundtrip_and_invalid_quaternion(self):
        for rpy in ((math.pi,0,0),(0,math.pi,0),(0,0,math.pi),(.7,-1.2,2.9)):
            R=independent_rotation(rpy)
            np.testing.assert_allclose(quaternion_matrix(matrix_quaternion(R)),R,atol=1e-12)
        for q in ((0,0,0,0),(math.nan,0,0,1)):
            with self.assertRaises(ValueError):quaternion_matrix(q)

    def test_parent_transform_applies_to_pose_and_synthetic_cloud(self):
        R=independent_rotation((.2,-.1,.7)); t=np.array([3.,-1.,.5])
        Tp=rigid_transform(t,(.2,-.1,.7))
        sample=convert_odometry(odom((1,2,3)),np.eye(4),Tp)
        np.testing.assert_allclose(sample.T_output_base[:3,3],R@np.array([1,2,3])+t)
        cloud=np.array([[0,0,0],[1,0,0],[0,2,0],[0,0,3]],dtype=float)
        expected=np.array([R@point+t for point in cloud])
        transformed=(Tp[:3,:3]@cloud.T).T+Tp[:3,3]
        np.testing.assert_allclose(transformed,expected)
        self.assertGreater(abs(transformed[2,0]-transformed[0,0]),.5)

    def test_twist_lever_arm_and_sensor_axis_rotation(self):
        Tbs=rigid_transform((.3,0,0),(0,0,math.pi/2))
        msg=odom(linear=(.5,-.4,0), angular=(0,0,2))
        sample=convert_odometry(msg,Tbs,np.eye(4))
        np.testing.assert_allclose(sample.linear_base,[.4,-.1,0],atol=1e-12)
        np.testing.assert_allclose(sample.angular_base,[0,0,2],atol=1e-12)

    def test_pose_difference_uses_source_time_and_corrected_base_center(self):
        Tbs=rigid_transform((.3,0,.5))
        first=convert_odometry(odom((.3,0,.5),stamp=(10,0)),Tbs,np.eye(4),'pose_difference')
        second=convert_odometry(odom((0,.3,.5),(0,0,math.sqrt(.5),math.sqrt(.5)),
                                    stamp=(10,100000000)),Tbs,np.eye(4),'pose_difference',first)
        self.assertFalse(first.velocity_valid)
        self.assertTrue(second.velocity_valid)
        np.testing.assert_allclose(second.linear_base,[0,0,0],atol=1e-12)
        self.assertAlmostEqual(second.angular_base[2],5*math.pi,10)

    def test_source_stamp_duplicate_does_not_refresh_transport_age(self):
        source=OdomTopicSource(timeout_s=.5)
        source.on_odom(odom(),100.)
        self.assertEqual((source.latest.stamp_sec,source.latest.stamp_nanosec),(12,123456789))
        source.on_odom(odom(),100.4)
        self.assertEqual(source.n_msgs,1)
        self.assertFalse(source.poll(100.6).valid)
        with self.assertRaises(ValueError):source.on_odom(odom(parent='wrong'),100.7)
        self.assertFalse(source.poll(100.71).valid)
        source.on_odom(odom(stamp=(13,0)),100.8)
        self.assertTrue(source.poll(100.81).valid)
        with self.assertRaises(ValueError):source.on_odom(odom(stamp=(11,0)),100.9)
        self.assertFalse(source.poll(100.91).valid)
        source.reset();source.on_odom(odom(stamp=(0,0)),101.)
        self.assertTrue(source.poll(101.01).valid)

    def test_unconfirmed_extrinsics_and_frame_alias_are_rejected(self):
        with self.assertRaises(ValueError):OdomTopicSource(child_frame_id='body')
        with self.assertRaises(ValueError):OdomTopicSource(frame_id='camera_init')
        source=OdomTopicSource(frame_id='camera_init',child_frame_id='body',
            sensor_extrinsics_confirmed=True,parent_transform_confirmed=True,
            sensor_in_base_xyz=(.3,0,.5),twist_source='pose_difference')
        source.on_odom(odom((.3,0,.5),parent='camera_init',child='body'),1.)
        self.assertEqual((source.frame_id,source.child_frame_id),('odom','base_link'))
        np.testing.assert_allclose(source.latest.T_output_base[:3,3],0,atol=1e-12)

    def test_ros_source_twist_becomes_output_frame_velocity_for_pure_consumer(self):
        source=OdomTopicSource()
        source.on_odom(odom(xyzw=(0,0,math.sqrt(.5),math.sqrt(.5)),linear=(1,0,0)),1.)
        p=source.poll(1.1)
        self.assertAlmostEqual(p.vx,0,12);self.assertAlmostEqual(p.vy,1,12)
        np.testing.assert_allclose(source.latest.linear_base,[1,0,0],atol=1e-12)

    def test_rotated_nonsquare_grid_cells_ros_order_and_pgm_yaml(self):
        data=[-1,0,100,100,-1,0]  # width3 height2; not symmetric in x or y
        gm=GridMap.from_occupancy_grid(3,2,.2,(2.,-1.),data,origin_yaw=math.pi/2)
        self.assertEqual(gm.to_occupancy_data(),data)
        np.testing.assert_allclose(gm.cell_to_world((2,0)),[1.9,-.5],atol=1e-12)
        for i in range(3):
            for j in range(2):
                self.assertEqual(gm.world_to_cell(gm.cell_to_world((i,j))),(i,j))
                np.testing.assert_allclose(world_to_pixel(gm.cell_to_world((i,j)),gm),[i,1-j],atol=1e-12)
        with tempfile.TemporaryDirectory() as d:
            paths=write_map(d,gm,image_name='43_map.pgm',yaml_name='43_map.yaml')
            np.testing.assert_array_equal(read_grid(paths['pgm']).grid,gm.grid)
            content=Path(paths['yaml']).read_text()
            self.assertIn('image: 43_map.pgm',content)
            self.assertIn('origin: [2, -1, 1.570796]',content)

    def test_grid_metadata_frame_composition_does_not_resample(self):
        xy,yaw=compose_grid_origin([2.,-1.],math.pi/4,[3.,4.,0.],
                                  [0,0,math.sqrt(.5),math.sqrt(.5)])
        np.testing.assert_allclose(xy,[4.,6.],atol=1e-12)
        self.assertAlmostEqual(yaw,3*math.pi/4,12)
        old=GridMap.from_arrays(np.array([[0,-1],[100,0],[0,100]],dtype=np.int8),.1,[2,-1],origin_yaw=math.pi/4)
        new=GridMap.from_arrays(old.grid,.1,xy,origin_yaw=yaw)
        for ij in ((0,0),(2,1)):
            x,y=old.cell_to_world(ij)
            np.testing.assert_allclose(new.cell_to_world(ij),[-y+3,x+4],atol=1e-12)
        np.testing.assert_array_equal(old.grid,new.grid)
        with self.assertRaises(ValueError):planar_quaternion_yaw([math.sin(.1),0,0,math.cos(.1)])

    def test_malformed_external_grid_and_ambiguous_probability(self):
        for args in ((0,2,.1,[0,0],[]),(2,2,0,[0,0],[0]*4),(2,2,.1,[0,0],[0]*3),
                     (2,2,.1,[0,0],[0,0,0,101])):
            with self.assertRaises(ValueError):GridMap.from_occupancy_grid(*args)
        gm=GridMap.from_occupancy_grid(5,1,.1,[0,0],[-1,19,20,64,65])
        self.assertEqual(gm.to_occupancy_data(),[-1,0,-1,-1,100])


class RosAdapterTests(unittest.TestCase):
    """Execute real ROS adapter methods on real message types.

    Constructors are bypassed (``__new__``) and the source/publishers injected;
    the conversion logic under test is the production code. These checks do not
    assert DDS/QoS or rclpy lifecycle behaviour.
    """
    def test_provider_preserves_all_3d_pose_stamp_and_child_twist(self):
        src=OdomTopicSource()
        q=matrix_quaternion(independent_rotation((.2,.3,.4)))
        src.on_odom(odom((1,2,.7),q,linear=(.4,.5,.6),angular=(.1,.2,.3)),100.)
        out=[];tfs=[]
        app=provider_module.BasePoseProviderNode.__new__(provider_module.BasePoseProviderNode)
        app._source=src;app._pub=NS(publish=out.append);app._tf=NS(sendTransform=tfs.append)
        stamp=Stamp(sec=12,nanosec=123456789)
        provider_module.BasePoseProviderNode._publish_odom(app,src.poll(100.1),stamp)
        provider_module.BasePoseProviderNode._publish_tf_msg(app,src.poll(100.1),stamp)
        m=out[0]
        self.assertEqual(m.header.stamp,stamp)
        self.assertEqual(tfs[0].header.stamp,stamp)
        self.assertEqual((m.header.frame_id,m.child_frame_id),('odom','base_link'))
        self.assertAlmostEqual(m.pose.pose.position.z,.7)
        np.testing.assert_allclose([m.pose.pose.orientation.x,m.pose.pose.orientation.y,
                                   m.pose.pose.orientation.z,m.pose.pose.orientation.w],q)
        np.testing.assert_allclose([m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z],[.4,.5,.6])
        self.assertAlmostEqual(tfs[0].transform.translation.z,.7)

    def test_map_callback_waits_for_tf_then_publishes_and_preserves_metadata(self):
        saved=[];outputs=[]
        available=[False]
        def lookup(*args,**kwargs):
            # 真实 lookup_transform 找不到 TF 时抛 TransformException 族（节点
            # 据此保留消息重试）；裸 RuntimeError 不在节点捕获范围内。
            if not available[0]:raise TransformException('origin TF not yet available')
            return NS(transform=NS(translation=NS(x=3.,y=4.,z=0.),
                                  rotation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5))))
        app=adapter.GridMapperNode.__new__(adapter.GridMapperNode)
        app._params={'grid.source_frame':'lio_map','grid.inflation_radius_m':0.}
        app._output_frame='map';app._pending_external=None;app._grid_map=None
        app._tf_buffer=NS(lookup_transform=lookup)
        app._log=NS(warning=lambda text:None,
                    error=lambda *a:None,info=lambda *a:None)
        app._save_outputs=lambda:saved.append(True)
        app._pub_grid=NS(publish=outputs.append);app._pub_ray=NS(publish=outputs.append)
        app._source_stamp=None;app._source_load_time=None
        app.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Stamp(sec=99,nanosec=0)))
        stamp=Stamp(sec=12,nanosec=234567890)
        message=OccupancyGrid()
        message.header.frame_id='lio_map';message.header.stamp=stamp
        message.data=[-1,0,100,100,-1,0]
        message.info.width=3;message.info.height=2;message.info.resolution=.2
        message.info.map_load_time=stamp
        message.info.origin.position.x=2.;message.info.origin.position.y=-1.
        message.info.origin.orientation.w=1.
        app._on_external_map(message)
        self.assertEqual(saved,[]);self.assertEqual(outputs,[])
        self.assertEqual(app._pending_external,message)
        available[0]=True;app._retry_pending_external()
        self.assertEqual(len(saved),1);self.assertEqual(len(outputs),2)
        out=outputs[0]
        self.assertEqual(out.header.stamp,stamp)
        self.assertEqual(out.header.frame_id,'map')
        self.assertEqual(list(out.data),list(message.data))
        self.assertAlmostEqual(out.info.origin.position.x,4.)
        self.assertAlmostEqual(out.info.origin.position.y,6.)
        self.assertAlmostEqual(out.info.origin.orientation.z,math.sqrt(.5))
        self.assertIsNone(app._pending_external)


if __name__=='__main__':
    unittest.main(verbosity=2)
