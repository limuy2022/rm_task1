"""真实Humble消息与TF Buffer：点云跨坐标、无TF、坏包和过期保护。"""
import struct

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2,PointField

import team43_chassis_advisor.chassis_advisor_node as module


@pytest.fixture
def node(monkeypatch):
    rclpy.init(args=[])
    monkeypatch.setattr(module,'declare_params',lambda _,defaults:{**defaults,
        'chassis.obstacle_guard_enabled':True,'chassis.obstacle_floor_valid':True})
    monkeypatch.setattr(module,'now',lambda:100.)
    app = module.ChassisAdvisorNode()
    app._clock_seconds = lambda:100.
    app._base_xy = np.array([2.,3.])
    app._has_base = True
    yield app
    app.destroy_node();rclpy.shutdown()


def cloud(z=.5):
    msg = PointCloud2();msg.header.frame_id = 'lio_world';msg.header.stamp.sec = 100
    msg.height = 1;msg.width = 101;msg.point_step = 12;msg.row_step = 12*msg.width
    msg.fields = [PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
    msg.data = b''.join(struct.pack('<fff',.65,float(y),z) for y in np.linspace(-2.,2.,101))
    return msg


def transform(node):
    tf = TransformStamped();tf.header.frame_id = 'map';tf.child_frame_id = 'lio_world'
    tf.transform.translation.x = 2.;tf.transform.translation.y = 3.
    tf.transform.rotation.z = tf.transform.rotation.w = np.sqrt(.5)
    node._obstacle_tf.set_transform_static(tf,'test')


def test_rotated_lio_cloud_becomes_map_obstacle_and_expires(node):
    transform(node);node._on_obstacles(cloud())
    assert node._obstacle_points is not None
    np.testing.assert_allclose(node._obstacle_points[:,1],3.65,atol=1e-6)
    cmd,_,blocked = node._guard_command(np.array([0.,.3]),100.)
    assert not blocked
    path = node._base_xy+np.linspace(0.,1.5,101)[:,None]*cmd
    assert np.linalg.norm(node._obstacle_points[None,:,:]-path[:,None,:],axis=2).min() >= .4
    node._clock_seconds = lambda:100.6
    stopped,_,blocked = node._guard_command(np.array([0.,.3]),100.6)
    assert blocked;np.testing.assert_array_equal(stopped,[0.,0.])


def test_missing_tf_ground_only_and_corrupt_cloud_fail_closed(node):
    node._on_obstacles(cloud());assert node._obstacle_points is None
    transform(node)
    node._on_obstacles(cloud(z=0.));assert node._obstacle_points is None
    bad = cloud();bad.data = b'bad'
    node._on_obstacles(bad);assert node._obstacle_points is None
    bad = cloud();bad.header.frame_id = 'wrong_frame'
    node._on_obstacles(bad);assert node._obstacle_points is None
    cmd,_,blocked = node._guard_command(np.array([.3,0.]),100.)
    assert blocked;np.testing.assert_array_equal(cmd,[0.,0.])
