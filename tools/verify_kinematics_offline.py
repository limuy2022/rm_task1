"""Full 3-D gimbal lever arms and same-time inversion; no hardware/ROS process."""
import math
import sys
import unittest
import types
from types import SimpleNamespace as NS
from unittest.mock import patch
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for pkg in ('gimbal_common', 'gimbal_localization'):
    sys.path.insert(0, str(ROOT/'src'/pkg))
from gimbal_common import frames as FR
from gimbal_localization.gimbal_kinematics import GimbalKinematics, SynchronizedKinematics, quaternion_rotation


def calibration():
    # Synthetic test fixture only. These are NOT measured team extrinsics.
    return dict(calibration_confirmed=True, joint_zero_known=True, mounted_on_base_confirmed=True,
        base_mount_xyz=[.21, -.08, .35], base_mount_rpy=[.03, -.04, .12],
        yaw_to_pitch_xyz=[.03, .02, .11], pitch_gimbal_xyz=[-.02, .01, .04],
        pitch_gimbal_rpy=[.08, -.02, .04], gimbal_camera_xyz=[.06, -.025, .04],
        gimbal_camera_rpy=[.025, -.03, .05], odom_world_rpy=[.1, -.08, .6],
        joint_zero_rad=[.13, -.04], yaw_limits_rad=[-2.8, 2.8], pitch_limits_rad=[-.5, .35])


class KinematicsTests(unittest.TestCase):
    def test_actual_node_defaults_and_full_stamped_tf_reset_gate(self):
        from verify_timing_offline import FakeNode, load_ast, transform, odom, quat, stamp
        from gimbal_common.time_buffer import stamp_seconds
        class Broadcaster:
            def __init__(self, node): self.messages = []
            def sendTransform(self, messages):
                self.messages.extend(messages if isinstance(messages, list) else [messages])
        class Base(FakeNode): overrides = {}
        env = load_ast('src/gimbal_localization/gimbal_localization/gimbal_kinematics_node.py',
            {'DEFAULTS','GimbalKinematicsNode'},dict(Node=Base, np=np, FR=FR,
                GimbalKinematics=GimbalKinematics,SynchronizedKinematics=SynchronizedKinematics,
                quaternion_rotation=quaternion_rotation,stamp_seconds=stamp_seconds,
                TransformBroadcaster=Broadcaster,StaticTransformBroadcaster=Broadcaster,
                TransformStamped=transform,JointState=lambda:NS(header=NS()),
                Odometry=object,Imu=object,UInt64=object,QoSProfile=lambda **kw:kw,
                ReliabilityPolicy=NS(BEST_EFFORT=0,RELIABLE=1),DurabilityPolicy=NS(TRANSIENT_LOCAL=1)))
        disabled = env['GimbalKinematicsNode'](); disabled.on_epoch(NS(data=7));disabled.tick()
        self.assertFalse(disabled.pub.messages); self.assertFalse(disabled.dynamic.messages)
        Base.overrides = {'kinematics.'+k:v for k,v in calibration().items()}
        Base.overrides['kinematics.calibrated_epoch']=7
        app = env['GimbalKinematicsNode'](); app.on_epoch(NS(data=7))
        model = GimbalKinematics(calibration()); R_ob = FR.rpy_to_mat([.1, -.1, .8])
        app.on_base(odom(1000.,(1.,2.,.3),(.1,-.1,.8)))
        chain = model.forward(.3,-.2)
        R_wg = model.R_odom_world.T@R_ob@chain.base_gimbal[:3,:3]
        msg = NS(header=NS(stamp=stamp(1000.),frame_id='gimbal_link'),
                 orientation=quat(R_wg),orientation_covariance=[0.]*9)
        app.on_imu(msg)
        fake = types.ModuleType('builtin_interfaces.msg')
        fake.Time=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'builtin_interfaces':types.ModuleType('builtin_interfaces'),
                                    'builtin_interfaces.msg':fake}): app.tick()
        self.assertEqual(stamp_seconds(app.pub.messages[-1].header.stamp),1000.)
        np.testing.assert_allclose(app.pub.messages[-1].position,[.3,-.2])
        self.assertEqual([(x.header.frame_id,x.child_frame_id) for x in app.static.messages],
                         [('gimbal_link','camera_link'),('camera_link','camera_optical_frame')])
        self.assertEqual([(x.header.frame_id,x.child_frame_id) for x in app.dynamic.messages],
                         [('base_link','gimbal_link')])
        app.on_epoch(NS(data=8)); app.on_imu(msg); app.tick()
        self.assertEqual(len(app.pub.messages),1)

    def test_forward_inverse_full_attitude_and_nonzero_zero(self):
        model = GimbalKinematics(calibration())
        R_ob = FR.rpy_to_mat([.19, -.12, 1.8])
        for y in np.linspace(-2.7, 2.7, 13):
            for p in (-.45, -.1, .3):
                out = model.forward(y, p)
                R_wg = model.R_odom_world.T @ R_ob @ out.base_gimbal[:3, :3]
                joints = model.inverse(R_ob, R_wg)
                np.testing.assert_allclose([joints.yaw, joints.pitch], [y, p], atol=1e-12)

    def test_optical_axes_and_full_lever_arms(self):
        model = GimbalKinematics(calibration())
        a = model.forward(0, 0); b = model.forward(.8, -.3)
        np.testing.assert_array_equal(a.camera_optical[:3, :3]@np.array([0, 0, 1]), [1, 0, 0])
        self.assertGreater(np.linalg.norm(a.base_camera[:3, 3]-b.base_camera[:3, 3]), .04)
        np.testing.assert_allclose(a.base_optical, a.base_gimbal@a.gimbal_camera@a.camera_optical)
        point = np.array([.1, .2, 3., 1.])
        np.testing.assert_allclose(np.linalg.inv(b.base_optical)@(b.base_optical@point), point)

    def test_missing_confirmation_zero_or_mount_rejects(self):
        for flag in ('calibration_confirmed', 'joint_zero_known', 'mounted_on_base_confirmed'):
            c = calibration(); c[flag] = False
            with self.assertRaises(ValueError): GimbalKinematics(c)
        c = calibration(); del c['gimbal_camera_xyz']
        with self.assertRaises(ValueError): GimbalKinematics(c)

    def test_wrong_imu_roll_and_world_alignment_are_rejected(self):
        model = GimbalKinematics(calibration()); R_ob = FR.rpy_to_mat([.1, -.1, .8])
        truth = model.forward(.2, .1)
        R_wg = model.R_odom_world.T@R_ob@truth.base_gimbal[:3, :3]
        with self.assertRaises(ValueError):
            model.inverse(R_ob, R_wg@FR.rpy_to_mat([.4, 0, 0]))

    def test_limit_branch_and_vertical_singularity(self):
        model = GimbalKinematics(calibration())
        with self.assertRaises(ValueError): model.forward(3, 0)
        c = calibration(); c['yaw_limits_rad'] = [-math.pi, math.pi]
        with self.assertRaises(ValueError): GimbalKinematics(c)
        c = calibration(); c['pitch_limits_rad'] = [-1.6, 1.6]
        with self.assertRaises(ValueError): GimbalKinematics(c)

    def test_sync_waits_for_bracket_and_preserves_imu_stamp(self):
        model = GimbalKinematics(calibration())
        sync = SynchronizedKinematics(model, calibrated_epoch=7)
        sync.observe_epoch(7)
        T_ob = FR.compose_T(FR.rpy_to_mat([.1, -.1, .8]), [2, 3, .1])
        R_wg = model.R_odom_world.T@T_ob[:3, :3]@model.forward(.3, -.2).base_gimbal[:3, :3]
        self.assertTrue(sync.add_base(1000., T_ob, 1000.04))
        self.assertTrue(sync.add_imu(1000.02, R_wg, 1000.04))
        self.assertFalse(sync.drain(1000.04))
        self.assertTrue(sync.add_base(1000.04, T_ob, 1000.04))
        values = sync.drain(1000.04); self.assertEqual(len(values), 1)
        self.assertEqual(values[0].stamp, 1000.02)
        np.testing.assert_allclose([values[0].joints.yaw, values[0].joints.pitch], [.3, -.2])
        self.assertFalse(sync.add_imu(1000.01, R_wg, 1000.05))

    def test_unmounted_missing_base_stale_or_epoch_fault_never_publish(self):
        model = GimbalKinematics(calibration())
        no_model = SynchronizedKinematics(calibrated_epoch=7); no_model.observe_epoch(7)
        self.assertFalse(no_model.enabled)
        sync = SynchronizedKinematics(model, calibrated_epoch=7); sync.observe_epoch(7)
        self.assertFalse(sync.add_imu(1, np.eye(3), 2))
        sync.add_imu(2., np.eye(3), 2.01)
        self.assertFalse(sync.drain(2.02))  # no fake table-top base
        self.assertFalse(sync.drain(2.3)); self.assertFalse(sync.pending)
        sync.observe_epoch(8); self.assertTrue(sync.fault)
        sync.observe_epoch(7); self.assertFalse(sync.enabled)  # latched; no silent recovery


if __name__ == '__main__': unittest.main(verbosity=2)
