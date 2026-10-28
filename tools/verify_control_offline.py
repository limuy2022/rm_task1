#!/usr/bin/env python3
"""Offline control regression against the real node classes and ROS messages.

Run: python tools/verify_control_offline.py
Requires a sourced ROS 2 Humble environment. Node constructors are bypassed
(``__new__``) and internal fields injected (controllable clocks, parameter
lookups, publisher capture); the executed callbacks and message classes are the
production code. No DDS traffic, serial access or hardware is used. The ideal
velocity-actuator closed loops verify geometry and signs, not motor dynamics.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ('team43_control', 'team43_common'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from builtin_interfaces.msg import Time as Stamp
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, UInt64

from team43_interfaces.msg import GimbalState, MarkerObservation, TargetState

from team43_common import frames
import team43_control.gimbal_bearing_node as bearing_module
import team43_control.gimbal_controller_node as controller_module
from team43_control.gimbal_bearing_node import GimbalBearingNode
from team43_control.gimbal_controller_node import GimbalControllerNode, HeightTracker
from team43_control.bearing_supervisor import BearingSupervisor
from team43_control.control_law import (
    BearingControlLaw, ControlLaw, feedforward, r_gimbal_from_opt,
)
from team43_control.search_pattern import make_small_search


def message_stamp(seconds, frame='map'):
    sec = int(math.floor(seconds))
    header = Header()
    header.stamp = Stamp(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))
    header.frame_id = frame
    return header


def marker_message(seconds, frame='camera_optical_frame', x=-.3, y=0., z=2.,
                   valid=True, marker_id=3):
    msg = MarkerObservation()
    msg.header = message_stamp(seconds, frame)
    msg.x, msg.y, msg.z = float(x), float(y), float(z)
    msg.valid = bool(valid)
    msg.marker_id = marker_id
    return msg


def ros_clock_view(clock):
    """Injection view of node.get_clock(): ROS seconds come from ``clock.ros``."""
    return NS(now=lambda: NS(nanoseconds=int(clock.ros * 1e9),
                             to_msg=lambda: message_stamp(clock.ros).stamp))


def node_harness():
    """Real GimbalControllerNode; fields injected exactly as the node sets them."""
    clock = NS(ros=10.0, mono=100.0)
    controller_module.now = lambda: clock.mono   # module-level monotonic clock mock
    node = GimbalControllerNode.__new__(GimbalControllerNode)
    parameters = {'control.enabled': False, 'control.world_geometry_verified': False,
                  'control.search_enabled': False, 'control.joint_yaw_name': 'yaw',
                  'control.joint_pitch_name': 'pitch'}
    node.get_parameter = lambda name: NS(value=parameters[name])
    node.get_clock = lambda: ros_clock_view(clock)
    node._input_timeout_s = 0.15
    node._imu_timeout_s = 0.1
    node._lost_timeout_s = 0.3
    node._search_after_s = 0.5
    node._wide_after_s = 5.0
    node._target = None
    node._t_last_target = None
    node._t_start = 100.0
    node._have_ever_tracked = False
    node._law = ControlLaw()
    node._t_last_cmd = None
    node._search_entry_t = None
    node._joint_angles = None
    sent = []
    node._pub = NS(publish=lambda msg: sent.append(msg))
    node._log = NS(info=lambda *a: None, warning=lambda *a: None,
                   error=lambda *a: None)
    return node, clock, parameters, sent


class HeightTrackerRegressions(unittest.TestCase):
    def test_constant_vertical_speed_has_no_low_pass_lag_gain(self):
        for speed in (.2, -.35):
            for dt in (.01, .02, .05):
                with self.subTest(speed=speed, sample_interval=dt):
                    height = HeightTracker(cutoff_hz=8.)
                    for i in range(501):
                        elapsed = i * dt
                        height.update(3. + speed * elapsed, 100. + elapsed)
                    self.assertAlmostEqual(height.vz, speed, places=9)

    def test_static_height_has_no_velocity_and_moving_target_can_stop(self):
        height = HeightTracker()
        for i in range(100):
            height.update(1.5, 100. + i * .02)
        self.assertEqual(height.height(), 1.5)
        self.assertEqual(height.vz, 0.)
        for i in range(101):
            height.update(1.5 + .2 * i * .02, 102. + i * .02)
        self.assertAlmostEqual(height.vz, .2, places=9)
        for i in range(1, 101):
            height.update(1.9, 104. + i * .02)
        self.assertAlmostEqual(height.height(), 1.9, places=9)
        self.assertAlmostEqual(height.vz, 0., places=9)

    def test_reset_and_first_sample_do_not_reuse_old_velocity(self):
        height = HeightTracker()
        self.assertEqual(height.height(default=7.), 7.)
        height.update(1., 100.)
        self.assertEqual((height.height(), height.vz), (1., 0.))
        height.update(2., 100.02)
        # The existing first-order height response is retained, independently
        # of how its raw observation derivative is now computed.
        expected_height = 2. - math.exp(-2. * math.pi * 8. * .02)
        self.assertAlmostEqual(height.height(), expected_height)
        self.assertGreater(height.vz, 0.)
        height.reset()
        self.assertIsNone(height.z)
        self.assertEqual(height.vz, 0.)
        height.update(-4., 10.)  # A new acquisition may have a new time origin.
        self.assertEqual((height.height(), height.vz), (-4., 0.))
        height.update(-4., 10.02)
        self.assertEqual(height.vz, 0.)

    def test_invalid_samples_do_not_move_the_valid_difference_baseline(self):
        height = HeightTracker()
        for z, t in ((1., float('nan')), (1., float('inf')), (float('nan'), 100.)):
            height.update(z, t)
            self.assertIsNone(height.z)
        height.update(1., 100.)
        height.update(1.1, 100.1)
        before = height.height(), height.vz
        for z, t in ((999., 100.1), (999., 99.), (999., float('nan')),
                     (999., float('inf')), (float('nan'), 100.2), (float('inf'), 100.2)):
            height.update(z, t)
            self.assertEqual((height.height(), height.vz), before)
        height.update(1.2, 100.2)
        self.assertGreater(height.height(), before[0])
        self.assertLessEqual(height.height(), 1.2)
        self.assertAlmostEqual(height.vz, 1., places=4)


class ControlRegressions(unittest.TestCase):
    def test_pd_derivative_uses_measured_minus_sight_line_rate(self):
        # A moving sight line already contributes through D even if the
        # separately configured feedforward gain is zero.
        angle = math.radians(10.)
        target = np.array([5. * math.cos(angle), 5. * math.sin(angle), 0.])
        velocity = .1 * np.array([-target[1], target[0], 0.])
        for ff_gain in (0., 1.):
            with self.subTest(ff_gain=ff_gain):
                law = ControlLaw(kp_yaw=6., kd_yaw=.5, ff_gain=ff_gain)
                out = law.compute(target, velocity, [1., 0., 0., 0.], [0., 0., .04],
                                  dt=.1, joint_angles=(0., 0.))
                self.assertTrue(out.valid)
                self.assertAlmostEqual(out.ff_yaw, .1)
                expected = ff_gain * .1 + 6. * angle - .5 * (.04 - .1)
                self.assertAlmostEqual(out.yaw_omega, expected)
                self.assertEqual(out.pitch_omega, 0.)

    def test_camera_bearing_signs(self):
        for point, yaw_sign, pitch_sign in (((-0.2, -0.1, 2), 1, -1),
                                            ((0.2, 0.1, 2), -1, 1)):
            out = BearingControlLaw().compute_bearing(
                point, joint_angles=(0, 0), R_gimbal_optical=r_gimbal_from_opt(),
                geometry_verified=True, enabled=True, dt=.1)
            self.assertTrue(out.valid)
            self.assertGreater(yaw_sign * out.yaw_omega, 0)
            self.assertGreater(pitch_sign * out.pitch_omega, 0)

    def test_unknown_joint_angles_do_not_become_world_angles(self):
        q = frames.quat_from_mat(frames.compose_yaw_pitch(3.0, 0))
        target = [5 * math.cos(3.1), 5 * math.sin(3.1), 0]
        law = ControlLaw(kd_yaw=0, kd_pitch=0)
        out = law.compute(target, [0, 0, 0], q, [0, 0, 0], dt=.1)
        self.assertFalse(out.valid)
        self.assertEqual((out.yaw_omega, out.pitch_omega), (0, 0))
        out = law.compute(target, [0, 0, 0], q, [0, 0, 0], dt=.1, joint_angles=(0, 0))
        self.assertTrue(out.valid)
        self.assertGreater(out.yaw_omega, 0)  # world yaw>mechanical maximum must not attenuate
        out = law.compute(target, [0, 0, 0], q, [0, 0, 0], dt=.1,
                          joint_angles=(law.yaw_max, 0))
        self.assertEqual(out.yaw_omega, 0)  # real mechanical limit must attenuate

    def test_relative_translation_and_base_rotation_feedforward(self):
        law = ControlLaw(kd_yaw=0, kd_pitch=0)
        args = ([2, 0, 0], [0, 0, 0], [1, 0, 0, 0], [0, 0, 0])
        out = law.compute(*args, dt=.1, joint_angles=(0, 0), origin_velocity_world=(0, 1, 0))
        self.assertAlmostEqual(out.ff_yaw, -.5)
        law.reset()
        out = law.compute([2, 0, 0], [0, 1, 0], [1, 0, 0, 0], [0, 0, 0], dt=.1,
                          joint_angles=(0, 0), origin_velocity_world=(0, 1, 0))
        self.assertAlmostEqual(out.ff_yaw, 0)
        out = law.compute(*args, dt=.1, joint_angles=(0, 0), base_angular_velocity_world=(0, 0, .4))
        self.assertAlmostEqual(out.yaw_omega, -.4)

    def test_sight_line_rate_matches_finite_difference_off_horizon(self):
        r = np.array([1.5, .5, 1.2]); velocity = np.array([-.2, .4, .3]); eps = 1e-6
        ff = feedforward(r, velocity)
        def angles(point):
            return np.array([math.atan2(point[1], point[0]),
                             -math.atan2(point[2], math.hypot(point[0], point[1]))])
        expected = (angles(r + eps * velocity) - angles(r - eps * velocity)) / (2 * eps)
        np.testing.assert_allclose(ff, expected, atol=1e-8)

    def test_bearing_closed_loop_with_rotating_base(self):
        law = BearingControlLaw()
        rotation = r_gimbal_from_opt()
        target = np.array([3., .6, .3]); joints = np.zeros(2)
        base_rate = .2; dt = .005
        first_error = None
        for i in range(800):
            R_wg = frames.compose_yaw_pitch(base_rate * i * dt + joints[0], joints[1])
            marker_optical = rotation.T @ R_wg.T @ target
            out = law.compute_bearing(marker_optical, joint_angles=joints,
                                      R_gimbal_optical=rotation, enabled=True,
                                      geometry_verified=True, dt=dt,
                                      base_omega_gimbal=R_wg.T @ [0, 0, base_rate])
            self.assertTrue(out.valid)
            error = math.hypot(math.atan2(marker_optical[0], marker_optical[2]),
                               math.atan2(marker_optical[1], marker_optical[2]))
            first_error = error if first_error is None else first_error
            joints += dt * np.array([out.yaw_omega, out.pitch_omega])
        self.assertLess(error, min(.002, first_error / 50))
        self.assertLess(joints[0], -.4)
        self.assertLess(joints[1], 0)

    def test_world_closed_loop_with_rotating_base(self):
        law = ControlLaw()
        target = [5, 0, 0]; joint_yaw = 0.0; yaw_rate = 0.0
        base_rate = .2; dt = .005
        for i in range(800):
            total_yaw = base_rate * i * dt + joint_yaw
            q = frames.quat_from_mat(frames.compose_yaw_pitch(total_yaw, 0))
            out = law.compute(target, [0, 0, 0], q, [0, 0, base_rate + yaw_rate], dt,
                              joint_angles=(joint_yaw, 0), base_angular_velocity_world=(0, 0, base_rate))
            self.assertTrue(out.valid)
            yaw_rate = out.yaw_omega
            joint_yaw += dt * yaw_rate
        self.assertLess(abs(out.e_yaw), .002)
        self.assertAlmostEqual(yaw_rate, -base_rate, places=3)

    def test_bearing_failure_inputs_stop_and_clear_slew(self):
        law = BearingControlLaw()
        good = dict(joint_angles=(0, 0), R_gimbal_optical=r_gimbal_from_opt(),
                    geometry_verified=True, enabled=True, dt=.01)
        for patch in ({'enabled': False}, {'geometry_verified': False}, {'joint_angles': None},
                      {'observation_age': .2}, {'observation_age': float('nan')},
                      {'dt': 0}, {'R_gimbal_optical': np.eye(3) * 2}):
            law.compute_bearing((-.2, -.1, 2), **good)
            out = law.compute_bearing((-.2, -.1, 2), **dict(good, **patch))
            self.assertFalse(out.valid)
            self.assertEqual(law.omega_prev, (0, 0))
        for point in ((0, 0, -2), (float('nan'), 0, 2), (0, 0, 0)):
            self.assertFalse(law.compute_bearing(point, **good).valid)
        # both gates are closed by default even if an explicit rotation is supplied
        self.assertFalse(law.compute_bearing((0, 0, 2), joint_angles=(0, 0),
                                             R_gimbal_optical=r_gimbal_from_opt()).valid)

    def test_bearing_feedforward_signs_and_scaling(self):
        # Translational feedforward is divided by distance; the base-rotation
        # term is not (both sides of the Jacobian are rad/s).
        law = BearingControlLaw(kp_yaw=0, kp_pitch=0, ff_gain=1)
        common = dict(joint_angles=(0, 0), R_gimbal_optical=r_gimbal_from_opt(),
                      geometry_verified=True, enabled=True, dt=.01)
        out = law.compute_bearing((0, 0, 2), target_relative_velocity_gimbal=(0, 0, -.5), **common)
        self.assertAlmostEqual(out.ff_pitch, .5 / 2)   # target descending -> head down
        out = law.compute_bearing((0, 0, 2), target_relative_velocity_gimbal=(0, .4, 0), **common)
        self.assertAlmostEqual(out.ff_yaw, .4 / 2)     # target moving left -> yaw left
        out = law.compute_bearing((0, 0, 2), base_omega_gimbal=(0, 0, .3), **common)
        self.assertAlmostEqual(out.ff_yaw, -.3)        # base turning left -> counter right

    def test_bearing_near_singular_pitch_rejected(self):
        # min singular value of the joint Jacobian is |cos(pitch)|.
        law = BearingControlLaw()
        common = dict(R_gimbal_optical=r_gimbal_from_opt(),
                      geometry_verified=True, enabled=True)
        self.assertTrue(law.compute_bearing((.5, 0, 2), joint_angles=(0, 1.2), **common).valid)
        self.assertFalse(law.compute_bearing((.5, 0, 2), joint_angles=(0, 1.4), **common).valid)
        self.assertFalse(law.compute_bearing((.5, 0, 2), joint_angles=(0, -1.4), **common).valid)

    def test_slew_magnitude_and_real_joint_limits(self):
        law = BearingControlLaw()
        out = law.compute_bearing((-1, -1, 2), joint_angles=(0, 0),
                                  R_gimbal_optical=r_gimbal_from_opt(), enabled=True,
                                  geometry_verified=True, dt=.01)
        self.assertLessEqual(abs(out.yaw_omega), .15 + 1e-12)
        self.assertLessEqual(abs(out.pitch_omega), .15 + 1e-12)
        out = law.compute_bearing((-1, -1, 2), joint_angles=(law.yaw_max, law.pitch_min),
                                  R_gimbal_optical=r_gimbal_from_opt(), enabled=True,
                                  geometry_verified=True, dt=.1)
        self.assertEqual((out.yaw_omega, out.pitch_omega), (0, 0))

    def test_unknown_fov_and_bad_search_time_do_not_start_scan(self):
        self.assertEqual(make_small_search().sample(.1), (0, 0))
        known = make_small_search(half_fov_rad=.38)
        self.assertEqual(known.sample(float('nan')), (0, 0))
        self.assertEqual(known.sample(-1), (0, 0))

    def test_estimator_heartbeat_does_not_renew_old_observation(self):
        node, clock, params, _ = node_harness()
        msg = TargetState()
        msg.header = message_stamp(10)
        msg.valid = True; msg.track_status = 2; msg.last_obs_age = .8
        msg.cur_x, msg.cur_y = 2., 0.
        msg.pred_x, msg.pred_y = 2., 0.
        msg.vel_x, msg.vel_y = 0., 0.
        node.on_target_state(msg)
        self.assertAlmostEqual(node._target_age(clock.mono), .8)
        self.assertEqual(node._mode_of_age(.8), 'idle')
        self.assertFalse(node._have_ever_tracked)
        msg.last_obs_age = 0.; msg.track_status = 1
        node.on_target_state(msg)
        self.assertEqual(node._mode_of_age(0), 'track')
        msg.valid = False
        node.on_target_state(msg)
        self.assertEqual(node._mode_of_age(0), 'idle')
        clock.ros += 1; clock.mono += 1
        node.on_target_state(msg)
        self.assertIsNone(node._target)
        params['control.search_enabled'] = True
        self.assertEqual(node._mode_of_age(1), 'search')  # only after explicit opt-in + prior track

    def test_stamped_imu_and_joint_inputs(self):
        node, clock, _, _ = node_harness()
        imu = GimbalState()
        imu.header = message_stamp(9.95, 'gimbal_link')
        imu.quat = [1., 0., 0., 0.]; imu.gyro = [0., 0., 0.]
        node.on_gimbal_state(imu)
        self.assertAlmostEqual(node._t_last_gimbal, 99.95)
        # Repeated old source stamp cannot be refreshed merely by another callback.
        clock.ros = 10.2; clock.mono = 100.2
        node.on_gimbal_state(imu)
        self.assertIsNone(node._gimbal_state)
        imu.header = message_stamp(10.2, 'gimbal_link'); imu.gyro = [float('nan'), 0., 0.]
        node.on_gimbal_state(imu)
        self.assertIsNone(node._gimbal_state)
        joint = JointState()
        joint.header = message_stamp(10.2); joint.name = ['pitch', 'yaw']
        joint.position = [-.1, .2]
        node.on_joint_state(joint)
        np.testing.assert_allclose(node._joint_angles, [.2, -.1])
        joint.name = ['yaw', 'yaw']
        node.on_joint_state(joint)
        self.assertIsNone(node._joint_angles)
        joint.name = ['yaw', 'pitch']
        joint.header = message_stamp(9.0)
        node.on_joint_state(joint)
        self.assertIsNone(node._joint_angles)

    def test_wrong_target_or_imu_frame_is_rejected(self):
        node, _, _, _ = node_harness()
        target = TargetState()
        target.header = message_stamp(10, 'camera_optical_frame')
        target.valid = True; target.track_status = 1; target.last_obs_age = 0.
        target.cur_x, target.cur_y = 2., 0.
        target.pred_x, target.pred_y = 2., 0.
        target.vel_x, target.vel_y = 0., 0.
        node.on_target_state(target)
        self.assertIsNone(node._target)
        imu = GimbalState()
        imu.header = message_stamp(10, 'world')
        imu.quat = [1., 0., 0., 0.]; imu.gyro = [0., 0., 0.]
        node.on_gimbal_state(imu)
        self.assertIsNone(node._gimbal_state)

    def test_node_disabled_or_unknown_joints_never_valid(self):
        node, _, params, sent = node_harness()
        node._law._omega_yaw_prev = 1.0
        node.tick()
        self.assertEqual(len(sent), 1)
        self.assertFalse(sent[-1].valid)
        self.assertEqual(node._law.omega_prev, (0, 0))
        params.update({'control.enabled': True, 'control.world_geometry_verified': True})
        node.tick()
        self.assertFalse(sent[-1].valid)
        self.assertEqual((sent[-1].yaw_omega, sent[-1].pitch_omega), (0, 0))


def bearing_node_harness():
    """Real GimbalBearingNode; fields injected exactly as the node sets them."""
    clock = NS(ros=100.0, mono=200.0)
    bearing_module.now = lambda: clock.mono    # module-level monotonic clock mock
    node = GimbalBearingNode.__new__(GimbalBearingNode)
    parameters = {'control.bearing.enabled': False,
                  'control.bearing.geometry_verified': False,
                  'control.joint_yaw_name': 'yaw',
                  'control.joint_pitch_name': 'pitch'}
    node.get_parameter = lambda name: NS(value=parameters[name])
    node.get_clock = lambda: ros_clock_view(clock)
    node._sup = BearingSupervisor(
        BearingControlLaw(kp_yaw=6., kp_pitch=6., ff_gain=0.),
        max_age_s=.15, joint_timeout_s=.2, default_dt_s=.02)
    node._prev_enabled = False
    sent = []
    node._pub = NS(publish=lambda msg: sent.append(msg))
    node._log = NS(info=lambda *a: None, warning=lambda *a: None,
                   error=lambda *a: None)
    return node, clock, parameters, sent


def joint_message(seconds, names, positions):
    joint = JointState()
    joint.header = message_stamp(seconds)
    joint.name = list(names)
    joint.position = [float(p) for p in positions]
    return joint


class BearingNodeRegressions(unittest.TestCase):
    def test_node_disabled_by_default_and_gates_inputs(self):
        node, clock, params, sent = bearing_node_harness()
        node.on_marker(marker_message(99.95))
        node.on_joint_state(joint_message(99.96, ['yaw', 'pitch'], [0., 0.]))
        node.tick()
        self.assertEqual(len(sent), 1)
        self.assertFalse(sent[-1].valid)                  # control.bearing.enabled=False
        self.assertEqual((sent[-1].yaw_omega, sent[-1].pitch_omega), (0., 0.))
        params.update({'control.bearing.enabled': True})
        node.tick()
        self.assertFalse(sent[-1].valid)                  # geometry_verified still False
        params['control.bearing.geometry_verified'] = True
        node.tick()
        self.assertTrue(sent[-1].valid)                   # fresh marker + joints -> command
        self.assertGreater(sent[-1].yaw_omega, 0)         # marker left -> turn left

    def test_node_rejects_wrong_marker_frame_and_bad_joints(self):
        node, clock, params, sent = bearing_node_harness()
        params.update({'control.bearing.enabled': True,
                       'control.bearing.geometry_verified': True})
        node.on_marker(marker_message(99.95, frame='map'))
        node.on_joint_state(joint_message(99.96, ['yaw', 'yaw'], [0., 0.]))
        node.tick()
        self.assertFalse(sent[-1].valid)                  # frame/关节名不符 -> 停止
        # 修正输入后恢复
        node.on_marker(marker_message(99.97))
        node.on_joint_state(joint_message(99.98, ['yaw', 'pitch'], [0., 0.]))
        node.tick()
        self.assertTrue(sent[-1].valid)

    def test_node_stops_on_stale_marker_and_epoch_reset(self):
        node, clock, params, sent = bearing_node_harness()
        params.update({'control.bearing.enabled': True,
                       'control.bearing.geometry_verified': True})
        node.on_marker(marker_message(99.95))
        node.on_joint_state(joint_message(99.96, ['yaw', 'pitch'], [0., 0.]))
        node.on_epoch(UInt64(data=5))
        node.tick()
        self.assertTrue(sent[-1].valid)
        # 观测断流 0.5s：源龄超 max_age -> invalid 零指令（保活帧仍在发）
        clock.ros += .5; clock.mono += .5
        node.tick()
        self.assertFalse(sent[-1].valid)
        # 恢复观测，但 epoch 复位 -> fault
        node.on_marker(marker_message(clock.ros - .02))
        node.on_joint_state(joint_message(clock.ros - .01, ['yaw', 'pitch'], [0., 0.]))
        node.on_epoch(UInt64(data=6))
        node.tick()
        self.assertFalse(sent[-1].valid)
        self.assertTrue(node._sup.fault)
        # enabled 关→开（上升沿 rearm）后，锁存新 epoch 并恢复输出
        params['control.bearing.enabled'] = False
        node.tick()
        params['control.bearing.enabled'] = True
        node.tick()
        self.assertFalse(node._sup.fault)
        # fault 时观测已被清空：重新喂新鲜观测/关节后恢复
        node.on_marker(marker_message(clock.ros - .02))
        node.on_joint_state(joint_message(clock.ros - .01, ['yaw', 'pitch'], [0., 0.]))
        node.on_epoch(UInt64(data=6))                     # 与新上膛值相同：不再 fault
        node.tick()
        self.assertTrue(sent[-1].valid)


if __name__ == '__main__':
    unittest.main(verbosity=2)
