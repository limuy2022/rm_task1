#!/usr/bin/env python3
"""Offline control regression, using real algorithms and AST-loaded node methods.

Run: python tools/verify_control_offline.py
Requires numpy; no ROS, serial access, hardware or files are used. The ideal
velocity-actuator closed loops verify geometry and signs, not motor dynamics or
real-time ROS delivery. AST-loaded methods are unit tests, not a ROS emulator.
"""
from __future__ import annotations

import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ('gimbal_control', 'gimbal_common'):
    sys.path.insert(0, str(ROOT / 'src' / package))
from gimbal_common import frames
from gimbal_control.control_law import (
    BearingControlLaw, ControlLaw, feedforward, r_gimbal_from_opt,
)
from gimbal_control.search_pattern import make_small_search


def message_stamp(seconds, frame='map'):
    sec = int(math.floor(seconds))
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=int(round((seconds - sec) * 1e9))))


class FakeCommand:
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.yaw_omega = self.pitch_omega = 0.0
        self.valid = False


def node_harness():
    """Load exact selected node methods without importing unavailable ROS."""
    tree = ast.parse((ROOT / 'src/gimbal_control/gimbal_control/gimbal_controller_node.py').read_text())
    source = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GimbalControllerNode')
    selected = {'_source_time', '_source_age', 'on_target_state', '_mode_of_age',
                '_target_age', 'on_gimbal_state', 'on_joint_state', '_stop', 'tick'}
    cls = ast.ClassDef(name='NodeHarness', bases=[], keywords=[],
                      body=[n for n in source.body if isinstance(n, ast.FunctionDef)
                            and n.name in selected], decorator_list=[])
    module = ast.Module(body=[ast.ImportFrom(module='__future__',
                       names=[ast.alias(name='annotations')], level=0), cls], type_ignores=[])
    clock = NS(ros=10.0, mono=100.0)
    scope = {'np': np, 'math': math, 'frames': frames, 'now': lambda: clock.mono,
             'MODE_IDLE': 'idle', 'MODE_TRACK': 'track', 'MODE_COAST': 'coast',
             'MODE_SEARCH': 'search', 'MODE_WIDE': 'wide',
             'GimbalOmegaCmd': FakeCommand, 'throttled': lambda *args: None}
    exec(compile(ast.fix_missing_locations(module), '<actual-control-methods>', 'exec'), scope)
    node = scope['NodeHarness']()
    parameters = {'control.enabled': False, 'control.world_geometry_verified': False,
                  'control.search_enabled': False, 'control.joint_yaw_name': 'yaw',
                  'control.joint_pitch_name': 'pitch'}
    node.get_parameter = lambda name: NS(value=parameters[name])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(clock.ros * 1e9),
                                                 to_msg=lambda: message_stamp(clock.ros).stamp))
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
    node._pub = NS(publish=lambda msg: sent.append(msg))
    node._log = NS(warn=lambda *args: None)
    sent = []
    return node, clock, parameters, sent


class ControlRegressions(unittest.TestCase):
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
        ff = feedforward(r, velocity, [0, 1, 0])
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
        msg = NS(header=message_stamp(10), valid=True, track_status=2,
                 last_obs_age=.8, cur_x=2, cur_y=0, pred_x=2, pred_y=0, vel_x=0, vel_y=0)
        node.on_target_state(msg)
        self.assertAlmostEqual(node._target_age(clock.mono), .8)
        self.assertEqual(node._mode_of_age(.8), 'idle')
        self.assertFalse(node._have_ever_tracked)
        msg.last_obs_age = 0; msg.track_status = 1
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
        imu = NS(header=message_stamp(9.95, 'gimbal_link'), quat=[1, 0, 0, 0], gyro=[0, 0, 0])
        node.on_gimbal_state(imu)
        self.assertAlmostEqual(node._t_last_gimbal, 99.95)
        # Repeated old source stamp cannot be refreshed merely by another callback.
        clock.ros = 10.2; clock.mono = 100.2
        node.on_gimbal_state(imu)
        self.assertIsNone(node._gimbal_state)
        imu.header = message_stamp(10.2, 'gimbal_link'); imu.gyro = [float('nan'), 0, 0]
        node.on_gimbal_state(imu)
        self.assertIsNone(node._gimbal_state)
        joint = NS(header=message_stamp(10.2), name=['pitch', 'yaw'], position=[-.1, .2])
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
        target = NS(header=message_stamp(10, 'camera_optical_frame'), valid=True,
                    track_status=1, last_obs_age=0, cur_x=2, cur_y=0,
                    pred_x=2, pred_y=0, vel_x=0, vel_y=0)
        node.on_target_state(target)
        self.assertIsNone(node._target)
        imu = NS(header=message_stamp(10, 'world'), quat=[1, 0, 0, 0], gyro=[0, 0, 0])
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


if __name__ == '__main__':
    unittest.main(verbosity=2)
