"""Production time/TF adapters on real ROS Humble node classes and messages.

Node constructors are bypassed via ``__new__`` and their internal fields are
injected (controllable clock, parameter lookups, capture publishers); every
executed method and every message class is the real production code. No DDS
traffic is created and no spin loop is entered.
"""
import copy
import math
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for pkg in ('team43_common', 'team43_estimation', 'team43_judge_io', 'team43_localization'):
    sys.path.insert(0, str(ROOT/'src'/pkg))
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformException

from team43_interfaces.msg import ChassisAdvice, MarkerObservation, TargetState

from team43_common import frames as FR
from team43_common.time_buffer import TimeBuffer, PoseHistory, stamp_seconds
from team43_estimation.ca_kalman import CAKalman
from team43_estimation.observation_builder import ObservationBuilder
from team43_estimation.stamped_tracker import StampedTracker
from team43_judge_io import judge_io_node as judge_module
from team43_judge_io.judge_io_node import JudgeIoNode
from team43_judge_io.tick_csv import TickCsvWriter
from team43_judge_io.tick_sampler import TickSampler
from team43_localization.map_frame_broadcaster_node import (
    PARAM_DEFAULTS as MAP_PARAM_DEFAULTS,
    MapFrameBroadcasterNode,
    _xyz_rpy_to_T,
)
from team43_estimation.target_estimator_node import TargetEstimatorNode


def stamp(t):
    ns = round(t*1e9)
    return Stamp(sec=ns//1000000000, nanosec=ns%1000000000)


def vec(values=(0., 0., 0.)):
    return Vector3(**dict(zip(('x', 'y', 'z'), map(float, values))))


def quat(R=None):
    q = FR.quat_from_mat(np.eye(3) if R is None else R)
    return Quaternion(**dict(zip(('w', 'x', 'y', 'z'), map(float, q))))


def transform(T=None):
    T = np.eye(4) if T is None else T
    msg = TransformStamped()
    msg.header.stamp = stamp(0); msg.header.frame_id = ''; msg.child_frame_id = ''
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = \
        map(float, T[:3, 3])
    msg.transform.rotation = quat(T[:3, :3])
    return msg


def odom(t=0., xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    msg = Odometry()
    msg.header.stamp = stamp(t); msg.header.frame_id = 'odom'; msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, xyz)
    msg.pose.pose.orientation = quat(FR.rpy_to_mat(rpy))
    msg.pose.covariance = list(map(float, np.diag([1., 2., 3., 4., 5., 6.]).flat))
    msg.twist.twist.linear = vec((.2, .1, .05))
    msg.twist.twist.angular = vec((.01, .02, .03))
    msg.twist.covariance = [0.]*36
    return msg


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))


class MutableClock:
    """Controllable ROS-clock injection: ``clock.now`` is seconds of ROS time."""

    def __init__(self, now=1000.04): self.now = now

    def get_clock(self):
        clock = self
        return NS(now=lambda: NS(nanoseconds=round(clock.now*1e9),
                                 to_msg=lambda: stamp(clock.now)))


LOG = NS(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)


class TfBroadcaster:
    def __init__(self, node=None): self.messages = []
    def sendTransform(self, msg): self.messages.append(copy.deepcopy(msg))


def map_node():
    """Real MapFrameBroadcasterNode with its constructor-time state injected."""
    clock = MutableClock(1000.04)
    app = MapFrameBroadcasterNode.__new__(MapFrameBroadcasterNode)
    app._node = clock
    app._log = LOG
    app._params = dict(MAP_PARAM_DEFAULTS)
    app._origin_mode = str(app._params['localization.origin_mode'])
    app._history = PoseHistory(capacity=2000)
    app._frozen = False
    app._clock_fault = False
    app._T_map_odom = None
    app._T_map_world = None
    app._origin_stamp = None
    app._replay_not_before = 1000.
    app._last_msg = None
    app._last_published_stamp = None
    app._t_first_valid = None
    app._n_pose = 0
    app._freeze_requested = False
    app._T_world_odom = _xyz_rpy_to_T(app._params['localization.world_odom_xyz_rpy'])
    app._world_confirmed = bool(app._params['localization.world_alignment_confirmed'])
    app._publish_map_odom = bool(app._params['localization.publish_map_odom'])
    app._pub_t = Publisher()
    app._pub_pose = Publisher()
    app._static_tf = TfBroadcaster()
    return clock, app


class FakeTfBuffer:
    """TF lookup injection: records requests, can raise the real TransformException."""

    def __init__(self):
        self.calls = []; self.missing = False; self.T = np.eye(4)

    def lookup_transform(self, target, source, stamp, **kwargs):
        self.calls.append((target, source, stamp_seconds(stamp.to_msg())))
        if self.missing: raise TransformException('no transform')
        return transform(self.T)


def target_node(confirmed=True):
    """Real TargetEstimatorNode with constructor-time state injected."""
    clock = MutableClock(1000.04)
    app = TargetEstimatorNode.__new__(TargetEstimatorNode)
    app._log = LOG
    app._frame = 'map'
    app._camera_frame = 'camera_optical_frame'
    app._horizon = .1
    kf = CAKalman(**{key: value for key, value in (
        ('jerk_std', 1.), ('max_dt', .5), ('nis_gate', 16.81), ('reinit_jump_m', 1.),
        ('p_max_trace', 100.), ('init_sigma_p', .05), ('init_sigma_v', 1.),
        ('init_sigma_a', 5.), ('cv_degrade_enable', False), ('cv_hold_s', .5))})
    app._tracker = StampedTracker(kf=kf, max_observation_delay=.3,
                                  lost_timeout=.5, fresh_timeout=.1)
    app._builder = ObservationBuilder(r_min_sigma=.002)
    app._tf_buffer = FakeTfBuffer()
    app._pending = []
    app._epoch = None
    app._last_now = None
    app._n_observed = 0
    app._n_rejected = 0
    app._pub = Publisher()
    params = {'estimation.geometry_confirmed': confirmed}
    app.get_parameter = lambda name: NS(value=params[name])
    app.get_clock = clock.get_clock
    return clock, app


def judge_node(output_dir):
    """Real JudgeIoNode with paths/writer/sampler wired to a temp directory."""
    app = JudgeIoNode.__new__(JudgeIoNode)
    app._log = LOG
    app.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1000200000000))
    params = dict(judge_module.PARAM_DEFAULTS)
    params.update({'judge.output_dir': output_dir, 'judge.enforce_v_floor': False})
    app._params = params
    out_dir = os.path.expanduser(str(params['judge.output_dir']))
    team = str(params['judge.team_id'])
    os.makedirs(out_dir, exist_ok=True)
    app._csv_path = os.path.join(out_dir, '%s.csv' % team)
    app._map_pgm = os.path.join(out_dir, '%s_map.pgm' % team)
    app._map_yaml = os.path.join(out_dir, '%s_map.yaml' % team)
    app._map_image_name = '%s_map.pgm' % team
    app._writer = TickCsvWriter(app._csv_path, logger=app._log)
    assert app._writer.open(), 'CSV writer must open on a fresh test directory'
    app._pred = np.zeros(2); app._cur = np.zeros(2)
    app._base = np.zeros(2); app._vel = np.zeros(2)
    app._has_target = app._has_base = app._has_advice = False
    app._sampler = TickSampler(float(params['judge.state_timeout_s']),
                                float(params['judge.observation_timeout_s']))
    app._n_target_fallback = 0
    app._epoch = None
    app._tick_index = -1
    app._n_tick = 0
    app._t_last_tick_mono = time.monotonic()
    app._origin_frozen = False
    app._replay_not_before = 1000.
    app._replay_stamp = None
    app._grid = None
    app._grid_res = 0.05
    app._grid_origin = np.zeros(2, dtype=float)
    app._grid_yaw = 0.0
    app._t_last_map_save = 0.0
    app._map_saved_final = False
    app._pub_origin_freeze = Publisher()
    return app


def observation(t, xyz=(.1, .2, 2.), marker_id=3):
    msg = MarkerObservation()
    msg.header.stamp = stamp(t); msg.header.frame_id = 'camera_optical_frame'
    msg.marker_id = marker_id; msg.valid = True
    msg.x, msg.y, msg.z = map(float, xyz)
    msg.cov_optical = list(map(float, (np.eye(3)*.0001).flat))
    return msg


class TimingTests(unittest.TestCase):
    def test_actual_submission_callback_sampling_and_body_twist(self):
        with tempfile.TemporaryDirectory() as directory:
            app = judge_node(directory)
            m = TargetState()
            m.header.stamp = stamp(1000.); m.header.frame_id = 'map'
            m.cur_x, m.cur_y = 1., 2.
            m.vel_x, m.vel_y = .4, -.2
            m.acc_x, m.acc_y = 0., 0.
            m.valid = True; m.track_status = 1; m.last_obs_age = 0.
            app._on_target(m)
            b = odom(1000., (2., 3., 0.), (0., 0., math.pi/2)); b.header.frame_id = 'map'
            b.twist.twist.linear = vec((.5, 0., 0.)); app._on_base_pose(b)
            advice = ChassisAdvice()
            advice.header.stamp = stamp(1000.); advice.header.frame_id = 'map'
            advice.cmd_vx, advice.cmd_vy, advice.valid = .3, .4, True
            app._on_advice(advice)
            replay = Header(); replay.stamp = stamp(1000.); replay.frame_id = 'replay'
            app._on_replay_start(replay)
            tick = Header(); tick.stamp = stamp(1000.04); tick.frame_id = '0'
            app._on_tick(tick)
            values = list(map(float, Path(app._csv_path).read_text().strip().split(',')))
            np.testing.assert_allclose(values, [1.056,1.972,1.016,1.992,2.,3.02,.3,.4,1000.04])
            invalid = ChassisAdvice()
            invalid.header.stamp = stamp(1000.05); invalid.header.frame_id = 'map'
            invalid.cmd_vx, invalid.cmd_vy, invalid.valid = 3., 4., False
            app._on_advice(invalid)
            tick = Header(); tick.stamp = stamp(1000.06); tick.frame_id = '1'
            app._on_tick(tick)
            self.assertFalse(app._has_advice)
            np.testing.assert_array_equal(app._vel, [0., 0.])
            app._writer.close()

    def test_pose_slerp_short_arc_and_no_extrapolation(self):
        h = PoseHistory()
        h.put(1., FR.compose_T(FR.rotz(math.radians(179)), [1, 2, 3]))
        h.put(1.1, FR.compose_T(FR.rotz(math.radians(-179)), [3, 4, 5]))
        mid = h.sample(1.05)
        np.testing.assert_allclose(mid[:3, 3], [2, 3, 4])
        self.assertAlmostEqual(abs(FR.yaw_of_mat(mid[:3, :3])), math.pi)
        self.assertIsNone(h.sample(.99)); self.assertIsNone(h.sample(1.11))
        self.assertIsNone(h.sample(1.05, max_gap=.05))

    def test_buffer_replacement_capacity_and_invalid_time(self):
        h = TimeBuffer(capacity=2)
        for t in (3., 1., 2.): h.put(t, t)
        self.assertEqual(h.times, [2., 3.])
        h.put(2., 'replacement'); self.assertEqual(h.before(2.1, .2), (2., 'replacement'))
        self.assertIsNone(h.before(1.9, .2)); self.assertIsNone(h.before(3.3, .2))
        for bad in (math.nan, -1., math.inf):
            with self.assertRaises(ValueError): h.put(bad, None)

    def test_delayed_replay_freezes_at_source_stamp_not_latest(self):
        clock, app = map_node()
        app._on_base_pose(odom(1000., (2, -1, .4), (.1, .05, math.pi/2)))
        app._on_origin_freeze(Header(stamp=stamp(1000.04)))
        self.assertFalse(app.frozen)
        clock.now = 1000.12
        src = odom(1000.1, (2, -.9, .4), (.1, .05, math.pi/2))
        app._on_base_pose(src); app._tick()
        self.assertTrue(app.frozen)
        out = app._pub_pose.messages[-1]
        self.assertEqual(stamp_seconds(out.header.stamp), 1000.1)
        np.testing.assert_allclose([out.pose.pose.position.x, out.pose.pose.position.y], [.06, 0], atol=1e-12)
        self.assertAlmostEqual(out.twist.twist.linear.x, src.twist.twist.linear.x)
        self.assertAlmostEqual(np.asarray(out.pose.covariance).reshape(6, 6)[0, 0], 2.)
        self.assertEqual([(x.header.frame_id, x.child_frame_id) for x in app._static_tf.messages], [('map', 'odom')])
        app._tick(); self.assertEqual(len(app._pub_pose.messages), 1)
        clock.now = 1001.; app._tick(); self.assertEqual(len(app._pub_pose.messages), 1)

    def test_stale_latched_start_does_not_block_new_start_or_rebase_frozen_round(self):
        clock, app = map_node(); clock.now = 2000.; app._replay_not_before = 2000.
        app._on_base_pose(odom(2000., (9, 10, 0))); app._tick()
        self.assertFalse(app.frozen)
        app._on_origin_freeze(Header(stamp=stamp(1990.)))
        self.assertFalse(app.frozen)
        app._on_origin_freeze(Header(stamp=stamp(2000.)))
        self.assertTrue(app.frozen)
        self.assertEqual(app._origin_stamp, 2000.)
        original = app._T_map_odom.copy()
        app._on_origin_freeze(Header(stamp=stamp(2001.)))
        self.assertEqual(app._origin_stamp, 2000.)
        np.testing.assert_array_equal(app._T_map_odom, original)

    def test_tracker_measurement_time_projection_is_nonmutating(self):
        track = StampedTracker()
        for i in range(100):
            t = 1000.+i/30
            self.assertTrue(track.observe(t, [1+.4*(t-1000), 2-.2*(t-1000)], np.eye(2)*1e-4,
                                          now=t+.04, marker_id=3))
        before = track.kf.x.copy(); filter_t = track.kf.t_last
        s = track.sample(t+.04)
        np.testing.assert_allclose(s.position, [1+.4*(t+.04-1000), 2-.2*(t+.04-1000)], atol=.003)
        np.testing.assert_allclose(s.prediction-s.position, [.04, -.02], atol=.002)
        np.testing.assert_array_equal(track.kf.x, before); self.assertEqual(track.kf.t_last, filter_t)
        self.assertFalse(track.observe(t-.01, [5, 5], np.eye(2), now=t+.05, marker_id=3))
        self.assertFalse(track.sample(t+.501).valid)
        self.assertTrue(track.observe(t+.6, [7, 8], np.eye(2), now=t+.64, marker_id=4))
        np.testing.assert_allclose(track.sample(t+.6).position, [7, 8])

    def test_tick_uses_prior_state_and_projects_both_times(self):
        sampler = TickSampler()
        sampler.put_target(1000., [1, 2], [.4, -.2], [0, 0])
        sampler.put_target(1000.05, [99, 99], [0, 0], [0, 0])
        sampler.put_base(1000., [2, 3], [.5, 0])
        sampler.put_advice(1000.02, [.3, 0])
        s = sampler.sample(1000.04)
        np.testing.assert_allclose(s.current, [1.016, 1.992])
        np.testing.assert_allclose(s.prediction, [1.056, 1.972])
        np.testing.assert_allclose(s.base, [2.02, 3])
        self.assertTrue(s.target_valid and s.base_valid and s.advice_valid)

    def test_tick_acceleration_stale_and_explicit_invalid(self):
        sampler = TickSampler()
        sampler.put_target(1., [0, 0], [1, 0], [2, 0], observation_age=.49)
        self.assertFalse(sampler.sample(1.02).target_valid)
        sampler.put_target(2., [0, 0], [1, 0], [2, 0])
        s = sampler.sample(2.1); self.assertAlmostEqual(s.current[0], .11)
        self.assertAlmostEqual(s.prediction[0], .24)
        sampler.put_target(2.11, [9, 9], [0, 0], [0, 0], valid=False)
        self.assertFalse(sampler.sample(2.12).target_valid)
        np.testing.assert_array_equal(sampler.sample(3.).current, [0, 0])

    def test_full_tf_offset_tilt_and_covariance(self):
        b = ObservationBuilder(); R = FR.rpy_to_mat([.2, -.1, .7])
        T = FR.compose_T(R, [3, -2, .8]); C = np.diag([.01, .02, .03])
        result = b.build_from_transform([.1, -.3, 4], C, T)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.pos_map, R@np.array([.1, -.3, 4])+T[:3, 3])
        np.testing.assert_allclose(result.R_map, (R@C@R.T)[:2, :2]+np.eye(2)*.002**2)
        T[0, 0] += .2; self.assertFalse(b.build_from_transform([0, 0, 2], C, T).valid)
        self.assertFalse(b.build_from_transform([0, 0, 2], -C, np.eye(4)).valid)

    def test_actual_estimator_adapter_requests_image_time(self):
        clock, app = target_node(); clock.now = 1000.04
        app._tf_buffer.T = FR.compose_T(FR.rotz(.3), [1., 2., .5])
        msg = observation(1000.)
        app.on_marker(msg); app.tick()
        self.assertEqual(app._tf_buffer.calls[0], ('map', 'camera_optical_frame', 1000.))
        self.assertEqual(app._n_observed, 1)
        out = app._pub.messages[-1]; self.assertTrue(out.valid)
        self.assertEqual(stamp_seconds(out.header.stamp), clock.now)
        expected = app._tf_buffer.T@np.array([.1, .2, 2, 1])
        np.testing.assert_allclose([out.cur_x, out.cur_y], expected[:2])

    def test_missing_tf_waits_then_drops_no_identity_fallback(self):
        clock, app = target_node(); app._tf_buffer.missing = True
        app.on_marker(observation(1000.)); app.tick()
        self.assertEqual(len(app._pending), 1); self.assertFalse(app._pub.messages[-1].valid)
        clock.now = 1000.4; app.tick()
        self.assertFalse(app._pending); self.assertFalse(app._pub.messages[-1].valid)
        self.assertEqual(app._n_observed, 0)

    def test_unconfirmed_geometry_and_clock_rewind_are_invalid(self):
        clock, app = target_node(False); app.on_marker(observation(1000.)); app.tick()
        self.assertFalse(app._pub.messages[-1].valid); self.assertFalse(app._tf_buffer.calls)
        clock, app = target_node(); app.on_marker(observation(1000.)); app.tick()
        self.assertTrue(app._pub.messages[-1].valid)
        clock.now = 999.; app.tick(); self.assertFalse(app._pub.messages[-1].valid)


if __name__ == '__main__': unittest.main(verbosity=2)
