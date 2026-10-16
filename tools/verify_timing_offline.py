"""Production time/TF adapters with in-memory ROS stand-ins; NOT DDS/Humble."""
import ast
import copy
import math
import sys
import os
import time
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for pkg in ('gimbal_common', 'gimbal_estimation', 'gimbal_judge_io', 'gimbal_localization'):
    sys.path.insert(0, str(ROOT/'src'/pkg))
from gimbal_common import frames as FR
from gimbal_common.time_buffer import TimeBuffer, PoseHistory, stamp_seconds
from gimbal_estimation.stamped_tracker import StampedTracker
from gimbal_estimation.observation_builder import ObservationBuilder
from gimbal_estimation.ca_kalman import CAKalman
from gimbal_judge_io.tick_sampler import TickSampler
from gimbal_judge_io.tick_csv import TickCsvWriter, V_STOP


def stamp(t):
    ns = round(t*1e9)
    return NS(sec=ns//1000000000, nanosec=ns%1000000000)


def vec(values=(0., 0., 0.)):
    return NS(**dict(zip(('x', 'y', 'z'), map(float, values))))


def quat(R=None):
    q = FR.quat_from_mat(np.eye(3) if R is None else R)
    return NS(**dict(zip(('w', 'x', 'y', 'z'), map(float, q))))


def transform(T=None):
    T = np.eye(4) if T is None else T
    return NS(header=NS(stamp=stamp(0), frame_id=''), child_frame_id='',
              transform=NS(translation=vec(T[:3, 3]), rotation=quat(T[:3, :3])))


def odom(t=0., xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    return NS(header=NS(stamp=stamp(t), frame_id='odom'), child_frame_id='base_link',
              pose=NS(pose=NS(position=vec(xyz), orientation=quat(FR.rpy_to_mat(rpy))),
                      covariance=list(np.diag([1., 2., 3., 4., 5., 6.]).flat)),
              twist=NS(twist=NS(linear=vec((.2, .1, .05)), angular=vec((.01, .02, .03))),
                       covariance=[0.]*36))


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))


class FakeNode:
    overrides = {}
    def __init__(self, *args):
        self.params = dict(self.overrides); self.now = 1000.04; self.publishers = {}
    def declare_parameter(self, name, value): self.params.setdefault(name, value)
    def get_parameter(self, name): return NS(value=self.params[name])
    def get_logger(self): return LOG
    def create_publisher(self, typ, topic, qos):
        self.publishers[topic] = Publisher(); return self.publishers[topic]
    def create_subscription(self, *args): return None
    def create_timer(self, *args): return None
    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(self.now*1e9), to_msg=lambda: stamp(self.now)))


LOG = NS(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)


def load_ast(relative_path, names, env):
    """Execute actual source definitions, excluding unavailable ROS imports."""
    path = ROOT/relative_path
    tree = ast.parse(path.read_text())
    body = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in names
            or isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in n.targets)
            or isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name) and n.target.id in names]
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), env)
    return env


def map_node():
    class Broadcaster:
        def __init__(self, node): self.messages = []
        def sendTransform(self, msg): self.messages.append(copy.deepcopy(msg))
    node = FakeNode()
    env = load_ast('src/gimbal_localization/gimbal_localization/map_frame_broadcaster_node.py',
        {'PARAM_DEFAULTS', '_xyz_rpy_to_T', 'MapFrameBroadcasterNode'},
        dict(copy=copy, np=np, FR=FR, math=math, PoseHistory=PoseHistory,
             stamp_seconds=stamp_seconds, TransformStamped=transform, Odometry=object,
             Header=object, StaticTransformBroadcaster=Broadcaster,
             _latched_qos=lambda: None, _sensor_qos=lambda: None,
             LU=NS(get_logger=lambda *a: LOG, throttled=lambda *a: None),
             NU=NS(declare_params=lambda node, defaults: dict(defaults, **node.params)),
             TU=NS(now=lambda: node.now)))
    return node, env['MapFrameBroadcasterNode'](node)


class MissingTransform(Exception): pass


def target_node(confirmed=True):
    class Base(FakeNode):
        overrides = {'estimation.geometry_confirmed': confirmed}
    class TF:
        def __init__(self, **kwargs):
            self.calls = []; self.missing = False; self.T = np.eye(4)
        def lookup_transform(self, target, source, time, **kwargs):
            self.calls.append((target, source, stamp_seconds(time)))
            if self.missing: raise MissingTransform()
            return transform(self.T)
    class Policy:
        BEST_EFFORT = RELIABLE = TRANSIENT_LOCAL = 0
    env = load_ast('src/gimbal_estimation/gimbal_estimation/target_estimator_node.py',
        {'TargetEstimatorNode'}, dict(Node=Base, math=math, np=np, frames=FR,
            CAKalman=CAKalman, StampedTracker=StampedTracker, ObservationBuilder=ObservationBuilder,
            stamp_seconds=stamp_seconds, get_logger=lambda *a: LOG, throttled=lambda *a: None,
            Buffer=TF, TransformListener=lambda *a: None, Duration=lambda **kwargs: kwargs,
            Time=NS(from_msg=lambda x: x), TransformException=MissingTransform,
            QoSProfile=lambda **kwargs: kwargs, ReliabilityPolicy=Policy, DurabilityPolicy=Policy,
            TargetState=lambda: NS(header=NS()), MarkerObservation=object, UInt64=object))
    return env['TargetEstimatorNode']()


def observation(t, xyz=(.1, .2, 2.), marker_id=3):
    return NS(header=NS(stamp=stamp(t), frame_id='camera_optical_frame'),
              marker_id=marker_id, valid=True, x=xyz[0], y=xyz[1], z=xyz[2],
              cov_optical=list((np.eye(3)*.0001).flat))


class TimingTests(unittest.TestCase):
    def test_actual_submission_callback_sampling_and_body_twist(self):
        with tempfile.TemporaryDirectory() as directory:
            class Base(FakeNode):
                overrides = {'judge.output_dir': directory, 'judge.enforce_v_floor': False}
            env = load_ast('src/gimbal_judge_io/gimbal_judge_io/judge_io_node.py',
                {'JudgeIoNode','PARAM_DEFAULTS','_stamp_to_sec'}, dict(Node=Base, Any=object,
                    Dict=dict, Optional=object, np=np, math=math, os=os, time=time,
                    Header=lambda:NS(), Odometry=object, OccupancyGrid=object,
                    ChassisAdvice=object, TargetState=object, V_STOP=V_STOP,
                    TickCsvWriter=TickCsvWriter, TickSampler=TickSampler, frames=FR,
                    stamp_seconds=stamp_seconds, NODE_NAME='judge_io_node',
                    _sensor_qos=lambda *a:None, _latched_qos=lambda *a:None, _judge_tick_qos=lambda:None))
            app = env['JudgeIoNode']()
            m = NS(header=NS(stamp=stamp(1000.),frame_id='map'), cur_x=1.,cur_y=2.,
                vel_x=.4,vel_y=-.2,acc_x=0.,acc_y=0.,valid=True,track_status=1,last_obs_age=0.)
            app._on_target(m)
            b = odom(1000., (2.,3.,0.), (0.,0.,math.pi/2)); b.header.frame_id='map'
            b.twist.twist.linear=vec((.5,0.,0.)); app._on_base_pose(b)
            app._on_advice(NS(header=NS(stamp=stamp(1000.),frame_id='map'),cmd_vx=.3,cmd_vy=.4,valid=True))
            app._on_replay_start(NS(stamp=stamp(1000.),frame_id='replay'))
            app._on_tick(NS(stamp=stamp(1000.04),frame_id='0'))
            values = list(map(float, Path(app._csv_path).read_text().strip().split(',')))
            np.testing.assert_allclose(values, [1.056,1.972,1.016,1.992,2.,3.02,.3,.4,1000.04])
            app._on_advice(NS(header=NS(stamp=stamp(1000.05),frame_id='map'),cmd_vx=3.,cmd_vy=4.,valid=False))
            app._on_tick(NS(stamp=stamp(1000.06),frame_id='1'))
            self.assertFalse(app._has_advice)
            np.testing.assert_array_equal(app._vel, [0.,0.])
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
        node, app = map_node()
        app._on_base_pose(odom(1000., (2, -1, .4), (.1, .05, math.pi/2)))
        app._on_origin_freeze(NS(stamp=stamp(1000.04)))
        self.assertFalse(app.frozen)
        node.now = 1000.12
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
        node.now = 1001.; app._tick(); self.assertEqual(len(app._pub_pose.messages), 1)

    def test_no_timeout_origin_fallback_or_new_round_rebase(self):
        node, app = map_node(); node.now = 2000.
        app._on_base_pose(odom(2000., (9, 10, 0))); app._tick()
        self.assertFalse(app.frozen)
        app._on_origin_freeze(NS(stamp=stamp(1990.)))
        self.assertFalse(app.frozen)
        app._on_origin_freeze(NS(stamp=stamp(2000.)))
        self.assertFalse(app.frozen)
        self.assertEqual(app._origin_stamp, 1990.)

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
        app = target_node(); app.now = 1000.04
        app._tf_buffer.T = FR.compose_T(FR.rotz(.3), [1., 2., .5])
        msg = observation(1000.)
        app.on_marker(msg); app.tick()
        self.assertEqual(app._tf_buffer.calls[0], ('map', 'camera_optical_frame', 1000.))
        self.assertEqual(app._n_observed, 1)
        out = app._pub.messages[-1]; self.assertTrue(out.valid)
        self.assertEqual(stamp_seconds(out.header.stamp), app.now)
        expected = app._tf_buffer.T@np.array([.1, .2, 2, 1])
        np.testing.assert_allclose([out.cur_x, out.cur_y], expected[:2])

    def test_missing_tf_waits_then_drops_no_identity_fallback(self):
        app = target_node(); app._tf_buffer.missing = True
        app.on_marker(observation(1000.)); app.tick()
        self.assertEqual(len(app._pending), 1); self.assertFalse(app._pub.messages[-1].valid)
        app.now = 1000.4; app.tick()
        self.assertFalse(app._pending); self.assertFalse(app._pub.messages[-1].valid)
        self.assertEqual(app._n_observed, 0)

    def test_unconfirmed_geometry_and_clock_rewind_are_invalid(self):
        app = target_node(False); app.on_marker(observation(1000.)); app.tick()
        self.assertFalse(app._pub.messages[-1].valid); self.assertFalse(app._tf_buffer.calls)
        app = target_node(); app.on_marker(observation(1000.)); app.tick()
        self.assertTrue(app._pub.messages[-1].valid)
        app.now = 999.; app.tick(); self.assertFalse(app._pub.messages[-1].valid)


if __name__ == '__main__': unittest.main(verbosity=2)
