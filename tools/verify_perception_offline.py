"""Real OpenCV image -> detector callbacks -> PnP callbacks on real ROS 2 Humble.

Runs inside the ROS 2 Humble container (see Dockerfile): imports the real
rclpy, sensor_msgs, geometry_msgs and gimbal_interfaces, constructs real nodes,
and captures what their publishers emit by wrapping Node.create_publisher.
Detection, subpixel refinement, pose solving, calibration loading and callback
bodies are the production implementations. There is no camera, physical
device, remote DDS peer or official scorer involved.
Run from any directory: python3 -B tools/verify_perception_offline.py
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
for package in ('gimbal_perception', 'gimbal_common'):
    sys.path.insert(0, str(ROOT / 'src' / package))

import gimbal_perception.aruco_detector_node as detector_module
import gimbal_perception.marker_pnp_node as pnp_module
from gimbal_common import log_utils, node_utils as NU
from gimbal_perception.pnp_solver import PnpSolver, marker_object_points, rpy_to_R

CAMERA_PATH = ROOT / 'src/gimbal_tracker_bringup/config/camera_team47.yaml'
CAMERA = yaml.safe_load(CAMERA_PATH.read_text())['camera']
K = np.array([[CAMERA['fx'], 0., CAMERA['cx']],
              [0., CAMERA['fy'], CAMERA['cy']], [0., 0., 1.]])
D = np.array(CAMERA['distortion'])
OBJECT = marker_object_points(.10)


class _RecordingPublisher:
    """Real rclpy publisher with every publish() recorded for assertions."""

    def __init__(self, publisher):
        self._publisher = publisher
        self.messages = []

    def publish(self, message):
        self.messages.append(message)
        return self._publisher.publish(message)

    def __getattr__(self, name):
        return getattr(self._publisher, name)


def install_publish_recording():
    """Wrap Node.create_publisher so created publishers record their messages."""
    original = RclpyNode.create_publisher

    def create_publisher_recording(self, *args, **kwargs):
        return _RecordingPublisher(original(self, *args, **kwargs))

    return mock.patch.object(RclpyNode, 'create_publisher', create_publisher_recording)


CREATED_NODES = []
_node_counter = 0


def fresh_node_name(base):
    global _node_counter
    _node_counter += 1
    return f'{base}_offline_{_node_counter}'


def set_int_parameter(node, key, value):
    # Humble rclpy exposes set_parameters(); the nodes declare the key first.
    node.set_parameters([Parameter(key, Parameter.Type.INTEGER, int(value))])


def drain_created_nodes():
    while CREATED_NODES:
        node = CREATED_NODES.pop()
        try:
            node.destroy_node()
        except Exception:
            pass


def project(translation, rotation=None):
    # Front of the printed marker faces the camera; marker y is up.
    rotation = rpy_to_R(math.pi, 0., 0.) if rotation is None else rotation
    rvec, _ = cv2.Rodrigues(rotation)
    corners, _ = cv2.projectPoints(OBJECT, rvec, np.array(translation, dtype=float), K, D)
    return corners.reshape(4, 2)


def render_marker(marker_id=3, translation=(0., 0., 2.), rotation=None):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.drawMarker(dictionary, marker_id, 256)
    source = np.array([[0., 0.], [255., 0.], [255., 255.], [0., 255.]], np.float32)
    destination = project(translation, rotation).astype(np.float32)
    warp = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(marker, warp, (1440, 1080), flags=cv2.INTER_NEAREST,
                               borderValue=255)


def image_message(image, sec=10):
    height, width = image.shape
    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = 'mono8'
    msg.step = width
    msg.data = image.tobytes()
    msg.header.stamp = Time(sec=sec, nanosec=123000000)
    return msg


def pipeline(target=3):
    # Real rclpy nodes on the live context; detection/PnP stay production code.
    # Fresh node names keep each pipeline a distinct DDS entity for teardown.
    detector = detector_module.ArucoDetectorNode(fresh_node_name(detector_module.NODE_NAME))
    CREATED_NODES.append(detector)
    if target != detector._target_digit:
        set_int_parameter(detector, 'perception.target_digit', target)
        detector._target_digit = target  # stay in sync so the first tick is no ID switch
    node = NU.create_node(fresh_node_name(pnp_module.NODE_NAME),
                          parameter_overrides=[
                              Parameter('perception.target_digit', Parameter.Type.INTEGER, int(target)),
                              Parameter('config.virtual_yaml', Parameter.Type.STRING, str(CAMERA_PATH))])
    CREATED_NODES.append(node)
    pnp = pnp_module.MarkerPnpNode(node)
    return detector, pnp


def feed(detector, pnp, image, sec=10):
    msg = image_message(image, sec)
    detector._on_image(msg)
    detector._on_timer()
    corners = detector._pub.messages[-1]
    pnp._on_corners(corners)
    return corners, pnp._pub.messages[-1]


class PerceptionRegression(unittest.TestCase):
    def tearDown(self):
        drain_created_nodes()

    def test_rendered_image_to_id_and_pnp(self):
        for translation in ((0., 0., 1.), (.10, -.06, 2.), (-.20, .12, 3.)):
            with self.subTest(translation=translation):
                detector, pnp = pipeline()
                corners, result = feed(detector, pnp, render_marker(3, translation))
                self.assertTrue(corners.valid)
                self.assertEqual(corners.marker_id, 3)
                self.assertEqual(len(corners.corners), 4)
                self.assertTrue(result.valid)
                self.assertEqual(result.header.stamp, corners.header.stamp)
                np.testing.assert_allclose([result.x, result.y, result.z], translation,
                                           atol=max(.02, translation[2] * .02))

    def test_rotated_code_keeps_decoder_corner_order(self):
        detector, pnp = pipeline()
        rotation = rpy_to_R(0., 0., .65) @ rpy_to_R(math.pi, .15, 0.)
        corners, result = feed(detector, pnp, render_marker(3, (.15, -.03, 2.), rotation))
        self.assertTrue(result.valid)
        self.assertEqual(corners.marker_id, 3)
        np.testing.assert_allclose([result.x, result.y, result.z], [.15, -.03, 2.], atol=.04)

    def test_detect_returns_two_production_values(self):
        detector, _ = pipeline()
        corners, ids = detector._detect(render_marker())
        self.assertIn(3, ids.ravel().tolist())
        self.assertEqual(len(corners), len(ids))

    def test_loss_clears_guess_and_reacquires(self):
        detector, pnp = pipeline()
        _, result = feed(detector, pnp, render_marker())
        self.assertTrue(result.valid)
        self.assertIsNotNone(pnp._solver._tvec_prev)
        corners, result = feed(detector, pnp, np.full((1080, 1440), 255, np.uint8), 11)
        self.assertFalse(corners.valid)
        self.assertEqual(corners.corners, [])
        self.assertFalse(result.valid)
        self.assertIsNone(pnp._solver._tvec_prev)
        _, result = feed(detector, pnp, render_marker(3, (.2, 0., 2.)), 12)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.x, .2, delta=.02)

    def test_image_stall_emits_loss_once(self):
        detector, pnp = pipeline()
        feed(detector, pnp, render_marker())
        last_count = len(detector._pub.messages)
        with mock.patch.object(detector_module.time, 'monotonic',
                               return_value=detector._last_image_received + 1.):
            detector._on_timer()
            detector._on_timer()
        self.assertEqual(len(detector._pub.messages), last_count + 1)
        pnp._on_corners(detector._pub.messages[-1])
        self.assertFalse(pnp._pub.messages[-1].valid)

    def test_delayed_id_and_queued_wrong_id_never_mislabel(self):
        detector, pnp = pipeline(-1)
        detector._on_image(image_message(render_marker()))
        detector._on_timer()
        self.assertEqual(detector._pub.messages, [])
        set_int_parameter(detector, 'perception.target_digit', 3)
        set_int_parameter(pnp._node, 'perception.target_digit', 3)
        detector._on_timer()  # ID change clears cached pre-selection image.
        self.assertFalse(detector._pub.messages[-1].valid)
        old_corners, result = feed(detector, pnp, render_marker())
        self.assertTrue(result.valid)
        set_int_parameter(pnp._node, 'perception.target_digit', 8)
        pnp._on_corners(old_corners)
        self.assertFalse(pnp._pub.messages[-1].valid)
        set_int_parameter(detector, 'perception.target_digit', 8)
        detector._on_timer()
        _, result = feed(detector, pnp, render_marker(8), 13)
        self.assertTrue(result.valid)
        self.assertEqual(result.marker_id, 8)

    def test_exact_distorted_projection_and_iterative_fallback(self):
        for translation in ((0., 0., 2.), (.3, -.2, 2.), (0., 0., 40.)):
            solver = PnpSolver.from_camera_config(CAMERA, marker_size_m=.10)
            result = solver.solve(project(translation))
            if translation[2] > solver.depth_max:
                self.assertFalse(result.ok)
                self.assertTrue(result.reason.startswith('depth'))
            else:
                self.assertTrue(result.ok, result.reason)
                np.testing.assert_allclose(result.tvec_opt, translation, atol=1e-5)
        # IPPE 支路整体抛错（如退化观测）时 ITERATIVE 兜底仍要出解。
        with mock.patch.object(cv2, 'solvePnPGeneric', side_effect=cv2.error('simulated solve failure')):
            result = solver.solve(project((.1, -.1, 2.)))
        self.assertTrue(result.ok, result.reason)
        np.testing.assert_allclose(result.tvec_opt, [.1, -.1, 2.], atol=1e-5)

    def test_malformed_corners_and_intrinsics_reject(self):
        solver = PnpSolver.from_camera_config(CAMERA, marker_size_m=.10)
        for corners in ([], [[1., 2.]] * 3, [[1., 2.]] * 4,
                        [[0., 0.], [2., 2.], [0., 2.], [2., 0.]],
                        [[0., 0.], [float('nan'), 0.], [2., 2.], [0., 2.]], [1., 2., 3.]):
            self.assertFalse(solver.solve(corners).ok)
        self.assertFalse(PnpSolver().solve(project((0., 0., 2.))).ok)
        with self.assertRaises(ValueError):
            solver.set_intrinsics(float('nan'), 1., 0., 0.)
        with self.assertRaises(ValueError):
            solver.set_intrinsics(1., 1., 0., 0., [0., 0., float('inf'), 0., 0.])

    def test_varying_reprojection_errors_share_one_throttle_key(self):
        detector, pnp = pipeline()
        corners, _ = feed(detector, pnp, render_marker())
        log_utils.reset_throttle()
        with mock.patch.object(log_utils.time_utils, 'now', return_value=100.):
            with mock.patch.object(pnp._log, 'info') as log:
                for error in (2.81, 2.53, 2.30):
                    with mock.patch.object(pnp._solver, 'solve',
                                           return_value=NS(ok=False, reason=f'reproj_{error:.2f}px')):
                        pnp._on_corners(corners)
                self.assertEqual(log.call_count, 1)
        self.assertEqual(pnp._n_reject, 3)
        self.assertFalse(pnp._pub.messages[-1].valid)
        log_utils.reset_throttle()


if __name__ == '__main__':
    print(f'OpenCV {cv2.__version__}; real rclpy nodes/messages on ROS 2 Humble; no camera/hardware',
          flush=True)
    rclpy.init()
    recording = install_publish_recording()
    recording.start()
    try:
        program = unittest.main(verbosity=2, exit=False)
    finally:
        recording.stop()
        drain_created_nodes()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if program.result.wasSuccessful() else 1)
