"""Real OpenCV image -> detector callbacks -> PnP callbacks, without ROS/DDS.

Only ROS node/message transport is replaced by in-memory test doubles. Detection,
subpixel refinement, pose solving, calibration loading and callback bodies are
the production implementations. This is not a Humble transport/hardware test.
Run from any directory: .venv/bin/python -B tools/verify_perception_offline.py
"""
from __future__ import annotations

import importlib
import math
import sys
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ('gimbal_perception', 'gimbal_common'):
    sys.path.insert(0, str(ROOT / 'src' / package))


class Header:
    def __init__(self):
        self.stamp = NS(sec=10, nanosec=0)
        self.frame_id = ''


class MarkerCorners:
    def __init__(self):
        self.header = Header()
        self.marker_id = -1
        self.corners = []
        self.valid = False


class MarkerObservation:
    def __init__(self):
        self.header = Header()


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self, name, parameters=None):
        self.name = name
        self.parameters = dict(parameters or {})
        self.publishers = {}

    def get_name(self):
        return self.name

    def has_parameter(self, key):
        return key in self.parameters

    def declare_parameter(self, key, value):
        self.parameters.setdefault(key, value)

    def get_parameter(self, key):
        return NS(value=self.parameters[key])

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = Publisher()
        return self.publishers[topic]

    def create_subscription(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=10, nanosec=900000000)))


def install_ros_test_doubles():
    def module(name, **members):
        value = types.ModuleType(name)
        value.__dict__.update(members)
        sys.modules[name] = value
        return value

    rclpy = module('rclpy')
    import logging
    rclpy.logging = module('rclpy.logging', get_logger=logging.getLogger)
    module('rclpy.node', Node=Node)
    module('rclpy.parameter', Parameter=object)
    policies = NS(RELIABLE=1, BEST_EFFORT=2, KEEP_LAST=1, VOLATILE=1, TRANSIENT_LOCAL=2)
    module('rclpy.qos', QoSProfile=lambda **kwargs: NS(**kwargs),
           QoSDurabilityPolicy=policies, QoSHistoryPolicy=policies,
           QoSReliabilityPolicy=policies)
    module('sensor_msgs')
    module('sensor_msgs.msg', Image=object, CameraInfo=object)
    module('geometry_msgs')
    module('geometry_msgs.msg', Point32=lambda: NS(x=0., y=0., z=0.))
    module('gimbal_interfaces')
    module('gimbal_interfaces.msg', MarkerCorners=MarkerCorners, MarkerObservation=MarkerObservation)

    class CvBridge:
        def imgmsg_to_cv2(self, *args, **kwargs):
            # cv_bridge/DDS are outside this offline test. Exercise the real
            # production byte/stride decoder, as the pre-merge harness did.
            raise RuntimeError('offline transport stand-in: use production manual decoder')

    module('cv_bridge', CvBridge=CvBridge)
    module('ament_index_python')

    def package_share(name):
        path = ROOT / 'src' / name
        if not path.is_dir():
            raise LookupError(f'no source-tree package share for {name}')
        return str(path)

    module('ament_index_python.packages', get_package_share_directory=package_share)


install_ros_test_doubles()
detector_module = importlib.import_module('gimbal_perception.aruco_detector_node')
pnp_module = importlib.import_module('gimbal_perception.marker_pnp_node')
from gimbal_perception.pnp_solver import PnpSolver, marker_object_points, rpy_to_R

CAMERA_PATH = ROOT / 'src/gimbal_tracker_bringup/config/camera_team47.yaml'
CAMERA = yaml.safe_load(CAMERA_PATH.read_text())['camera']
K = np.array([[CAMERA['fx'], 0., CAMERA['cx']],
              [0., CAMERA['fy'], CAMERA['cy']], [0., 0., 1.]])
D = np.array(CAMERA['distortion'])
OBJECT = marker_object_points(.10)


def project(translation, rotation=None):
    # Front of the printed marker faces the camera; marker y is up.
    rotation = rpy_to_R(math.pi, 0., 0.) if rotation is None else rotation
    rvec, _ = cv2.Rodrigues(rotation)
    corners, _ = cv2.projectPoints(OBJECT, rvec, np.array(translation, dtype=float), K, D)
    return corners.reshape(4, 2)


def render_marker(marker_id=3, translation=(0., 0., 2.), rotation=None):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 256)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 256)
    source = np.array([[0., 0.], [255., 0.], [255., 255.], [0., 255.]], np.float32)
    destination = project(translation, rotation).astype(np.float32)
    warp = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(marker, warp, (1440, 1080), flags=cv2.INTER_NEAREST,
                               borderValue=255)


def image_message(image, sec=10, padding=0):
    height, width = image.shape
    rows = np.pad(image, ((0, 0), (0, padding)), constant_values=123)
    return NS(height=height, width=width, encoding='mono8', step=width + padding,
              data=rows.tobytes(), header=NS(stamp=NS(sec=sec, nanosec=123000000)))


def pipeline(target=3):
    # Constructor uses actual calibration/noise setup and real OpenCV detectors.
    detector = detector_module.ArucoDetectorNode()
    detector.parameters['perception.target_digit'] = target
    detector._target_digit = target
    node = Node('marker_pnp_node', {'perception.target_digit': target,
                                  'config.virtual_yaml': str(CAMERA_PATH)})
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
    def test_rendered_image_to_id_and_pnp(self):
        for translation in ((0., 0., 1.), (.10, -.06, 2.), (-.20, .12, 3.)):
            with self.subTest(translation=translation):
                detector, pnp = pipeline()
                corners, result = feed(detector, pnp, render_marker(3, translation))
                self.assertTrue(corners.valid)
                self.assertEqual(corners.marker_id, 3)
                self.assertEqual(len(corners.corners), 4)
                self.assertTrue(result.valid)
                self.assertIs(result.header.stamp, corners.header.stamp)
                np.testing.assert_allclose([result.x, result.y, result.z], translation,
                                           atol=max(.02, translation[2] * .02))

    def test_rotated_code_keeps_decoder_corner_order(self):
        detector, pnp = pipeline()
        rotation = rpy_to_R(0., 0., .65) @ rpy_to_R(math.pi, .15, 0.)
        corners, result = feed(detector, pnp, render_marker(3, (.15, -.03, 2.), rotation))
        self.assertTrue(result.valid)
        self.assertEqual(corners.marker_id, 3)
        np.testing.assert_allclose([result.x, result.y, result.z], [.15, -.03, 2.], atol=.04)

    def test_old_detect_api_returns_two_production_values(self):
        detector, _ = pipeline()
        detector._aruco_det = None
        corners, ids = detector._detect(render_marker())
        self.assertIn(3, ids.ravel().tolist())
        self.assertEqual(len(corners), len(ids))

    def test_old_parameter_api_branch(self):
        constructor = (cv2.aruco.DetectorParameters if hasattr(cv2.aruco, 'DetectorParameters')
                       else cv2.aruco.DetectorParameters_create)
        with mock.patch.object(cv2.aruco, 'DetectorParameters', side_effect=AttributeError('old API'), create=True):
            with mock.patch.object(cv2.aruco, 'DetectorParameters_create',
                                   side_effect=constructor, create=True):
                self.assertIsNotNone(detector_module._make_detector_params())

    def test_stride_fallback_reads_padded_rows(self):
        detector, _ = pipeline()
        detector._bridge = None
        image = render_marker()
        np.testing.assert_array_equal(detector._to_gray(image_message(image, padding=8)), image)
        bad = image_message(image)
        bad.step = image.shape[1] - 1
        self.assertIsNone(detector._to_gray(bad))

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
        detector.parameters['perception.target_digit'] = 3
        pnp._node.parameters['perception.target_digit'] = 3
        detector._on_timer()  # ID change clears cached pre-selection image.
        self.assertFalse(detector._pub.messages[-1].valid)
        old_corners, result = feed(detector, pnp, render_marker())
        self.assertTrue(result.valid)
        pnp._node.parameters['perception.target_digit'] = 8
        pnp._on_corners(old_corners)
        self.assertFalse(pnp._pub.messages[-1].valid)
        detector.parameters['perception.target_digit'] = 8
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
        solver._has_ippe = False
        result = solver.solve(project((0., 0., 2.)))
        self.assertTrue(result.ok, result.reason)
        np.testing.assert_allclose(result.tvec_opt, [0., 0., 2.], atol=1e-5)
        solver._has_ippe = True
        with mock.patch.object(cv2, 'solvePnPGeneric', side_effect=cv2.error('simulated old failure')):
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
        pnp_module.LU.reset_throttle()
        with mock.patch.object(pnp_module.LU.time_utils, 'now', return_value=100.):
            with mock.patch.object(pnp._log, 'info') as log:
                for error in (2.81, 2.53, 2.30):
                    with mock.patch.object(pnp._solver, 'solve',
                                           return_value=NS(ok=False, reason=f'reproj_{error:.2f}px')):
                        pnp._on_corners(corners)
                self.assertEqual(log.call_count, 1)
        self.assertEqual(pnp._n_reject, 3)
        self.assertFalse(pnp._pub.messages[-1].valid)
        pnp_module.LU.reset_throttle()


if __name__ == '__main__':
    print(f'OpenCV {cv2.__version__}; real detection/PnP, mocked ROS transport; Humble/4.5.4 not verified',
          flush=True)
    unittest.main(verbosity=2)
