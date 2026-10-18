"""Exercise production debug decoding/drawing/logging with ROS-only stand-ins.

Uses real OpenCV, NumPy, CSV and files in a temporary directory. It never starts
ROS, opens a serial port or publishes a velocity command.
"""
import ast
import copy
import csv
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'src/gimbal_control/gimbal_control/tracking_debug_node.py'
CAMERA = {'width': 640, 'height': 480, 'cx': 320., 'cy': 240.}


class Image:
    pass


def header(source=100.):
    ns = round(source*1e9)
    sec, nanosec = divmod(ns, 1_000_000_000)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
                           frame_id='camera_optical_frame')


def image(pixels=None, source=100., encoding='bgr8', padding=0):
    if pixels is None:
        pixels = np.zeros((480, 640, 3), dtype=np.uint8)
    height, width = pixels.shape[:2]
    channels = 1 if pixels.ndim == 2 else pixels.shape[2]
    step = width*channels+padding
    data = np.full((height, step), 254, dtype=np.uint8)
    data[:, :width*channels] = pixels.reshape(height, width*channels)
    return SimpleNamespace(header=header(source), encoding=encoding,
                           width=width, height=height, step=step, data=data.tobytes())


def corners(source=100.):
    return SimpleNamespace(header=header(source), valid=True, marker_id=2,
        corners=[SimpleNamespace(x=x, y=y) for x, y in
                 ((40, 300), (100, 300), (100, 360), (40, 360))])


class HarnessNode:
    def __init__(self, name):
        self.name = name
        self.params = {}
        self.published = []
        self.subscriptions = []
        self.warns = []
        self.clock = 100.

    def declare_parameter(self, name, default):
        self.params[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.clock*1e9)))

    def get_logger(self):
        return SimpleNamespace(warning=self.warns.append)

    def create_publisher(self, message_type, topic, qos):
        assert message_type is Image and topic == 'debug_image'
        return SimpleNamespace(publish=lambda message: self.published.append(copy.deepcopy(message)))

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions.append((message_type, topic))

    def create_timer(self, period, callback):
        assert period == .1

    def destroy_node(self):
        return True


def load_actual_definitions():
    tree = ast.parse(SOURCE.read_text())
    declarations = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]
    environment = dict(copy=copy, csv=csv, json=json, math=math, Path=Path, cv2=cv2, np=np,
        Node=HarnessNode, Image=Image, String=object, MarkerCorners=object,
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'),
        load_yaml=lambda path: {'camera': copy.deepcopy(CAMERA)})
    exec(compile(ast.Module(body=declarations, type_ignores=[]), str(SOURCE), 'exec'), environment)
    return environment


ENV = load_actual_definitions()
decode = ENV['decode_image']
overlay = ENV['draw_overlay']
StatusCSV = ENV['StatusCSV']
DebugNode = ENV['TrackingDebugNode']


def test_color_and_padded_step():
    rgb = np.array([[[255, 0, 0], [0, 255, 0]],
                    [[0, 0, 255], [12, 34, 56]]], dtype=np.uint8)
    message = image(rgb, encoding='rgb8', padding=5)
    assert np.array_equal(decode(message), rgb[:, :, ::-1])
    bgr = image(rgb, encoding='bgr8', padding=2)
    decoded = decode(bgr)
    assert np.array_equal(decoded, rgb)
    decoded[0, 0] = 0
    assert decode(bgr)[0, 0, 0] == 255  # Not a view into message storage.


def test_mono_and_invalid_buffers():
    gray = np.array([[0, 20], [100, 255]], dtype=np.uint8)
    decoded = decode(image(gray, encoding='mono8', padding=3))
    assert decoded.shape == (2, 2, 3)
    assert np.array_equal(decoded[:, :, 1], gray)
    for change in ({'encoding': '16UC1'}, {'step': 1}, {'height': 0}, {'data': b'bad'}):
        message = image()
        message.__dict__.update(change)
        try:
            decode(message)
        except ValueError:
            continue
        raise AssertionError('invalid image accepted: '+str(change))


def test_overlay_matching_and_immutability():
    original = np.zeros((480, 640, 3), dtype=np.uint8)
    matched = overlay(original, CAMERA, 100., corners(100.079),
                      {'image_stamp': 100., 'state': '跟踪中', 'error_px': [8, 9],
                       'filtered_error_deg': [1, 2], 'control_error_deg': [0, 2],
                       'yaw_omega': .1, 'pitch_omega': .2, 'lost_count': 0})
    assert np.array_equal(matched[300, 40], [0, 255, 0])
    assert np.array_equal(matched[240, 330], [0, 255, 255])
    assert not np.any(original)
    mismatched = overlay(original, CAMERA, 100., corners(100.081), {'image_stamp': 99.})
    assert not np.any(mismatched[300, 40])
    assert not np.array_equal(matched[:180], mismatched[:180])
    try:
        overlay(original, dict(CAMERA, width=320), 100.)
    except ValueError:
        pass
    else:
        raise AssertionError('mismatched calibration accepted')


def test_invalid_corners_not_drawn():
    original = np.zeros((480, 640, 3), dtype=np.uint8)
    for field, value in (('valid', False), ('corners', [])):
        message = corners()
        setattr(message, field, value)
        assert not np.any(overlay(original, CAMERA, 100., message)[300, 40])
    message = corners()
    message.corners[0].x = float('nan')
    assert not np.any(overlay(original, CAMERA, 100., message)[300, 40])


def test_csv_visible_while_open_and_exclusive():
    warnings = []
    with TemporaryDirectory(prefix='tracking-debug-') as directory:
        path = Path(directory)/'trial.csv'
        log = StatusCSV(str(path), warnings.append)
        assert len(list(csv.reader(path.open()))) == 1  # Header already flushed.
        status = {'source_stamp': 10., 'status_stamp': 10.02, 'error_px': [1., 2.],
                  'filtered_error_deg': [.1, .2], 'control_error_deg': [0., .2],
                  'lost_count': 1, 'yaw_omega': .2, 'pitch_omega': -.3, 'state': '跟踪中'}
        log.write(status)
        rows = list(csv.reader(path.open()))
        assert len(rows) == 2 and len(rows[1]) == len(StatusCSV.columns)
        assert rows[1][:4] == ['10.0', '10.02', '1.0', '2.0']
        assert rows[1][-1] == '跟踪中'
        before = path.read_bytes()
        second = StatusCSV(str(path), warnings.append)
        assert second.file is None and warnings
        second.write(status)
        assert path.read_bytes() == before
        log.write({'image_stamp': 11., 'error_px': [float('nan'), None]})
        rows = list(csv.reader(path.open()))
        assert rows[-1][2:4] == ['', '']
        log.close()


def test_csv_disk_failure_disables_only_logging():
    warnings = []
    with TemporaryDirectory(prefix='tracking-debug-') as directory:
        log = StatusCSV(str(Path(directory)/'trial.csv'), warnings.append)

        class BrokenWriter:
            def writerow(self, row):
                raise OSError('simulated disk full')

        log.writer = BrokenWriter()
        log.write({'image_stamp': 100.})
        assert log.file is None and len(warnings) == 1
        log.write({'image_stamp': 101.})
        assert len(warnings) == 1
    assert StatusCSV('', warnings.append).file is None


def test_node_keeps_camera_stamp_no_repeated_or_stale_frames():
    node = DebugNode()
    node.image = image()
    node.corners = corners()
    node.on_status(SimpleNamespace(data=json.dumps({'image_stamp': 100., 'state': '跟踪中'})))
    node.render()
    assert len(node.published) == 1
    result = node.published[0]
    assert result.header is not node.image.header
    assert result.header.stamp.sec == 100 and result.header.stamp.nanosec == 0
    assert result.encoding == 'bgr8' and result.step == 640*3
    assert len(result.data) == 480*640*3
    node.clock = 100.2
    node.render()
    assert len(node.published) == 1
    node.image = image(source=99.8)
    node.render()
    assert len(node.published) == 1  # Old frames cannot receive a new ROS timestamp.
    node.image = image(source=101.)
    node.render()
    assert len(node.published) == 1  # Excessively future-dated image rejected too.
    node.image = image(source=100.2)
    node.render()
    assert len(node.published) == 2
    assert {topic for _, topic in node.subscriptions} == {'/image_raw', 'marker_corners', 'status'}
    node.destroy_node()


def test_node_bad_json_or_image_is_isolated():
    node = DebugNode()
    node.on_status(SimpleNamespace(data='bad json'))
    node.on_status(SimpleNamespace(data='[]'))
    assert node.status is None
    node.on_status(SimpleNamespace(data='{}'))
    assert node.status == {}
    node.image = image(encoding='unknown')
    node.render()
    assert not node.published and node.warns
    node.clock += .1
    node.image = image(source=node.clock)
    node.render()
    assert len(node.published) == 1
    node.destroy_node()


def test_observer_has_no_actuation_imports_or_publishers():
    tree = ast.parse(SOURCE.read_text())
    imports = [node.module or '' for node in tree.body if isinstance(node, ast.ImportFrom)]
    assert not any('geometry_msgs' in name or 'serial' in name for name in imports)
    publishers = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute) and node.func.attr == 'create_publisher']
    assert len(publishers) == 1 and publishers[0].args[0].id == 'Image'


if __name__ == '__main__':
    tests = [value for name, value in list(globals().items()) if name.startswith('test_')]
    for test in tests:
        test()
    print(json.dumps({'checks_passed': len(tests), 'actual_opencv': cv2.__version__,
                      'ros_or_serial_started': False}))
