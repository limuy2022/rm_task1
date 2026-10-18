"""Read-only image/status observer. It never publishes a motion command.

Drawing and optional disk I/O live here, outside the controller process.
The image retains the camera timestamp; unmatched old detections are not drawn.
"""
import copy
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from gimbal_interfaces.msg import MarkerCorners
from gimbal_common.config_loader import load_yaml
from gimbal_common.qos_utils import sensor_qos


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def decode_image(message):
    """Decode supported 8-bit encodings, respecting ROS Image row padding."""
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(message.encoding)
    if channels is None:
        raise ValueError('expected bgr8/rgb8/mono8, got '+str(message.encoding))
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width*channels:
        raise ValueError('invalid image dimensions or row step')
    data = np.frombuffer(bytes(message.data), dtype=np.uint8)
    if data.size < height*step:
        raise ValueError('image data is shorter than height*step')
    pixels = data[:height*step].reshape(height, step)[:, :width*channels]
    if channels == 1:
        return cv2.cvtColor(pixels.reshape(height, width), cv2.COLOR_GRAY2BGR)
    pixels = pixels.reshape(height, width, channels)
    if message.encoding == 'rgb8':
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return pixels.copy()  # Never draw into a subscription message's data buffer.


def finite_number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return value
    return None


def pair(status, key):
    value = status.get(key)
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return (None, None)
    return tuple(finite_number(item) for item in value)


def status_image_stamp(status):
    return finite_number(status.get('image_stamp', status.get('source_stamp')))


def number_text(value):
    return '--' if finite_number(value) is None else f'{value:.3f}'


def draw_overlay(image, camera, source, corners=None, status=None, tolerance=.08):
    """Return a BGR copy. A 3-D point is unnecessary for this debug view."""
    canvas = image.copy()
    height, width = canvas.shape[:2]
    cx, cy = float(camera['cx']), float(camera['cy'])
    if int(camera['width']) != width or int(camera['height']) != height:
        raise ValueError('image dimensions do not match camera calibration')
    center = (int(round(cx)), int(round(cy)))
    cv2.drawMarker(canvas, center, (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
    cv2.putText(canvas, 'optical axis', (center[0]+12, center[1]-10),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1, cv2.LINE_AA)
    if (corners is not None and corners.valid and len(corners.corners) == 4
            and corners.header.frame_id == 'camera_optical_frame'
            and abs(stamp_seconds(corners.header)-source) <= tolerance):
        xy = np.asarray([(point.x, point.y) for point in corners.corners], dtype=float)
        if (np.all(np.isfinite(xy)) and np.all(xy >= 0)
                and np.all(xy[:, 0] < width) and np.all(xy[:, 1] < height)):
            points = np.rint(xy).astype(np.int32)
            cv2.polylines(canvas, [points], True, (0, 255, 0), 2)
            mean = tuple(np.rint(xy.mean(axis=0)).astype(int))
            cv2.line(canvas, center, mean, (0, 180, 0), 1)
            cv2.putText(canvas, 'ID '+str(corners.marker_id), tuple(points[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2, cv2.LINE_AA)
    lines = [f'camera stamp: {source:.6f}']
    if (status is not None and status_image_stamp(status) is not None
            and abs(status_image_stamp(status)-source) <= tolerance):
        state = str(status.get('state', '--'))
        labels = {'跟踪中': 'tracking', '只看不动': 'motion disabled',
                  '丢失衰减': 'target lost: decay', '等待指定码的新图像': 'waiting for target',
                  '等待新云台IMU': 'waiting for fresh IMU', '云台IMU中断': 'IMU interrupted',
                  '本次试转时间结束': 'trial timeout', '本次转动达到角度范围': 'rotation limit'}
        state = labels.get(state, state).encode('ascii', 'backslashreplace').decode()[:95]
        lines.append('state: '+state)
        for key, label in (('error_px', 'raw px'), ('filtered_error_deg', 'filtered deg'),
                           ('control_error_deg', 'control deg')):
            values = pair(status, key)
            lines.append(label+': '+', '.join(number_text(value) for value in values))
        lines.append('omega yaw/pitch rad/s: '+', '.join(number_text(status.get(key))
                     for key in ('yaw_omega', 'pitch_omega')))
        lines.append('lost frames: '+str(status.get('lost_count', '--')))
    else:
        lines.append('status not matched to this image')
    for index, line in enumerate(lines):
        at = (12, 25+24*index)
        cv2.putText(canvas, line, at, cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, line, at, cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


class StatusCSV:
    """Create a new CSV only. A disk failure disables logging, not tracking."""
    columns = ('source_stamp', 'status_stamp', 'error_px_x', 'error_px_y',
               'filtered_yaw_deg', 'filtered_pitch_deg', 'control_yaw_deg',
               'control_pitch_deg', 'lost_count', 'yaw_omega', 'pitch_omega', 'state')

    def __init__(self, path, warn):
        self.file = None
        self.warn = warn
        if not path:
            return
        try:
            self.file = Path(path).expanduser().open('x', newline='', encoding='utf-8')
            self.writer = csv.writer(self.file)
            self.writer.writerow(self.columns)
            self.file.flush()
        except (OSError, ValueError, csv.Error) as exc:
            self._failed(exc)

    def _failed(self, exc):
        self.warn('Debug CSV disabled: '+str(exc))
        self.close()

    def write(self, status):
        if self.file is None:
            return
        row = (status_image_stamp(status), finite_number(status.get('status_stamp')),
               *pair(status, 'error_px'), *pair(status, 'filtered_error_deg'),
               *pair(status, 'control_error_deg'), finite_number(status.get('lost_count')),
               finite_number(status.get('yaw_omega')), finite_number(status.get('pitch_omega')),
               str(status.get('state', '')))
        try:
            self.writer.writerow(row)
            self.file.flush()
        except (OSError, ValueError, csv.Error) as exc:
            self._failed(exc)

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
            self.file = None


class TrackingDebugNode(Node):
    def __init__(self):
        super().__init__('tracking_debug_node')
        self.declare_parameter('camera_config', '')
        self.declare_parameter('csv_path', '')
        self.camera = load_yaml(str(self.get_parameter('camera_config').value))['camera']
        for key in ('cx', 'cy', 'width', 'height'):
            if not math.isfinite(float(self.camera[key])):
                raise ValueError('invalid camera '+key)
        self.image = None
        self.corners = None
        self.status = None
        self.last_source = None
        self.last_warning = None
        self.csv = StatusCSV(str(self.get_parameter('csv_path').value), self.get_logger().warning)
        sensor = sensor_qos(1)
        self.pub = self.create_publisher(Image, 'debug_image', sensor)
        self.create_subscription(Image, '/image_raw', self.on_image, sensor)
        self.create_subscription(MarkerCorners, 'marker_corners', self.on_corners, sensor)
        self.create_subscription(String, 'status', self.on_status, 10)
        self.create_timer(.1, self.render)

    def on_image(self, message):
        self.image = message

    def on_corners(self, message):
        self.corners = message

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
            self.status = status
            self.csv.write(status)
        except (TypeError, ValueError):
            pass

    def render(self):
        if self.image is None:
            return
        try:
            message = self.image
            source = stamp_seconds(message.header)
            age = self.get_clock().now().nanoseconds*1e-9-source
            if source == self.last_source or not -.03 <= age <= .3:
                return  # Never stamp an old camera frame as if it were fresh.
            self.last_source = source
            canvas = draw_overlay(decode_image(message), self.camera, source,
                                  self.corners, self.status)
            result = Image()
            result.header = copy.deepcopy(message.header)
            result.height, result.width = canvas.shape[:2]
            result.encoding = 'bgr8'
            result.is_bigendian = 0
            result.step = result.width*3
            result.data = canvas.tobytes()
            self.pub.publish(result)
            self.last_warning = None
        except (AttributeError, TypeError, ValueError, cv2.error) as exc:
            text = str(exc)
            if text != self.last_warning:
                self.get_logger().warning('Debug image skipped: '+text)
                self.last_warning = text

    def destroy_node(self):
        self.csv.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
