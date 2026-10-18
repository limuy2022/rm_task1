"""Run the real visual-servo methods with ROS message/clock stand-ins.

No ROS, serial device, motor or firmware is used.  AST loading removes ROS
imports, not controller logic; the full production class (including __init__)
and production image_error/AxisPID execute unchanged.
"""

import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gimbal_control'))
from gimbal_control.visual_pid import AxisPID, ErrorConditioner


class Clock:
    def __init__(self):
        self.ros, self.mono = 100., 10.

    def advance(self, dt):
        self.ros += dt
        self.mono += dt


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0., y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=0.)


class String:
    def __init__(self):
        self.data = ''


def _header(clock, source=None, frame='camera_optical_frame'):
    ns = round((clock.ros if source is None else source) * 1e9)
    sec, nanosec = divmod(ns, 1_000_000_000)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
                           frame_id=frame)


def corners_message(clock, center_x, center_y, marker_id=0, valid=True,
                    source=None):
    return SimpleNamespace(header=_header(clock, source), valid=valid,
        marker_id=marker_id,
        corners=[SimpleNamespace(x=center_x+dx, y=center_y+dy)
                 for dx, dy in ((-10, -10), (10, -10), (10, 10), (-10, 10))])


def imu_message(clock, q_wxyz=(1., 0., 0., 0.), source=None):
    return SimpleNamespace(header=_header(clock, source, 'gimbal_link'),
        orientation=SimpleNamespace(**dict(zip(('w', 'x', 'y', 'z'), q_wxyz))),
        orientation_covariance=[0.] * 9)


def create_harness(camera):
    """Return (real_node, clock, mutable_parameter_dict, published_Twists)."""
    clock, published = Clock(), []

    class HarnessNode:
        def __init__(self, name):
            self._params = {}
            self.log_errors, self.status_messages = [], []

        def declare_parameter(self, name, value):
            self._params[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self._params[name])

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(
                nanoseconds=round(clock.ros * 1e9)))

        def get_logger(self):
            return SimpleNamespace(info=lambda message: None,
                                   error=self.log_errors.append)

        def create_publisher(self, message_type, topic, qos):
            destination = published if message_type is Twist else self.status_messages
            return SimpleNamespace(publish=lambda message:
                                   destination.append(copy.deepcopy(message)))

        def create_subscription(self, *args, **kwargs):
            return None

        def create_timer(self, *args, **kwargs):
            return None

    tree = ast.parse((ROOT / 'src/gimbal_control/gimbal_control/visual_servo_node.py').read_text())
    definitions = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                   and node.name in ('stamp_seconds', 'image_error', 'VisualServoNode')]
    for node in definitions:
        if isinstance(node, ast.ClassDef):
            node.bases = [ast.Name(id='HarnessNode', ctx=ast.Load())]
    module = ast.fix_missing_locations(ast.Module(body=definitions, type_ignores=[]))
    namespace = dict(HarnessNode=HarnessNode, json=json, math=math, cv2=cv2, np=np,
        time=SimpleNamespace(monotonic=lambda: clock.mono), AxisPID=AxisPID, ErrorConditioner=ErrorConditioner,
        Twist=Twist, String=String, Imu=SimpleNamespace, MarkerCorners=SimpleNamespace,
        UInt64=SimpleNamespace, load_yaml=lambda path: {'camera': copy.deepcopy(camera)},
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort', RELIABLE='reliable'),
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL='transient_local'),
        # 节点 __init__ 经 gimbal_common.qos_utils 构造 QoS；harness 的发布/订阅
        # 忽略 qos 实参，这里给出可调用的桩即可。
        sensor_qos=lambda depth=10: SimpleNamespace(depth=depth),
        latched_qos=lambda depth=1: SimpleNamespace(depth=depth))
    exec(compile(module, 'production_visual_servo_node.py', 'exec'), namespace)
    node = namespace['VisualServoNode']()
    return node, clock, node._params, published


CAMERA = {'width': 1280, 'height': 960, 'fx': 800., 'fy': 800.,
          'cx': 640., 'cy': 480., 'distortion': [0., 0., 0., 0., 0.]}


def _armed_tracking(camera=CAMERA, center=(700., 520.), **parameters):
    node, clock, params, sent = create_harness(camera)
    params.update(parameters)
    params['motion_enabled'] = True
    node.tick()  # arm at the current ROS time; pre-arm data is insufficient
    clock.advance(.02)
    node.on_imu(imu_message(clock))
    node.on_corners(corners_message(clock, *center))
    node.tick()
    assert sent, 'fresh image and IMU should allow a command'
    return node, clock, params, sent


def _is_zero(message):
    return message.angular.z == 0. and message.angular.y == 0.


def main():
    checks = []
    node, clock, params, sent = create_harness(CAMERA)
    for _ in range(8):
        clock.advance(.02)
        node.on_imu(imu_message(clock))
        node.on_corners(corners_message(clock, 700., 520.))
        node.tick()
    assert sent == []
    checks.append('disabled_start_never_publishes_Twist')

    # An otherwise fresh sample received before arming must not start motion.
    params['motion_enabled'] = True
    clock.advance(.01)
    node.tick()
    assert not sent
    clock.advance(.01)
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert not sent, 'must wait for an IMU newer than arming, not cached IMU'
    clock.advance(.01)
    node.on_imu(imu_message(clock))
    node.tick()
    assert all(_is_zero(message) for message in sent)
    # Waiting for the IMU clears PID state; the next fresh image supplies a
    # fresh correction after both inputs are available.
    clock.advance(.01)
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert sent and sent[-1].angular.z < 0 and sent[-1].angular.y > 0
    checks.append('arming_requires_post_arm_image_and_IMU')
    checks.append('image_right_down_commands_negative_yaw_positive_pitch')

    node, clock, params, sent = _armed_tracking(center=(650., 480.), ki_yaw=.5)
    before = node.pid[0]._integral_output
    same = corners_message(clock, 650., 480.)
    node.on_corners(same)
    node.on_corners(corners_message(clock, 650., 480., source=clock.ros-.001))
    for _ in range(2):
        clock.advance(.02)
        node.on_imu(imu_message(clock))
        node.tick()
    assert node.pid[0]._integral_output == before
    clock.advance(.01)
    node.on_corners(corners_message(clock, 650., 480.))
    assert node.pid[0]._integral_output < before < 0
    checks.append('PID_integrates_only_new_image_stamps_not_timer_or_duplicates')

    node, clock, params, sent = _armed_tracking()
    before = len(sent)
    node.on_corners(corners_message(clock, 700., 520., valid=False))
    for _ in range(7):
        clock.advance(.02)
        node.on_imu(imu_message(clock))
        node.tick()
    assert len(sent) == before + 3
    assert all(_is_zero(message) for message in sent[before:])
    checks.append('invalid_image_sends_three_stop_frames_then_silence')

    for label, mutate in (
            ('stale', lambda n, c: n.on_corners(corners_message(c, 700., 520., source=c.ros-.5))),
            ('crossed', lambda n, c: n.on_corners(SimpleNamespace(
                **{**vars(corners_message(c, 700., 520.)), 'corners': [
                    SimpleNamespace(x=x, y=y) for x, y in
                    ((680, 500), (720, 540), (720, 500), (680, 540))]}))),
            ('out_of_bounds', lambda n, c: n.on_corners(corners_message(c, 1275., 520.)))):
        node, clock, params, sent = _armed_tracking()
        clock.advance(.01)
        mutate(node, clock)
        clock.advance(.02)
        node.on_imu(imu_message(clock))
        node.tick()
        assert _is_zero(sent[-1]) and node.observation is None, label
    checks.append('stale_crossed_and_out_of_bounds_images_stop')

    node, clock, params, sent = _armed_tracking()
    clock.advance(.16)
    node.on_imu(imu_message(clock))
    node.tick()
    assert _is_zero(sent[-1]) and not node.fault
    checks.append('camera_silence_times_out_even_without_invalid_callback')

    node, clock, params, sent = _armed_tracking()
    clock.advance(.2)
    node.tick()
    assert node.fault == '云台IMU中断'
    clock.advance(.01)
    node.on_imu(imu_message(clock))
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert node.fault and _is_zero(sent[-1])
    params['motion_enabled'] = False
    clock.advance(.02)
    node.tick()
    assert not node.armed and not node.fault
    checks.append('IMU_loss_latches_until_disabled')

    node, clock, params, sent = _armed_tracking()
    node.on_epoch(SimpleNamespace(data=1))
    node.on_epoch(SimpleNamespace(data=2))
    clock.advance(.02)
    node.on_imu(imu_message(clock))
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert node.fault and _is_zero(sent[-1])
    checks.append('reset_epoch_change_latches_stop')

    node, clock, params, sent = _armed_tracking()
    params['run_seconds'] = 15.
    clock.advance(15.)
    node.on_imu(imu_message(clock))
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert node.fault == '本次试转时间结束' and _is_zero(sent[-1])
    checks.append('15_second_trial_limit_latches_stop')

    node, clock, params, sent = _armed_tracking()
    clock.advance(.02)
    node.on_imu(imu_message(clock, (math.cos(.4/2), 0., 0., math.sin(.4/2))))
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert node.fault == '本次转动达到角度范围' and _is_zero(sent[-1])
    checks.append('relative_attitude_range_latches_stop')

    node, clock, params, sent = _armed_tracking()
    clock.advance(.02)
    node.on_imu(imu_message(clock, (-1., 0., 0., 0.)))
    node.on_corners(corners_message(clock, 700., 520.))
    node.tick()
    assert not node.fault and not _is_zero(sent[-1])
    checks.append('equivalent_negative_quaternion_does_not_trigger_range')

    node, clock, params, sent = _armed_tracking(axes='yaw')
    assert sent[-1].angular.z < 0 and sent[-1].angular.y == 0
    node, clock, params, sent = _armed_tracking(axes='pitch', pitch_sign=-1.)
    assert sent[-1].angular.z == 0 and sent[-1].angular.y < 0
    checks.append('axis_selection_and_explicit_sign_apply_to_actual_Twist')

    print(json.dumps({'passed': True, 'checks': checks,
        'production_logic': ['full VisualServoNode class from AST', 'image_error', 'AxisPID'],
        'substitutions': ['ROS base/messages/QoS/publishers', 'ROS and monotonic clocks', 'camera YAML read'],
        'physical_devices_opened': 0, 'ros_nodes_started': 0,
        'limitations': ['ROS graph/QoS and real camera/firmware/motor response remain unverified']},
        ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
