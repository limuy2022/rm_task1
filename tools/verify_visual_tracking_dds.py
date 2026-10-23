"""真实 launch/进程/DDS：合成图像→检测/PnP/图像控制→C++桥→新建PTY。

只在独立ROS域中运行；串口仅使用本进程新建的PTY，运动开启只作用于该虚拟端口。
覆盖默认静默、两轴方向、相机断流停转、恢复和参数关闭；不代表真实电机制动或NUC实时性。
"""
from pathlib import Path
from array import array
import json
import math
import os
import signal
import struct
import subprocess
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gimbal_interfaces.msg import MarkerObservation

from verify_perception_offline import CAMERA, render_marker

ROOT = Path(__file__).resolve().parents[1]


def checksum(data):
    total = cumulative = 0
    for byte in data:
        total = (total + byte) & 255
        cumulative = (cumulative + total) & 255
    return bytes((total, cumulative))


class Probe(Node):
    def __init__(self, master, launch):
        super().__init__('tracking_dds_probe')
        self.master, self.launch = master, launch
        self.seq = 0
        self.last_imu = self.last_image = 0.
        self.send_images = True
        self.serial = bytearray()
        self.commands, self.twists, self.observations, self.statuses = [], [], [], []
        self.imu_count = 0
        self.image = render_marker(3, (.12, .06, 2.))
        # ROS消息对bytes逐元素校验；预建原生uint8数组，避免测试源自己阻塞IMU喂送。
        self.image_data = array('B', self.image.tobytes())
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.create_subscription(Twist, '/team47_tracking/cmd_vel', self.twists.append, 10)
        self.create_subscription(MarkerObservation, '/team47_tracking/marker_cam', self.observations.append, qos_profile_sensor_data)
        self.create_subscription(String, '/team47_tracking/status', self.statuses.append, 10)
        self.create_subscription(Imu, '/red_standard_robot1/gimbal', self.on_imu, 10)
        self.parameters = self.create_client(SetParameters, '/team47_tracking/visual_servo_node/set_parameters')

    def on_imu(self, message):
        self.imu_count += 1

    def pump(self):
        if self.launch.poll() is not None:
            raise RuntimeError('实际 launch 提前退出，见 visual_tracking_dds.log')
        now = time.monotonic()
        if now - self.last_imu >= .02:
            self.seq += 1
            data = b'\xaa\xff\x01\x1a' + struct.pack(
                '<HI3h3h4h', self.seq & 65535, (self.seq * 20000) & 0xffffffff,
                0, 0, 1365, 0, 0, 0, 32760, 0, 0, 0)
            os.write(self.master, data + checksum(data))
            self.last_imu = now
        if self.send_images and now - self.last_image >= 1. / 20:
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.height, message.width = self.image.shape
            message.encoding = 'mono8'; message.step = message.width
            message.data = self.image_data
            self.image_pub.publish(message)
            self.last_image = now
        rclpy.spin_once(self, timeout_sec=.002)
        try:
            self.serial.extend(os.read(self.master, 4096))
        except BlockingIOError:
            pass
        while len(self.serial) >= 14:
            frame = bytes(self.serial[:14])
            if frame[:4] != b'\xaa\xee\x30\x08' or checksum(frame[:12]) != frame[12:]:
                raise AssertionError('PTY收到坏帧或非0x30帧')
            yaw, pitch = struct.unpack('<ff', frame[4:12])
            assert math.isfinite(yaw) and math.isfinite(pitch)
            self.commands.append((time.monotonic(), yaw, pitch))
            del self.serial[:14]

    def wait(self, condition, timeout=12.):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() >= deadline:
                raise AssertionError('DDS场景等待超时')
            self.pump()

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= end, seconds + 1.)

    def arm(self, enabled):
        self.wait(self.parameters.service_is_ready)
        request = SetParameters.Request()
        request.parameters = [Parameter('motion_enabled', value=enabled).to_parameter_msg()]
        future = self.parameters.call_async(request)
        self.wait(future.done)
        assert all(result.successful for result in future.result().results)

    def check_stop(self, first):
        # 观察完整停止窗，包括有界宽限、三零帧和之后的静默。
        self.run_for(.8)
        commands = self.commands[first:]
        zero = next((i for i, (_, y, p) in enumerate(commands) if y == p == 0.), None)
        assert zero is not None, '没有实际收到零指令'
        assert len(commands[zero:]) == 3 and all(y == p == 0. for _, y, p in commands[zero:]), commands
        assert not self.serial, 'PTY遗留不完整帧'


def main():
    # 为本次测试建立独立DDS域；实际launch与probe继承同一环境。
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 80)
    output = ROOT / 'build/offline'
    output.mkdir(parents=True, exist_ok=True)
    master, slave = os.openpty()
    # launch 打开前就关闭PTY的回显，不能把注入的IMU回显误读成桥的输出。
    tty.setraw(slave)
    os.set_blocking(master, False)
    port = os.ttyname(slave)
    probe = launch = None
    report = dict(passed=False, kind='real_launch_dds_pty', hardware_accessed=False,
                  ros_domain_id=os.environ['ROS_DOMAIN_ID'], checks={},
                  limitations=['合成静态图像、恒定IMU；未模拟电机动力学', '未验证NUC的80ms提交截止时间'])
    try:
        with (output / 'visual_tracking_dds.log').open('w') as log:
            launch = subprocess.Popen([
                'ros2', 'launch', 'gimbal_tracker_bringup', 'visual_tracking.launch.py',
                'target_id:=3', 'with_bridge:=true', 'port:=' + port],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = Probe(master, launch)
            probe.wait(lambda: probe.imu_count >= 10 and len(probe.statuses) >= 3
                       and sum(m.valid for m in probe.observations) >= 3)
            probe.run_for(.3)
            assert not probe.twists and not probe.commands, '默认关闭状态发出了速度'
            report['checks']['disabled_is_silent'] = True

            probe.arm(True)
            probe.wait(lambda: len(probe.commands) >= 8)
            assert all(y < 0 < p and max(abs(y), abs(p)) <= .250001
                       for _, y, p in probe.commands), probe.commands
            report['checks']['image_to_pnp_and_serial_direction'] = True
            for message in probe.observations:
                if message.valid:
                    assert abs(message.x - .12) < .03 and abs(message.y - .06) < .03 and abs(message.z - 2.) < .06

            first = len(probe.commands)
            probe.send_images = False
            probe.check_stop(first)
            report['checks']['camera_stall_sends_three_zero_frames_then_silence'] = True

            probe.send_images = True
            first = len(probe.commands)
            probe.wait(lambda: any(y < 0 < p for _, y, p in probe.commands[first:]))
            report['checks']['fresh_images_resume_tracking'] = True

            first = len(probe.commands)
            probe.arm(False)
            probe.check_stop(first)
            report['checks']['parameter_disarm_stops_actual_serial'] = True
            # 带着运动状态请求SIGINT，检验finally停转时context仍然有效。
            probe.arm(True)
            first = len(probe.commands)
            probe.wait(lambda: any(y < 0 < p for _, y, p in probe.commands[first:]))
            report.update(imu_received=probe.imu_count, pnp_observations=len(probe.observations),
                          serial_commands=len(probe.commands), passed=True)
    except Exception as exc:
        report['error'] = repr(exc)
        if probe is not None:
            report['last_statuses'] = [json.loads(m.data) for m in probe.statuses[-5:]]
    finally:
        if launch is not None:
            # 整个launch进程组退出，不能留下持有PTY的桥进程。
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
            try:
                report['launch_exit_code'] = launch.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
                report['launch_exit_code'] = launch.returncode
            report['passed'] = report['passed'] and report['launch_exit_code'] == 0
            report['clean_child_shutdown'] = 'process has died' not in (output / 'visual_tracking_dds.log').read_text()
            report['passed'] = report['passed'] and report['clean_child_shutdown']
        if probe is not None: probe.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        os.close(master); os.close(slave)
    (output / 'visual_tracking_dds.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
