"""离线回归（ROS 2 Humble 容器内直跑）：真实 MarkerCorners/Twist/GimbalOmegaCmd/
MarkerObservation 消息、指令门控、日志格式与图像stamp；生产节点类原样调用，
不 spin、不实机、不物理串口。"""
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point32, PolygonStamped
from team43_interfaces.msg import GimbalOmegaCmd, MarkerCorners

root=Path(__file__).resolve().parents[3]
for name in ('team43_perception','team43_gimbal_io','team43_common'):
    sys.path.insert(0,str(root/'src'/name))
import team43_gimbal_io.hardware_adapter_node as hw_module
from team43_perception.marker_source import SingleTopicMarkerSource
from team43_gimbal_io.command_gate import CommandGate
from team43_common.protocol import encode_omega, frame_ok

def bare_adapter(**fields):
    # Real HardwareAdapter instance with the constructor bypassed and only the
    # dependencies tick()/on_cmd() touch injected (gate/clock/params/publisher).
    adapter = hw_module.HardwareAdapter.__new__(hw_module.HardwareAdapter)
    for key, value in fields.items():
        setattr(adapter, key, value)
    return adapter

class Regression(unittest.TestCase):
    def test_adapter_tick_maps_two_joint_rates_to_twist_and_wire_payload(self):
        # Run the production timer body on a real adapter instance: no ROS
        # executor, physical port or actuator. Independent expected bytes catch
        # a swap of yaw/pitch or an accidental degree/radian conversion.
        clock = NS(value=.6)
        gate = CommandGate(); gate.enable(True, 0.)
        for k in range(31): gate.imu(k * .02)
        gate.command(.4, -.2, True, .6)
        sent = []
        adapter = bare_adapter(gate=gate, get_parameter=lambda name: NS(value=True),
                               cmd_pub=NS(publish=sent.append))
        with mock.patch.object(hw_module.time, 'monotonic', return_value=clock.value):
            adapter.tick()
        self.assertEqual((sent[-1].angular.z, sent[-1].angular.y), (0., 0.))
        clock.value=.62; gate.imu(.62); gate.command(.4, -.2, True, .62)
        with mock.patch.object(hw_module.time, 'monotonic', return_value=clock.value):
            adapter.tick()
        message = sent[-1]
        self.assertAlmostEqual(message.angular.z, .3)
        self.assertAlmostEqual(message.angular.y, -.2)
        self.assertEqual((message.angular.x, message.linear.x, message.linear.y,
                          message.linear.z), (0., 0., 0., 0.))
        frame = encode_omega(message.angular.z, message.angular.y)
        self.assertEqual(frame[:4], bytes.fromhex('aa ee 30 08'))
        self.assertEqual(frame[4:12], struct.pack('<ff', .3, -.2))
        self.assertEqual(len(frame), 14)
        self.assertTrue(frame_ok(frame, 8))

    def test_marker_corners_carries_actual_id(self):
        msg=MarkerCorners()
        msg.marker_id=3
        msg.valid=True
        msg.header.frame_id='camera_optical_frame'
        msg.header.stamp=Time(sec=5,nanosec=123)
        msg.corners=[Point32(x=float(x),y=float(y)) for x,y in ((1,2),(3,2),(3,4),(1,4))]
        result=SingleTopicMarkerSource().on_polygon(msg,t_mono=1.)
        self.assertIsNotNone(result)
        self.assertEqual(result.marker_id,3)
        np.testing.assert_array_equal(result.corners_uv,[[1,2],[3,2],[3,4],[1,4]])
        msg.corners=[]
        self.assertIsNone(SingleTopicMarkerSource().on_polygon(msg))
    def test_legacy_points_do_not_invent_id_and_validity(self):
        with self.assertRaises(AttributeError):
            SingleTopicMarkerSource().on_polygon(PolygonStamped())
    def test_disabled_and_before_imu_are_silent(self):
        g=CommandGate();g.command(1.,1.,True,0.)
        self.assertIsNone(g.output(0.))
        g.enable(True,0.);g.command(1.,1.,True,.7)
        self.assertIsNone(g.output(.7))
    def test_quiet_limit_slew_and_fault(self):
        g=CommandGate();g.imu(0.);g.enable(True,0.)
        for k in range(1,31):
            t=k*.02;g.imu(t);g.command(100.,-100.,True,t)
            value=g.output(t)
            if t<.6:self.assertIsNone(value)
        self.assertEqual(value,(0.,0.))
        g.imu(.62);g.command(100.,-100.,True,.62)
        y,p=g.output(.62);self.assertAlmostEqual(y,.3);self.assertAlmostEqual(p,-.3)
        self.assertIsNone(g.output(.8));self.assertTrue(g.fault)
        g.imu(.81);g.command(1.,1.,True,.81)
        self.assertIsNone(g.output(.81)) # 不自动恢复
        g.enable(False,.81);self.assertFalse(g.fault)
    def test_stale_command_latches(self):
        g=CommandGate();g.imu(0.);g.enable(True,0.)
        for k in range(1,31):g.imu(k*.02)
        g.command(1.,1.,True,.6);g.output(.6);g.imu(.75)
        self.assertIsNone(g.output(.75));self.assertTrue(g.fault)
    def test_nonfinite_command_becomes_zero(self):
        g=CommandGate();g.command(float('nan'),1.,True,0.)
        self.assertEqual(g.cmd,(0.,0.))
    def test_invalid_command_stops_without_previous_slew_tail(self):
        g=CommandGate();g.imu(0.);g.enable(True,0.)
        for k in range(1,50):
            t=k*.02;g.imu(t);g.command(2.,-2.,True,t);g.output(t)
        self.assertGreater(abs(g.previous[0]),1.)
        g.command(100.,100.,False,1.);g.imu(1.)
        self.assertEqual(g.output(1.),(0.,0.))
    def test_adapter_rejects_old_source_stamp_and_duplicate(self):
        adapter=bare_adapter(gate=CommandGate(),last_cmd_stamp=None,
                             get_clock=lambda:NS(now=lambda:NS(nanoseconds=1100000000)))
        msg=GimbalOmegaCmd()
        msg.header.stamp=Time(sec=1,nanosec=50000000)
        msg.header.frame_id='gimbal_link'
        msg.yaw_omega=1.;msg.pitch_omega=-1.;msg.valid=True
        adapter.on_cmd(msg);self.assertEqual(adapter.gate.cmd,(1.,-1.))
        adapter.on_cmd(msg);self.assertEqual(adapter.gate.cmd,(0.,0.))
        msg.header.stamp=Time(sec=0,nanosec=0);adapter.on_cmd(msg)
        self.assertEqual(adapter.gate.cmd,(0.,0.))
    def test_mcu_reset_latches(self):
        g=CommandGate();g.enable(True,0.);g.reset()
        self.assertTrue(g.fault)
        g.imu(.01);g.command(1.,1.,True,.01)
        self.assertIsNone(g.output(.01))
        g.enable(False,.02);g.enable(True,.02)
        self.assertFalse(g.fault)
        self.assertGreaterEqual(g.ready_at,.62)
    def test_pnp_output_preserves_image_stamp(self):
        from team43_perception.marker_pnp_node import MarkerPnpNode
        published=[]
        pnp=MarkerPnpNode.__new__(MarkerPnpNode)
        stamp=Time(sec=7,nanosec=456)
        pnp._input_stamp=stamp;pnp._pub=NS(publish=published.append)
        pnp._publish(3,np.array([1,2,3]),np.eye(3),3.,.1,True)
        self.assertEqual(published[0].header.stamp,stamp)
    def test_late_target_id_waits_and_clears_cached_image(self):
        from team43_perception.aruco_detector_node import ArucoDetectorNode
        detector=ArucoDetectorNode.__new__(ArucoDetectorNode)
        detector._ok=True;detector._target_digit=0;detector._params={}
        detector._latest=object()
        detector._publish=lambda *args:None
        detector.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=7,nanosec=0)))
        detector.get_parameter=lambda key:NS(value=-1)
        detector._log=NS(info=lambda msg:None)
        detector._on_timer()
        self.assertEqual(detector._target_digit,-1);self.assertIsNone(detector._latest)
