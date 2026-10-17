"""离线回归：真实PolygonStamped形状、指令门控、日志格式与图像stamp。不运行ROS。"""
import ast
import importlib.util
import sys
import math
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
root=Path(__file__).resolve().parents[1]
for name in ('gimbal_perception','gimbal_serial_bridge','gimbal_common'):
    sys.path.insert(0,str(root/'src'/name))
from gimbal_perception.marker_source import SingleTopicMarkerSource
from gimbal_serial_bridge.command_gate import CommandGate
from gimbal_common.log_utils import _FormatLogger
from gimbal_common.protocol import encode_omega, frame_ok

def extract_method(path,name,env):
    tree=ast.parse(path.read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),env)
    return env[name]

class Regression(unittest.TestCase):
    def test_adapter_tick_maps_two_joint_rates_to_twist_and_wire_payload(self):
        # Execute the production timer body with a message double: no ROS,
        # physical port, or actuator. Independent expected bytes catch a swap
        # of yaw/pitch or an accidental degree/radian conversion.
        class TwistDouble:
            def __init__(self):
                self.angular = NS(x=0., y=0., z=0.)
                self.linear = NS(x=0., y=0., z=0.)
        clock = NS(value=.6)
        fn = extract_method(root/'src/gimbal_serial_bridge/gimbal_serial_bridge/hardware_adapter_node.py',
                            'tick', {'time': NS(monotonic=lambda: clock.value),
                                     'Twist': TwistDouble})
        gate = CommandGate(); gate.enable(True, 0.)
        for k in range(31): gate.imu(k * .02)
        gate.command(.4, -.2, True, .6)
        sent = []
        adapter = NS(gate=gate, get_parameter=lambda name: NS(value=True),
                     cmd_pub=NS(publish=sent.append))
        fn(adapter)
        self.assertEqual((sent[-1].angular.z, sent[-1].angular.y), (0., 0.))
        clock.value=.62; gate.imu(.62); gate.command(.4, -.2, True, .62)
        fn(adapter)
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

    def test_real_polygon_shape(self):
        stamp=NS(sec=5,nanosec=123)
        msg=NS(header=NS(frame_id='camera_optical_frame',stamp=stamp),
               polygon=NS(points=[NS(x=x,y=y) for x,y in ((1,2),(3,2),(3,4),(1,4))]))
        result=SingleTopicMarkerSource().on_polygon(msg,marker_id=3,t_mono=1.)
        self.assertIsNotNone(result)
        self.assertEqual(result.marker_id,3)
        np.testing.assert_array_equal(result.corners_uv,[[1,2],[3,2],[3,4],[1,4]])
        msg.polygon.points=[]
        self.assertIsNone(SingleTopicMarkerSource().on_polygon(msg))
    def test_legacy_points_still_supported(self):
        self.assertIsNotNone(SingleTopicMarkerSource().on_polygon(
            NS(points=[NS(x=1,y=2)]*4)))
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
        path=root/'src/gimbal_serial_bridge/gimbal_serial_bridge/hardware_adapter_node.py'
        env={'math':math,'time':NS(monotonic=lambda:1.1)}
        fn=extract_method(path,'on_cmd',env)
        obj=NS(gate=CommandGate(),last_cmd_stamp=None,
               get_clock=lambda:NS(now=lambda:NS(nanoseconds=1100000000)))
        msg=NS(header=NS(stamp=NS(sec=1,nanosec=50000000),frame_id='gimbal_link'),
               yaw_omega=1.,pitch_omega=-1.,valid=True)
        fn(obj,msg);self.assertEqual(obj.gate.cmd,(1.,-1.))
        fn(obj,msg);self.assertEqual(obj.gate.cmd,(0.,0.))
        msg.header.stamp=NS(sec=0,nanosec=0);fn(obj,msg)
        self.assertEqual(obj.gate.cmd,(0.,0.))
    def test_mcu_reset_latches(self):
        g=CommandGate();g.enable(True,0.);g.reset()
        self.assertTrue(g.fault)
        g.imu(.01);g.command(1.,1.,True,.01)
        self.assertIsNone(g.output(.01))
        g.enable(False,.02);g.enable(True,.02)
        self.assertFalse(g.fault)
        self.assertGreaterEqual(g.ready_at,.62)
    def test_logger_native_single_argument(self):
        messages=[]
        native=NS(info=lambda message:messages.append(message))
        _FormatLogger(native).info('fx=%.2f, id=%d',1778.23212,3)
        self.assertEqual(messages,['fx=1778.23, id=3'])
    def test_pnp_output_preserves_image_stamp(self):
        path=root/'src/gimbal_perception/gimbal_perception/marker_pnp_node.py'
        published=[]
        env={'MarkerObservation':lambda:NS(header=NS()),'np':np}
        fn=extract_method(path,'_publish',env)
        stamp=NS(sec=7,nanosec=456)
        obj=NS(_input_stamp=stamp,_pub=NS(publish=published.append))
        fn(obj,3,np.array([1,2,3]),np.eye(3),3.,.1,True)
        self.assertIs(published[0].header.stamp,stamp)
    def test_late_target_id_waits_and_clears_cached_image(self):
        path=root/'src/gimbal_perception/gimbal_perception/aruco_detector_node.py'
        env={'Any':object}
        fn=extract_method(path,'_on_timer',env)
        obj=NS(_ok=True,_target_digit=0,_params={},_latest=object(),
               _publish=lambda *args:None, get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=7,nanosec=0))),
               get_parameter=lambda key:NS(value=-1),_log=NS(info=lambda msg:None))
        fn(obj)
        self.assertEqual(obj._target_digit,-1);self.assertIsNone(obj._latest)

if __name__=='__main__':unittest.main()
