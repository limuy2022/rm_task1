"""42桥的无串口适配器；不发布TF，固件使能状态未知。"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from gimbal_interfaces.msg import GimbalState, GimbalOmegaCmd
from gimbal_common.frames import quat_to_mat, decompose_yaw_pitch
from .command_gate import CommandGate

class HardwareAdapter(Node):
    def __init__(self):
        super().__init__('hardware_adapter_node')
        self.declare_parameter('control_enabled',False)
        self.declare_parameter('imu_topic','/red_standard_robot1/gimbal')
        self.declare_parameter('cmd_topic','/offboardlink/cmd_vel_gimbal')
        self.gate=CommandGate()
        self.received=0
        self.epoch=None
        self.last_cmd_stamp=None
        self.state_pub=self.create_publisher(GimbalState,'/gimbal/state',10)
        self.cmd_pub=self.create_publisher(Twist,str(self.get_parameter('cmd_topic').value),10)
        self.create_subscription(Imu,str(self.get_parameter('imu_topic').value),self.on_imu,
                                 QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(GimbalOmegaCmd,'/gimbal/cmd_omega',self.on_cmd,
                                 QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(UInt64,'/task1/gimbal_reset_epoch',self.on_epoch,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.02,self.tick)
        self.get_logger().warning('指令转发默认关闭；只适配消息，不知道固件enabled/CAN/机械关节角')

    def on_epoch(self,msg):
        if self.epoch is not None and msg.data != self.epoch:
            self.gate.reset()
            self.get_logger().error('MCU已复位：停止转发；原world对齐失效，重新标定后方可重新启用')
        self.epoch=msg.data

    def on_imu(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if msg.header.frame_id != 'gimbal_link' or not math.isfinite(age) or age < -.01 or age > self.gate.imu_timeout:
            return  # 可靠队列中的旧消息不能刷新新鲜度
        q=[msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z]
        gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        accel=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        if not all(math.isfinite(v) for v in q+gyro+accel):
            return
        norm=math.sqrt(sum(v*v for v in q))
        if norm < .5:
            return
        q=[v/norm for v in q]
        self.gate.imu(time.monotonic())
        yaw,pitch=decompose_yaw_pitch(quat_to_mat(q))
        state=GimbalState();state.header=msg.header
        state.quat=q;state.gyro=gyro;state.accel=accel
        # 惯性姿态；不能用作底盘转动时的机械关节限位。
        state.yaw=float(yaw);state.pitch=float(pitch)
        state.omega_yaw=float('nan');state.omega_pitch=float('nan')
        # 没有编码器/固件反馈，不能冒充已知关节速度、使能或ACTIVE状态。
        state.enabled=False;state.link_state=0
        self.state_pub.publish(state)
        self.received+=1

    def on_cmd(self,msg):
        try:
            source=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            age=self.get_clock().now().nanoseconds*1e-9-source
            if (msg.header.frame_id!='gimbal_link' or not math.isfinite(age)
                    or age < -.01 or age > self.gate.cmd_timeout
                    or (self.last_cmd_stamp is not None and source<=self.last_cmd_stamp)):
                self.gate.command(0.,0.,False,time.monotonic());return
            self.last_cmd_stamp=source
            self.gate.command(float(msg.yaw_omega),float(msg.pitch_omega),bool(msg.valid),time.monotonic())
        except (TypeError,ValueError):  # 无效角速度必须送入停止门控，不能把坏指令发给电机。
            self.gate.command(0.,0.,False,time.monotonic())

    def tick(self):
        now=time.monotonic()
        self.gate.enable(bool(self.get_parameter('control_enabled').value),now)
        before=self.gate.fault
        value=self.gate.output(now)
        if self.gate.fault and not before:
            self.get_logger().error('IMU/指令过期，停止转发；恢复后需先禁用再显式启用')
        if value is None:
            return
        cmd=Twist();cmd.angular.z=float(value[0]);cmd.angular.y=float(value[1])
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args);node=HardwareAdapter()
    try:
        rclpy.spin(node)
    finally:
        # 不再保活，依赖协议0.5秒看门狗；现场操作必须知道下位机停止有延迟。
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
