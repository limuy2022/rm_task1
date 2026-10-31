"""Image-stamped target estimation using TF2 and the existing CA filter.

The TF request is map <- camera_optical_frame at the image stamp, including the
actual camera offset and moving lever arms. Missing calibration/TF drops or
briefly queues a measurement; it never silently substitutes an identity pose.
ROS stamps drive estimation; monotonic time is only used for local diagnostics.
"""
from __future__ import annotations
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import UInt64
from team43_interfaces.msg import MarkerObservation, TargetState
from team43_common import frames
from team43_common.time_buffer import stamp_seconds
from team43_common.log_utils import get_logger, throttled
from team43_common.qos_utils import latched_qos, sensor_qos
from .ca_kalman import CAKalman
from .observation_builder import ObservationBuilder
from .stamped_tracker import StampedTracker


class TargetEstimatorNode(Node):
    def __init__(self):
        super().__init__('target_estimator_node')
        self._log=get_logger(self,'target_estimator')
        defaults={
            'estimation.jerk_std':1., 'estimation.predict_dt':.1,
            'estimation.max_dt':.5, 'estimation.nis_gate':16.81,
            'estimation.reinit_jump_m':1., 'estimation.p_max_trace':100.,
            'estimation.init_sigma_p':.05, 'estimation.init_sigma_v':1.,
            'estimation.init_sigma_a':5., 'estimation.rate_hz':100.,
            'estimation.cv_degrade_enable':False, 'estimation.cv_hold_s':.5,
            'estimation.track_fresh_s':.1, 'estimation.lost_timeout_s':.5,
            'estimation.max_observation_delay_s':.3,
            'estimation.geometry_confirmed':False,
            'estimation.map_frame':'map', 'estimation.camera_frame':'camera_optical_frame',
            'perception.r_min_sigma':.002,
        }
        for name,value in defaults.items(): self.declare_parameter(name,value)
        p=lambda name:self.get_parameter(name).value
        self._frame=str(p('estimation.map_frame'))
        self._camera_frame=str(p('estimation.camera_frame'))
        self._horizon=float(p('estimation.predict_dt'))
        kf=CAKalman(**{key:p('estimation.'+key) for key in (
            'jerk_std','max_dt','nis_gate','reinit_jump_m','p_max_trace',
            'init_sigma_p','init_sigma_v','init_sigma_a','cv_degrade_enable','cv_hold_s')})
        self._tracker=StampedTracker(kf=kf,
            max_observation_delay=p('estimation.max_observation_delay_s'),
            lost_timeout=p('estimation.lost_timeout_s'),fresh_timeout=p('estimation.track_fresh_s'))
        self._builder=ObservationBuilder(r_min_sigma=p('perception.r_min_sigma'))
        self._tf_buffer=Buffer(cache_time=Duration(seconds=5.))
        self._tf_listener=TransformListener(self._tf_buffer,self)
        self._pending=[];self._epoch=None;self._last_now=None
        self._n_observed=0;self._n_rejected=0
        sensor=sensor_qos(20)
        self._pub=self.create_publisher(TargetState,'/estimation/target_state',sensor)
        self.create_subscription(MarkerObservation,'/perception/marker_cam',self.on_marker,sensor)
        self.create_subscription(UInt64,'/task1/gimbal_reset_epoch',self.on_epoch,
            latched_qos(1))
        self.create_timer(1/max(1.,float(p('estimation.rate_hz'))),self.tick)
        self._log.info('按图像stamp查询完整TF；geometry_confirmed=false时不使用目标观测')

    def on_epoch(self,msg):
        if self._epoch is not None and msg.data != self._epoch:
            self._tracker.reset();self._pending.clear()
            # IMU reboot changes its world reference; operator must reconfirm TF.
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('estimation.geometry_confirmed',value=False)])
            self._log.error('MCU复位：清目标并取消坐标确认，需要重新对齐后显式确认')
        self._epoch=msg.data

    def on_marker(self,msg):
        try:
            if msg.header.frame_id != self._camera_frame:
                self._n_rejected+=1;return
            mid=int(msg.marker_id)
            if self._tracker.marker_id is not None and mid != self._tracker.marker_id:
                self._tracker.reset();self._pending.clear()
            if not bool(msg.valid): return
            if not bool(self.get_parameter('estimation.geometry_confirmed').value):return
            stamp_seconds(msg.header.stamp)
            self._pending.append(msg)
            self._pending.sort(key=lambda m:stamp_seconds(m.header.stamp))
            self._pending=self._pending[-32:]
            self._drain()
        except (TypeError,ValueError):
            # Invalid observation stamp/data cannot be sent into the estimator.
            self._n_rejected+=1

    def _drain(self):
        current=self.get_clock().now().nanoseconds*1e-9
        if self._last_now is not None and current < self._last_now-.01:
            self._tracker.reset();self._pending.clear()
        self._last_now=current
        remaining=[]
        for msg in self._pending:
            stamp=stamp_seconds(msg.header.stamp)
            if stamp-current > .01:
                # 图像戳领先本地钟多为跨机时钟偏移；拒收必须可见，否则整链"在跑但无观测"。
                throttled(self._log,'warning',
                    '观测stamp领先本地钟%.1fms被拒收(累计%d)；持续出现需检查与图像源的时钟同步'
                    %((stamp-current)*1e3,self._n_rejected),'future_stamp')
                self._n_rejected+=1;continue
            if current-stamp > self._tracker.max_delay:
                self._n_rejected+=1;continue
            try:
                tf=self._tf_buffer.lookup_transform(self._frame,self._camera_frame,
                    Time.from_msg(msg.header.stamp),timeout=Duration(seconds=0.))
            except TransformException:
                # TF may arrive after the image; queue only until the observation deadline.
                remaining.append(msg);continue
            tr=tf.transform
            q=np.array([tr.rotation.w,tr.rotation.x,tr.rotation.y,tr.rotation.z])
            if not np.all(np.isfinite(q)) or np.linalg.norm(q)<.5:
                self._n_rejected+=1;continue
            T=frames.compose_T(frames.quat_to_mat(q),
                [tr.translation.x,tr.translation.y,tr.translation.z])
            obs=self._builder.build_from_transform([msg.x,msg.y,msg.z],msg.cov_optical,T)
            if obs.valid and self._tracker.observe(stamp,obs.pos_map[:2],obs.R_map,
                                                  now=current,marker_id=int(msg.marker_id)):
                self._n_observed+=1
            else:self._n_rejected+=1
        self._pending=remaining

    def tick(self):
        confirmed=bool(self.get_parameter('estimation.geometry_confirmed').value)
        if confirmed:self._drain()
        else:self._tracker.reset();self._pending.clear()
        stamp=self.get_clock().now().to_msg()
        sample=self._tracker.sample(stamp_seconds(stamp),self._horizon)
        msg=TargetState();msg.header.stamp=stamp;msg.header.frame_id=self._frame
        msg.cur_x,msg.cur_y=map(float,sample.position)
        msg.pred_x,msg.pred_y=map(float,sample.prediction)
        msg.vel_x,msg.vel_y=map(float,sample.velocity)
        msg.acc_x,msg.acc_y=map(float,sample.acceleration)
        msg.cov_xy=list(map(float,sample.covariance.reshape(4)))
        msg.valid=bool(sample.valid);msg.track_status=int(sample.status)
        msg.last_obs_age=float(sample.age) if math.isfinite(sample.age) else -1.
        self._pub.publish(msg)

    def shutdown_summary(self):
        self._log.info(f'图像时刻目标估计：接受{self._n_observed}，拒绝{self._n_rejected}；未代替实机验证')


def main(args=None):
    rclpy.init(args=args);node=TargetEstimatorNode()
    try:rclpy.spin(node)
    # Ctrl+C 是正常停机请求；finally 仍执行停止/关闭资源。
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):pass
    except Exception:
        # SIGINT关闭context时，tf2py可能把中断翻译成类型错误；运行期异常仍抛出。
        if rclpy.ok():raise
    finally:
        node.shutdown_summary();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0
