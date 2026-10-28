"""不依赖ROS的指令门控。禁用/故障时静默，重新启用前留协议静默窗口。"""
import math

class CommandGate:
    def __init__(self, imu_timeout=.1, cmd_timeout=.1, quiet_s=.6,
                 omega_max=2., slew=15.):
        if not (0 < imu_timeout < .5 and 0 < cmd_timeout < .5 and quiet_s >= .5
                and 0 < omega_max <= 2. and 0 < slew <= 15.):
            raise ValueError('超时须在(0,.5)，静默>=.5，速度<=2，斜率<=15')
        self.imu_timeout=imu_timeout; self.cmd_timeout=cmd_timeout
        self.quiet_s=quiet_s; self.omega_max=omega_max; self.slew=slew
        self.enabled=False; self.fault=False
        self.imu_at=None; self.cmd_at=None; self.cmd=None
        self.ready_at=0.; self.last_output_at=None; self.previous=(0.,0.)

    def enable(self, enabled, now):
        if not enabled:
            self.enabled=False; self.fault=False; self.cmd_at=None; self.cmd=None
            self.last_output_at=None; self.previous=(0.,0.)
        elif not self.enabled:
            self.enabled=True; self.ready_at=now+self.quiet_s
            self.cmd_at=None; self.cmd=None; self.last_output_at=None
            self.previous=(0.,0.)

    def imu(self, now):
        if self.enabled and self.imu_at is not None and now-self.imu_at > self.imu_timeout:
            self.fault=True
        self.imu_at=now

    def reset(self):
        self.fault=True; self.cmd_at=None; self.cmd=None

    def command(self, yaw, pitch, valid, now):
        if not valid or not all(math.isfinite(v) for v in (yaw,pitch)):
            self.cmd=(0.,0.)
            # An invalid target/command is a stop, not a ramp from the previous
            # nonzero value. Valid motion resumes through the normal slew cap.
            self.previous=(0.,0.); self.last_output_at=None
        else:
            self.cmd=tuple(max(-self.omega_max,min(self.omega_max,v)) for v in (yaw,pitch))
        self.cmd_at=now

    def output(self, now):
        if not self.enabled or self.fault:
            return None
        if self.imu_at is None:
            return None
        if now-self.imu_at > self.imu_timeout:
            self.fault=True; return None
        if now < self.ready_at:
            return None
        if self.cmd_at is None:
            return None
        if now-self.cmd_at > self.cmd_timeout:
            self.fault=True; return None
        dt=0. if self.last_output_at is None else max(0.,min(now-self.last_output_at,.1))
        step=self.slew*dt
        self.previous=tuple(max(p-step,min(p+step,v)) for p,v in zip(self.previous,self.cmd))
        self.last_output_at=now
        return self.previous
