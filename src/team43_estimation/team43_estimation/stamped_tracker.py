"""Reuse CAKalman in observation time; publishing never advances the filter.

A delayed image is updated at its own stamp. Out-of-order images are rejected;
this is not an out-of-sequence smoother. A long outage starts a new track.
"""
from dataclasses import dataclass
import math
import numpy as np
from .ca_kalman import CAKalman


@dataclass
class TrackSample:
    stamp: float
    position: np.ndarray
    prediction: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    covariance: np.ndarray
    age: float
    status: int
    valid: bool


class StampedTracker:
    def __init__(self, kf=None, max_observation_delay=.3, lost_timeout=.5, fresh_timeout=.1):
        self.kf = kf or CAKalman()
        self.max_delay = float(max_observation_delay)
        self.lost_timeout = float(lost_timeout)
        self.fresh_timeout = float(fresh_timeout)
        if (not all(math.isfinite(v) for v in (self.max_delay, self.lost_timeout, self.fresh_timeout))
                or self.max_delay <= 0 or self.lost_timeout <= 0
                or not 0 <= self.fresh_timeout <= self.lost_timeout):
            raise ValueError('invalid observation freshness limits')
        self.last_observation = None
        self.marker_id = None
        self.reason = 'uninitialized'

    def reset(self):
        self.kf.reset(); self.last_observation = None; self.marker_id = None
        self.reason = 'reset'

    def observe(self, stamp, position, covariance, *, now, marker_id=0):
        t, current = float(stamp), float(now)
        z, R = np.asarray(position,float).reshape(2), np.asarray(covariance,float).reshape(2,2)
        if (not math.isfinite(t) or not math.isfinite(current) or t < 0
                or t-current > .01 or current-t > self.max_delay
                or not np.all(np.isfinite(z)) or not np.all(np.isfinite(R))):
            self.reason='invalid_or_stale'; return False
        R = (R+R.T)/2
        if np.linalg.eigvalsh(R).min() <= 0:
            self.reason='bad_covariance'; return False
        if self.marker_id is not None and marker_id != self.marker_id:
            self.reset()
        if self.kf.t_last is not None and t <= self.kf.t_last:
            self.reason='out_of_order'; return False
        if self.last_observation is not None and t-self.last_observation > self.lost_timeout:
            self.kf.reset()
        self.marker_id = marker_id
        self.kf.advance(t)
        accepted = self.kf.update(z,R)
        self.reason = self.kf.last_result
        if accepted: self.last_observation=t
        return accepted

    def sample(self, stamp, prediction_horizon=.1):
        t = float(stamp)
        if not math.isfinite(float(prediction_horizon)) or prediction_horizon < 0:
            raise ValueError('prediction horizon must be finite and nonnegative')
        dt = 0. if self.kf.t_last is None else t-self.kf.t_last
        age = math.inf if self.last_observation is None else t-self.last_observation
        valid = (self.kf.initialized and math.isfinite(t) and dt >= -1e-7
                 and -1e-7 <= age <= self.lost_timeout and not self.kf.diverged)
        if not valid:
            z=np.zeros(2)
            return TrackSample(t,z.copy(),z.copy(),z.copy(),z.copy(),np.eye(2),age,0,False)
        dt=max(dt,0.)
        state=self.kf.x
        pos=state[:2]+state[2:4]*dt+.5*state[4:6]*dt*dt
        vel=state[2:4]+state[4:6]*dt
        horizon=float(prediction_horizon)
        pred=pos+vel*horizon+.5*state[4:6]*horizon*horizon
        F=self.kf._build_F(dt)
        cov=(F@self.kf.P@F.T+self.kf._build_Q(dt))[:2,:2]
        return TrackSample(t,pos,pred,vel,state[4:6].copy(),cov,max(age,0.),
                           1 if age<=self.fresh_timeout else 2,True)
