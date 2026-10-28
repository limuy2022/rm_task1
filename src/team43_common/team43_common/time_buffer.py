"""Bounded measurement-time buffers; no ROS clock or hardware access."""
from bisect import bisect_left, bisect_right
import math
import numpy as np
from .frames import quat_to_mat, quat_from_mat, compose_T


def stamp_seconds(stamp):
    sec, ns = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= ns < 1000000000:
        raise ValueError('invalid ROS stamp')
    return sec + ns * 1e-9


class TimeBuffer:
    def __init__(self, capacity=512):
        self.capacity = max(2, int(capacity))
        self.times, self.values = [], []

    def clear(self):
        self.times.clear(); self.values.clear()

    def put(self, stamp, value):
        t = float(stamp)
        if not math.isfinite(t) or t < 0:
            raise ValueError('measurement time must be finite and nonnegative')
        i = bisect_left(self.times, t)
        if i < len(self.times) and self.times[i] == t:
            self.values[i] = value
        else:
            self.times.insert(i, t); self.values.insert(i, value)
        if len(self.times) > self.capacity:
            n = len(self.times) - self.capacity
            del self.times[:n]; del self.values[:n]

    def before(self, stamp, max_age):
        t = float(stamp)
        if not math.isfinite(t):
            return None
        if not math.isfinite(float(max_age)) or max_age < 0: return None
        i = bisect_right(self.times, t) - 1
        if i < 0 or t - self.times[i] > float(max_age):
            return None
        return self.times[i], self.values[i]

    def bracket(self, stamp, max_gap):
        t = float(stamp)
        if not math.isfinite(t) or not math.isfinite(float(max_gap)) or max_gap <= 0: return None
        i = bisect_left(self.times, t)
        if i < len(self.times) and self.times[i] == t:
            return self.values[i], self.values[i], 0.
        if i == 0 or i == len(self.times):
            return None
        a, b = self.times[i-1], self.times[i]
        if b - a > float(max_gap):
            return None
        return self.values[i-1], self.values[i], (t-a)/(b-a)


def interpolate_transform(a, b, fraction):
    """Translation interpolation and shortest-arc SLERP, wxyz internally."""
    a, b = np.asarray(a, float).reshape(4,4), np.asarray(b, float).reshape(4,4)
    u = float(fraction)
    if not 0 <= u <= 1 or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError('invalid pose interpolation')
    qa, qb = quat_from_mat(a[:3,:3]), quat_from_mat(b[:3,:3])
    dot = float(qa @ qb)
    if dot < 0: qb, dot = -qb, -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        q = qa + u*(qb-qa); q /= np.linalg.norm(q)
    else:
        angle = math.acos(dot)
        q = (math.sin((1-u)*angle)*qa + math.sin(u*angle)*qb)/math.sin(angle)
    return compose_T(quat_to_mat(q), (1-u)*a[:3,3] + u*b[:3,3])


class PoseHistory(TimeBuffer):
    def sample(self, stamp, max_gap=.25):
        pair = self.bracket(stamp, max_gap)
        return None if pair is None else interpolate_transform(*pair)
