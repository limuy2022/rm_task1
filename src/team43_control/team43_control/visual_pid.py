"""A single angular-error PID: radians in, requested radians/second out.

This module has no ROS, geometry, serial or device dependencies.  The caller
supplies elapsed image time and applies command freshness and acceleration
limits.  ``integral_limit`` bounds the I term's *output contribution*, in rad/s.
"""

import math


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class ErrorConditioner:
    """First-order low-pass and continuous dead zone, in angular units.

    Filter only new camera samples using their source-time dt. The dead zone
    subtracts its width outside the zone, avoiding an output jump at its edge.
    """
    def __init__(self, tau=.04, deadband=(0., 0.)):
        self.tau = float(tau)
        self.deadband = tuple(float(value) for value in deadband)
        if (len(self.deadband) != 2 or not math.isfinite(self.tau) or self.tau < 0
                or any(not math.isfinite(value) or value < 0 for value in self.deadband)):
            raise ValueError('filter/deadband must be finite and nonnegative')
        self.reset()

    def reset(self):
        self.filtered = None

    def update(self, error, dt):
        error = tuple(float(value) for value in error)
        if len(error) != 2 or not all(math.isfinite(value) for value in error) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('invalid filter input')
        alpha = dt / (self.tau + dt)
        self.filtered = error if self.filtered is None else tuple(
            old + alpha*(value-old) for old, value in zip(self.filtered, error))
        control = tuple(math.copysign(max(0., abs(value)-width), value)
                        for value, width in zip(self.filtered, self.deadband))
        return self.filtered, control


class AxisPID:
    """PID with conditional integration and a filtered error derivative.

    The first valid sample after construction/reset has D = 0.  Invalid input
    clears all history and returns zero, so a later target cannot inherit the
    previous target's integral or derivative history.
    """

    def __init__(self, kp=2.0, ki=0.0, kd=0.0, output_limit=.25,
                 integral_limit=.1, derivative_tau=.05):
        settings = (kp, ki, kd, output_limit, integral_limit, derivative_tau)
        settings = tuple(float(value) for value in settings)
        if not all(math.isfinite(value) for value in settings):
            raise ValueError('PID parameters must be finite numbers')
        if any(value < 0 for value in settings) or settings[3] <= 0:
            raise ValueError('PID gains/limits must be nonnegative; output_limit > 0')
        (self.kp, self.ki, self.kd, self.output_limit,
         self.integral_limit, self.derivative_tau) = settings
        self.reset()

    def reset(self):
        self._integral_output = 0.0
        self._previous_error = None
        self._derivative = 0.0

    def update(self, error, dt):
        try:
            error, dt = float(error), float(dt)
        except (TypeError, ValueError, OverflowError):
            # 外部观测/时差非法时复位积分和导数，不沿用旧控制输出。
            self.reset()
            return 0.0
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0:
            self.reset()
            return 0.0

        derivative = 0.0
        if self._previous_error is not None:
            raw_derivative = (error - self._previous_error) / dt
            alpha = dt / (self.derivative_tau + dt)
            derivative = self._derivative + alpha * (raw_derivative - self._derivative)
        p_output, d_output = self.kp * error, self.kd * derivative
        integral_delta = self.ki * error * dt
        if not all(math.isfinite(value) for value in
                   (derivative, p_output, d_output, integral_delta)):
            self.reset()
            return 0.0

        candidate = _clamp(self._integral_output + integral_delta,
                           self.integral_limit)
        proposed = p_output + candidate + d_output
        if not math.isfinite(proposed):
            self.reset()
            return 0.0
        # Do not accumulate more I in the direction of an already saturated
        # output.  Integration in the opposite direction is still allowed.
        pushing_high = proposed > self.output_limit and integral_delta > 0
        pushing_low = proposed < -self.output_limit and integral_delta < 0
        if not (pushing_high or pushing_low):
            self._integral_output = candidate

        self._previous_error = error
        self._derivative = derivative
        return _clamp(p_output + self._integral_output + d_output,
                      self.output_limit)
