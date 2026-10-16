"""Calibrated two-axis gimbal geometry; numpy only, no ROS or hardware access.

All transforms are parent<-child. The rigid chain is
    T_base_mount Rz(yaw+zero_yaw) Trans(yaw_to_pitch_xyz)
    Ry(pitch+zero_pitch) T_pitch_gimbal T_gimbal_camera R_camera_optical.
Joint pitch is the protocol's right-handed +y angle (head down is positive).
IMU orientation is R_world_gimbal, NOT an encoder or a mechanical yaw angle.

A calibration is deliberately unusable without externally measured joint zero,
world alignment, mounting geometry and an explicitly confirmed base mounting.
The IMU can resolve only a calibrated single-turn joint branch; it cannot detect
unobserved full revolutions. A table-top camera does not imply a base_link pose.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np

from gimbal_common import frames as FR
from gimbal_common.time_buffer import PoseHistory


def finite_vector(value, length, name):
    try:
        out = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name}: expected {length} finite numbers') from exc
    if out.shape != (length,) or not np.all(np.isfinite(out)):
        raise ValueError(f'{name}: expected {length} finite numbers')
    return out.copy()


def rotation_matrix(value, name='rotation'):
    out = np.asarray(value, dtype=float)
    if (out.shape != (3, 3) or not np.all(np.isfinite(out))
            or not np.allclose(out.T @ out, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(out)), 1., abs_tol=1e-6)):
        raise ValueError(f'{name}: expected a proper 3x3 rotation')
    return out.copy()


def transform_matrix(value, name='transform'):
    out = np.asarray(value, dtype=float)
    if (out.shape != (4, 4) or not np.all(np.isfinite(out))
            or not np.allclose(out[3], [0., 0., 0., 1.], atol=1e-9, rtol=0)):
        raise ValueError(f'{name}: expected a rigid 4x4 transform')
    rotation_matrix(out[:3, :3], name)
    return out.copy()


def quaternion_rotation(wxyz):
    q = finite_vector(wxyz, 4, 'quaternion wxyz')
    norm = float(np.linalg.norm(q))
    if norm < .5 or norm > 1.5:
        raise ValueError('quaternion has invalid norm')
    return FR.quat_to_mat(q / norm)


def _ry(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def _rotation_error(a, b):
    return math.acos(float(np.clip((np.trace(a.T @ b) - 1.) / 2., -1., 1.)))


@dataclass(frozen=True)
class JointSolution:
    yaw: float
    pitch: float
    residual_rad: float


@dataclass(frozen=True)
class GimbalTransforms:
    base_gimbal: np.ndarray
    gimbal_camera: np.ndarray
    camera_optical: np.ndarray
    base_camera: np.ndarray
    base_optical: np.ndarray


class GimbalKinematics:
    """A measured calibration and exact forward/inverse rigid chain.

    Required mapping keys (no guessed geometry or zero defaults):
      base_mount_xyz/rpy, yaw_to_pitch_xyz, pitch_gimbal_xyz/rpy,
      gimbal_camera_xyz/rpy, odom_world_rpy, joint_zero_rad,
      yaw_limits_rad, pitch_limits_rad, and three True confirmation flags.

    odom_world_rpy defines R_odom<-IMU-world. It is the inverse rotation of
    localization.world_odom_xyz_rpy (T_world<-odom), if that calibration is used.
    joint_zero_rad is the measured axis angle at reported mechanical joint zero;
    forward(axis angle) = reported joint angle + joint_zero_rad.
    yaw_limits_rad must span less than one revolution: orientation cannot resolve
    a multi-turn encoder value. Effective pitch must remain inside (-pi/2,pi/2).
    """
    def __init__(self, calibration: Mapping):
        c = calibration
        for key in ('calibration_confirmed', 'joint_zero_known', 'mounted_on_base_confirmed'):
            if c.get(key) is not True:
                raise ValueError(f'{key} must be explicitly true after measurement')
        required = {
            'base_mount_xyz': 3, 'base_mount_rpy': 3,
            'yaw_to_pitch_xyz': 3, 'pitch_gimbal_xyz': 3, 'pitch_gimbal_rpy': 3,
            'gimbal_camera_xyz': 3, 'gimbal_camera_rpy': 3,
            'odom_world_rpy': 3, 'joint_zero_rad': 2,
            'yaw_limits_rad': 2, 'pitch_limits_rad': 2,
        }
        self.values = {}
        for key, length in required.items():
            if key not in c:
                raise ValueError(f'missing measured calibration: {key}')
            self.values[key] = finite_vector(c[key], length, key)
        self.zero = self.values['joint_zero_rad']
        self.yaw_limits = self.values['yaw_limits_rad']
        self.pitch_limits = self.values['pitch_limits_rad']
        if not 0 < self.yaw_limits[1] - self.yaw_limits[0] < 2. * math.pi:
            raise ValueError('yaw_limits_rad must select a single branch narrower than 2pi')
        if not self.pitch_limits[0] < self.pitch_limits[1]:
            raise ValueError('pitch_limits_rad must be increasing')
        effective_pitch = self.pitch_limits + self.zero[1]
        if not (-math.pi / 2. < effective_pitch[0] < effective_pitch[1] < math.pi / 2.):
            raise ValueError('effective pitch limits must stay inside (-pi/2,pi/2)')
        self.max_residual = float(c.get('max_residual_rad', .05))
        self.min_pitch_cos = float(c.get('min_pitch_cos', .05))
        if not math.isfinite(self.max_residual) or not 0 < self.max_residual < .5:
            raise ValueError('max_residual_rad must be finite, between 0 and .5')
        if not math.isfinite(self.min_pitch_cos) or not 0 < self.min_pitch_cos < 1.:
            raise ValueError('min_pitch_cos must be between 0 and 1')
        self.T_base_mount = FR.compose_T(FR.rpy_to_mat(self.values['base_mount_rpy']), self.values['base_mount_xyz'])
        self.T_pitch_gimbal = FR.compose_T(FR.rpy_to_mat(self.values['pitch_gimbal_rpy']), self.values['pitch_gimbal_xyz'])
        self.T_gimbal_camera = FR.compose_T(FR.rpy_to_mat(self.values['gimbal_camera_rpy']), self.values['gimbal_camera_xyz'])
        # Historical helper name is misleading: this is camera_link<-optical.
        self.T_camera_optical = FR.compose_T(FR.R_opt_from_link(), [0., 0., 0.])
        self.R_odom_world = FR.rpy_to_mat(self.values['odom_world_rpy'])

    def _angles(self, yaw, pitch):
        q = finite_vector([yaw, pitch], 2, 'joint angles')
        if not (self.yaw_limits[0] - 1e-9 <= q[0] <= self.yaw_limits[1] + 1e-9
                and self.pitch_limits[0] - 1e-9 <= q[1] <= self.pitch_limits[1] + 1e-9):
            raise ValueError('joint angles outside measured limits')
        effective = q + self.zero
        if abs(math.cos(float(effective[1]))) < self.min_pitch_cos:
            raise ValueError('pitch branch too close to vertical')
        return effective

    def forward(self, yaw, pitch):
        y, p = self._angles(yaw, pitch)
        T_bg = (self.T_base_mount
                @ FR.compose_T(FR.rotz(y), [0., 0., 0.])
                @ FR.compose_T(np.eye(3), self.values['yaw_to_pitch_xyz'])
                @ FR.compose_T(_ry(p), [0., 0., 0.])
                @ self.T_pitch_gimbal)
        T_bc = T_bg @ self.T_gimbal_camera
        return GimbalTransforms(T_bg, self.T_gimbal_camera.copy(), self.T_camera_optical.copy(),
                                T_bc, T_bc @ self.T_camera_optical)

    def inverse(self, R_odom_base, R_world_gimbal):
        """Recover joint angles from SAME-STAMP full 3-D attitudes.

        Full base roll/pitch/yaw and both fixed mounting rotations are removed
        before Rz*Ry decomposition. A residual rotation (including unmodelled
        roll, incorrect IMU convention or calibration) rejects the result.
        No principal/world yaw is used as a mechanical angle.
        """
        R_ob = rotation_matrix(R_odom_base, 'R_odom_base')
        R_wg = rotation_matrix(R_world_gimbal, 'R_world_gimbal')
        R_bg = R_ob.T @ self.R_odom_world @ R_wg
        R_axes = self.T_base_mount[:3, :3].T @ R_bg @ self.T_pitch_gimbal[:3, :3].T
        cp = math.hypot(float(R_axes[0, 0]), float(R_axes[1, 0]))
        if cp < self.min_pitch_cos:
            raise ValueError('joint inversion too close to vertical')
        y = math.atan2(float(R_axes[1, 0]), float(R_axes[0, 0]))
        p = math.atan2(float(-R_axes[2, 0]), cp)
        residual = _rotation_error(FR.rotz(y) @ _ry(p), R_axes)
        if residual > self.max_residual:
            raise ValueError(f'two-axis orientation residual {residual:.6f} rad exceeds calibration tolerance')
        raw_y = y - self.zero[0]
        # There must be exactly one equivalent yaw within the measured branch.
        lower, upper = self.yaw_limits
        k_min = math.ceil((lower - raw_y - 1e-9) / (2. * math.pi))
        k_max = math.floor((upper - raw_y + 1e-9) / (2. * math.pi))
        if k_min != k_max:
            raise ValueError('IMU yaw is outside or ambiguous in measured joint branch')
        yaw, pitch = raw_y + k_min * 2. * math.pi, p - self.zero[1]
        self._angles(yaw, pitch)
        return JointSolution(float(yaw), float(pitch), float(residual))


@dataclass(frozen=True)
class KinematicsSample:
    stamp: float
    joints: JointSolution
    transforms: GimbalTransforms


class SynchronizedKinematics:
    """Measurement-time history, freshness and MCU-reset gate (no ROS).

    IMU samples wait for an exact base pose or a bounded SLERP bracket. They are
    never paired with the latest pose or extrapolated. A reset/epoch mismatch is
    latched until a new instance is created with a freshly confirmed calibration.
    """
    def __init__(self, model=None, *, calibrated_epoch=-1, max_age=.2, max_gap=.1,
                 future_tolerance=.01):
        self.model = model
        self.calibrated_epoch = int(calibrated_epoch)
        self.max_age, self.max_gap, self.future_tolerance = map(float, (max_age, max_gap, future_tolerance))
        if (not all(math.isfinite(v) for v in (self.max_age, self.max_gap, self.future_tolerance))
                or self.max_age <= 0 or self.max_gap <= 0 or self.future_tolerance < 0):
            raise ValueError('invalid synchronization tolerances')
        self.base = PoseHistory(capacity=512)
        self.pending = {}
        self.epoch = None
        self.fault = False
        self.last_stamp = -1.
        self.last_error = ''

    @property
    def enabled(self):
        return (self.model is not None and self.calibrated_epoch >= 0
                and self.epoch == self.calibrated_epoch and not self.fault)

    def observe_epoch(self, epoch):
        value = int(epoch)
        if value < 0:
            raise ValueError('epoch must be nonnegative')
        if ((self.epoch is not None and value != self.epoch)
                or value != self.calibrated_epoch):
            self.fault = True
            self.base.clear()
            self.pending.clear()
            self.last_error = 'MCU epoch changed or differs from externally calibrated epoch; recalibration/restart required'
        self.epoch = value

    def _fresh(self, stamp, now):
        t, n = float(stamp), float(now)
        return (math.isfinite(t) and math.isfinite(n) and t >= 0
                and -self.future_tolerance <= n - t <= self.max_age)

    def add_base(self, stamp, T_odom_base, now):
        if not self.enabled or not self._fresh(stamp, now):
            return False
        try:
            self.base.put(stamp, transform_matrix(T_odom_base, 'T_odom_base'))
        except (TypeError, ValueError) as exc:
            self.last_error = str(exc)
            return False
        return True

    def add_imu(self, stamp, R_world_gimbal, now):
        if not self.enabled or not self._fresh(stamp, now) or stamp <= self.last_stamp:
            return False
        try:
            self.pending[float(stamp)] = rotation_matrix(R_world_gimbal, 'R_world_gimbal')
        except (TypeError, ValueError) as exc:
            self.last_error = str(exc)
            return False
        if len(self.pending) > 64:
            del self.pending[min(self.pending)]
        return True

    def drain(self, now):
        if not self.enabled:
            return []
        out = []
        for stamp in sorted(self.pending):
            if stamp <= self.last_stamp or not self._fresh(stamp, now):
                del self.pending[stamp]
                continue
            T_ob = self.base.sample(stamp, max_gap=self.max_gap)
            if T_ob is None:
                continue
            R_wg = self.pending.pop(stamp)
            try:
                joints = self.model.inverse(T_ob[:3, :3], R_wg)
                out.append(KinematicsSample(stamp, joints, self.model.forward(joints.yaw, joints.pitch)))
                self.last_stamp = stamp
                self.last_error = ''
            except (TypeError, ValueError) as exc:
                self.last_error = str(exc)
        return out
