"""ROS-free rigid odometry conversion.

``T_A_B`` maps coordinates in B into A. An odometry pose is T_parent_sensor;
measured mounting parameters are T_base_sensor. Therefore
T_output_base = T_output_parent @ T_parent_sensor @ inv(T_base_sensor).
All rotations and translations are applied in 3D before the planar projection.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


def quaternion_matrix(xyzw: Sequence[float]) -> np.ndarray:
    q = np.asarray(xyzw, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.all(np.isfinite(q)) or norm < 1e-12:
        raise ValueError('orientation quaternion must be finite and nonzero')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ])


def matrix_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return unit xyzw, using a stable largest-diagonal branch near pi."""
    R = np.asarray(rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(R)) or not np.allclose(R.T @ R, np.eye(3), atol=1e-8) or np.linalg.det(R) < 0:
        raise ValueError('rotation must be a proper orthonormal matrix')
    trace = float(np.trace(R))
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = [(R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s,
             (R[1, 0]-R[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * math.sqrt(max(0.0, 1 + R[i, i]-R[j, j]-R[k, k]))
        q = [0.0]*4
        q[i] = s/4
        q[j] = (R[i, j]+R[j, i])/s
        q[k] = (R[i, k]+R[k, i])/s
        q[3] = (R[k, j]-R[j, k])/s
    result = np.asarray(q, dtype=float)
    return result / np.linalg.norm(result)


def rigid_transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)) -> np.ndarray:
    """Fixed-axis XYZ radians: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    t = np.asarray(xyz, dtype=float).reshape(3)
    r, p, y = np.asarray(rpy, dtype=float).reshape(3)
    if not np.all(np.isfinite(t)) or not all(map(math.isfinite, (r, p, y))):
        raise ValueError('extrinsic xyz/rpy must be finite')
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                 [-sp, cp*sr, cp*cr]]
    T[:3, 3] = t
    return T


def inverse_rigid(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


@dataclass(frozen=True)
class ConvertedOdometry:
    T_output_base: np.ndarray
    linear_base: np.ndarray
    angular_base: np.ndarray
    stamp_sec: int
    stamp_nanosec: int
    velocity_valid: bool = True

    @property
    def stamp_seconds(self) -> float:
        return self.stamp_sec + self.stamp_nanosec * 1e-9

    @property
    def yaw(self) -> float:
        R = self.T_output_base[:3, :3]
        if math.hypot(R[0, 0], R[1, 0]) < 1e-8:
            raise ValueError('base forward axis is vertical; planar yaw undefined')
        return math.atan2(R[1, 0], R[0, 0])

    @property
    def linear_output(self) -> np.ndarray:
        return self.T_output_base[:3, :3] @ self.linear_base


def convert_odometry(msg, T_base_sensor: np.ndarray, T_output_parent: np.ndarray,
                     twist_source='message', previous: ConvertedOdometry | None = None) -> ConvertedOdometry:
    """Convert pose and standard child-frame twist, preserving the exact stamp.

    ``pose_difference`` is needed for the supplied Point-LIO version, whose
    Odometry publisher leaves twist zero. Its first sample has unknown velocity
    (covariance must reflect that), not a measured stationary velocity.
    """
    try:
        stamp = msg.header.stamp
        sec, nsec = int(stamp.sec), int(stamp.nanosec)
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            raise ValueError('invalid source stamp')
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T_parent_sensor = np.eye(4)
        T_parent_sensor[:3, 3] = [float(position.x), float(position.y), float(position.z)]
        T_parent_sensor[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])
    except AttributeError as exc:
        raise TypeError('odometry must contain a source header stamp and a 3D pose') from exc
    if not np.all(np.isfinite(T_parent_sensor)):
        raise ValueError('nonfinite source pose')
    T = T_output_parent @ T_parent_sensor @ inverse_rigid(T_base_sensor)
    if twist_source == 'message':
        try:
            twist = msg.twist.twist
            linear_sensor = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=float)
            angular_sensor = np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=float)
        except AttributeError as exc:
            raise TypeError('message twist source requires linear and angular vectors') from exc
        if not np.all(np.isfinite(linear_sensor)) or not np.all(np.isfinite(angular_sensor)):
            raise ValueError('nonfinite source twist')
        Rbs = T_base_sensor[:3, :3]
        angular_base = Rbs @ angular_sensor
        # v_sensor = v_base + omega x (base->sensor); coordinates in base.
        linear_base = Rbs @ linear_sensor - np.cross(angular_base, T_base_sensor[:3, 3])
        valid = True
    elif twist_source == 'pose_difference':
        linear_base, angular_base = np.zeros(3), np.zeros(3)
        valid = False
        if previous is not None:
            dt = sec + nsec*1e-9 - previous.stamp_seconds
            if 0 < dt <= 1.0:
                linear_base = T[:3, :3].T @ ((T[:3, 3]-previous.T_output_base[:3, 3]) / dt)
                delta = previous.T_output_base[:3, :3].T @ T[:3, :3]
                qdelta = matrix_quaternion(delta)
                if qdelta[3] < 0:
                    qdelta = -qdelta
                length = np.linalg.norm(qdelta[:3])
                if length > 1e-12:
                    # delta axis is in previous base; express it in current base.
                    omega_previous = qdelta[:3]/length * (2*math.atan2(length, qdelta[3])/dt)
                    angular_base = delta.T @ omega_previous
                valid = True
    else:
        raise ValueError('odom_twist_source must be message or pose_difference')
    result = ConvertedOdometry(T, linear_base, angular_base, sec, nsec, valid)
    _ = result.yaw
    return result
