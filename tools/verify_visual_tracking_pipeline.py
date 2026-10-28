"""Synthetic images -> production ArUco detector -> production image PID -> ideal velocity plant.

Tests software direction/convergence only; does not establish real motor polarity,
firmware control, ROS transport latency, or gains on the actual gimbal.
"""
from pathlib import Path
import json
import math
import sys
from types import SimpleNamespace
import cv2
import numpy as np
import yaml
from verify_visual_servo import create_harness, corners_message, imu_message

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/team43_perception'))
# Reuse transport/import stand-ins; importing this module does not run its
# unittest suite. The production detector, OpenCV and PID methods run below.
import verify_perception_offline  # noqa: F401 - installs the shared stand-ins
from team43_perception.aruco_detector_node import ArucoDetectorNode, _make_detector_params


def main():
    camera = yaml.safe_load((ROOT/'src/team43_bringup/config/camera_team43.yaml').read_text())['camera']
    K = np.array([[camera['fx'], 0., camera['cx']], [0., camera['fy'], camera['cy']], [0., 0., 1.]])
    D = np.asarray(camera['distortion'])
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = object.__new__(ArucoDetectorNode)
    detector._dict = dictionary; detector._detector = _make_detector_params()
    detector._target_digit = 0
    detector._log = SimpleNamespace(error=lambda s: (_ for _ in ()).throw(AssertionError(s)))
    src = np.float32([[0,0],[255,0],[255,255],[0,255]])

    def render(center_optical, marker_id):
        x, y, z = center_optical
        square = np.array([[x-.05,y-.05,z],[x+.05,y-.05,z],
                           [x+.05,y+.05,z],[x-.05,y+.05,z]])
        dst = cv2.projectPoints(square, np.zeros(3), np.zeros(3), K, D)[0].reshape(4,2).astype(np.float32)
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 256)
        image = cv2.warpPerspective(marker, cv2.getPerspectiveTransform(src,dst),
            (camera['width'],camera['height']), flags=cv2.INTER_NEAREST, borderValue=255)
        corners, ids = detector._detect(image)
        detector._target_digit = marker_id
        picked = detector._pick(corners, ids)
        assert picked is not None, f'real detector did not find ID {marker_id}'
        return picked

    for marker_id in range(4): render([.12,.06,2.], marker_id)
    node, clock, params, sent = create_harness(camera)
    params.update(motion_enabled=True, run_seconds=10.)
    node.tick()
    target = np.array([3., .25, .12])
    yaw, pitch, first_error = 0., 0., None
    for _ in range(150):
        dt = 1./30; clock.advance(dt)
        cy,sy,cp,sp = math.cos(yaw),math.sin(yaw),math.cos(pitch),math.sin(pitch)
        R = np.array([[cy*cp,-sy,cy*sp],[sy*cp,cy,sy*sp],[-sp,0.,cp]])
        body = R.T@target
        optical = np.array([-body[1],-body[2],body[0]])
        picked = render(optical, 0)
        q = (math.cos(yaw/2)*math.cos(pitch/2),-math.sin(yaw/2)*math.sin(pitch/2),
             math.cos(yaw/2)*math.sin(pitch/2),math.sin(yaw/2)*math.cos(pitch/2))
        node.on_imu(imu_message(clock,q))
        msg = corners_message(clock, camera['cx'], camera['cy'])
        msg.corners = [SimpleNamespace(x=float(x),y=float(y)) for x,y in picked]
        node.on_corners(msg); node.tick()
        error = np.asarray(node.observation[2])
        if first_error is None:
            first_error = error.copy()
            assert sent[-1].angular.z > 0 and sent[-1].angular.y < 0
        assert not node.fault, node.fault
        yaw += sent[-1].angular.z*dt
        pitch += sent[-1].angular.y*dt
    assert np.linalg.norm(error)<5., error
    assert np.linalg.norm(error)<np.linalg.norm(first_error)*.05
    dynamics = []
    for scenario in ('delayed_step', 'delayed_moving'):
        node, clock, params, sent = create_harness(camera)
        params.update(motion_enabled=True, run_seconds=10.)
        node.tick()
        yaw = pitch = 0.
        velocity = np.zeros(2)
        pending = [np.zeros(2), np.zeros(2)]
        pointing_errors = []
        for k in range(180):
            dt = 1. / 30; t = k * dt; clock.advance(dt)
            if scenario == 'delayed_step':
                target = np.array([3., .25, .12]) if t < 2. else np.array([3., -.20, -.10])
            else:
                target = np.array([3., .20 + .10 * math.sin(.8 * t), .08 + .05 * math.sin(.5 * t)])
            cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
            R = np.array([[cy*cp,-sy,cy*sp],[sy*cp,cy,sy*sp],[-sp,0.,cp]])
            body = R.T @ target
            optical = np.array([-body[1], -body[2], body[0]])
            picked = render(optical, 0)
            q = (math.cos(yaw/2)*math.cos(pitch/2), -math.sin(yaw/2)*math.sin(pitch/2),
                 math.cos(yaw/2)*math.sin(pitch/2), math.sin(yaw/2)*math.cos(pitch/2))
            node.on_imu(imu_message(clock, q))
            msg = corners_message(clock, camera['cx'], camera['cy'])
            msg.corners = [SimpleNamespace(x=float(x), y=float(y)) for x, y in picked]
            node.on_corners(msg); node.tick()
            assert not node.fault, (scenario, node.fault)
            pending.append(np.array([sent[-1].angular.z, sent[-1].angular.y]))
            request = pending.pop(0)
            # 67ms命令延迟、80ms一阶响应、速度死区及限幅；不调整生产增益。
            request = np.where(np.abs(request) < .001, 0., np.clip(request, -.25, .25))
            velocity += dt / (.08 + dt) * (request - velocity)
            yaw += velocity[0] * dt; pitch += velocity[1] * dt
            pointing_errors.append(math.degrees(math.atan2(np.linalg.norm(optical[:2]), optical[2])))
        tail = pointing_errors[-30:]
        maximum = max(tail)
        assert maximum < (.4 if scenario == 'delayed_step' else .9), (scenario, maximum)
        dynamics.append(dict(scenario=scenario, last_second_max_error_deg=maximum,
                             command_delay_s=2./30, actuator_tau_s=.08, deadzone_rad_s=.001))
    print(json.dumps({'passed':True,'real_detector_ids':[0,1,2,3],
        'initial_error_px':first_error.tolist(),'final_error_px':error.tolist(),
        'simulated_seconds':5., 'dynamic_scenarios': dynamics,
        'model':'理想基线及带延迟/惯性/死区的合成执行器；不代表实机'},ensure_ascii=False))


if __name__=='__main__': main()
