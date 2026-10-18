"""Behavior tests of the production image controller under jitter and target loss.

Uses the existing ROS stand-ins; real filter/PID/command/stop code executes.
No camera driver, serial port or real motor is opened.
"""
import json
import math
import statistics
from types import SimpleNamespace
from verify_visual_servo import CAMERA, create_harness, corners_message, imu_message


def start(**settings):
    node, clock, params, sent = create_harness(CAMERA)
    params.update(settings); params['motion_enabled'] = True
    node.tick()
    sample(node, clock, sent, (720., 520.))
    return node, clock, params, sent


def sample(node, clock, sent, center=None, *, dt=.02, missing=False, marker_id=0):
    clock.advance(dt)
    node.on_imu(imu_message(clock))
    if center is not None or missing:
        message = corners_message(clock, *(center or (720., 520.)), marker_id=marker_id)
        if missing: message.valid = False; message.corners = []
        node.on_corners(message)
    node.tick()
    return sent[-1] if sent else None


def main():
    checks = []
    def jitter(tau):
        node, clock, params, sent = start(filter_tau=tau, deadband_px=0., slew=15.)
        outputs = []
        for k in range(160):
            value = sample(node, clock, sent, (720.+(8. if k%2 else -8.),520.))
            if k>30: outputs.append(value.angular.z)
        return statistics.pstdev(outputs)
    raw, filtered = jitter(0.), jitter(.04)
    assert filtered < raw*.4, (raw, filtered)
    checks.append('lowpass_reduces_command_jitter_without_real_hardware')

    node, clock, params, sent = start(filter_tau=0., deadband_px=2.,ki_yaw=.5,kd_yaw=.1)
    for k in range(70):
        value = sample(node, clock, sent, (640.+(-1. if k%2 else 1.),480.+(.8 if k%3 else -.8)))
    assert value.angular.z == value.angular.y == 0.
    assert all(pid._integral_output==0. for pid in node.pid)
    checks.append('sub_deadband_noise_settles_to_zero_with_I_and_D_enabled')

    node, clock, params, sent = start()
    velocity = sent[-1].angular.z
    for _ in range(2):
        value = sample(node,clock,sent,missing=True)
        assert 0 < abs(value.angular.z) < abs(velocity)
        assert math.copysign(1.,value.angular.z)==math.copysign(1.,velocity)
        velocity=value.angular.z
    # A third failed image stops even though the 80ms time window has not elapsed.
    before=len(sent);value=sample(node,clock,sent,missing=True)
    assert value.angular.z==value.angular.y==0.
    for _ in range(7): sample(node,clock,sent)
    assert len(sent)==before+3
    assert all(m.angular.z==m.angular.y==0. for m in sent[before:])
    checks.append('lost_frame_budget_stops_and_only_three_zero_frames_follow')

    node, clock, params, sent = start()
    value=sample(node,clock,sent,missing=True)
    assert abs(value.angular.z)>0
    # No more images: the finite grace still expires; no indefinite velocity hold.
    for _ in range(6): value=sample(node,clock,sent)
    assert value.angular.z==value.angular.y==0.
    before=len(sent)
    for _ in range(5): sample(node,clock,sent)
    assert len(sent)==before
    checks.append('loss_grace_expires_without_more_image_callbacks')

    node, clock, params, sent = start()
    sample(node,clock,sent,missing=True)
    params['motion_enabled']=False
    assert sample(node,clock,sent).angular.z==0
    checks.append('disarm_overrides_loss_decay')

    node, clock, params, sent = start(ki_yaw=.5,kd_yaw=.1)
    for _ in range(5): sample(node,clock,sent,(720.,520.))
    assert node.pid[0]._integral_output<0
    sample(node,clock,sent,missing=True)
    assert node.pid[0]._integral_output==0
    sample(node,clock,sent,(560.,440.),marker_id=1)
    assert node.marker_id==1 and node.lost_count==0 and node.desired[0]>0
    assert node.pid[0]._derivative==0.  # reacquisition cannot inherit a D kick
    checks.append('loss_and_ID_change_clear_history_and_accept_new_direction')

    node, clock, params, sent = start(loss_grace_s=0.)
    assert sample(node,clock,sent,missing=True).angular.z==0.
    checks.append('zero_grace_restores_immediate_stop')

    node, clock, params, sent = start()
    node.timer=SimpleNamespace(timer_period_ns=20_000_000)
    params['control_hz']=25.
    sample(node,clock,sent,(720.,520.))
    assert node.timer.timer_period_ns==40_000_000
    params['max_omega']=.03; params['deadband_px']=0.; params['filter_tau']=0.
    params['kp_yaw']=3.;params['kd_yaw']=.05
    for _ in range(5): sample(node,clock,sent,(1000.,520.))
    assert all(abs(v)<=.03 for v in node.output)
    params['control_hz']=0.
    sample(node,clock,sent,(1000.,520.))
    assert sent[-1].angular.z==0 and node.fault
    checks.append('dynamic_rate_gains_and_limits_apply_invalid_rate_stops')
    print(json.dumps({'passed':True,'checks':checks,'jitter_output_std_rad_s':{'unfiltered':raw,'filtered':filtered},
        'real_devices_opened':0,'limitations':'synthetic camera/clock; real response and ROS timing unverified'},indent=2))


if __name__=='__main__': main()
