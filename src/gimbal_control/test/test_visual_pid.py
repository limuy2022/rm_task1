"""Exercise the production visual PID without ROS or physical devices.

由 pytest 自动收集；验证理想角速度执行器下的控制律。
The simulated plant executes requested angular speed exactly.  These checks
test the algorithm, not a motor, serial connection, camera or real tuning.
"""

import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/gimbal_control'))
from gimbal_control.visual_pid import AxisPID


def simulate(pid, seconds=12.0, target_speed=0.0, disturbance=0.0,
             initial_angle=-.2):
    dt, angle = .02, initial_angle
    errors, outputs = [], []
    for step in range(round(seconds / dt)):
        target = target_speed * step * dt
        error = target - angle
        output = pid.update(error, dt)
        errors.append(error)
        outputs.append(output)
        angle += (output - disturbance) * dt
    return errors, outputs


def test_visual_pid_regressions():
    checks, results = [], {}
    errors, outputs = simulate(AxisPID())
    assert all(0 <= error <= .2 + 1e-12 for error in errors)
    assert all(next_error <= error + 1e-12
               for error, next_error in zip(errors, errors[1:]))
    assert abs(errors[-1]) < 1e-8, errors[-1]
    assert max(map(abs, outputs)) <= .25
    results['default_p_static_final_error_rad'] = errors[-1]
    checks.append('default_P_converges_without_overshoot_in_speed_plant')

    for label, target_speed, disturbance in (
            ('constant_disturbance', 0.0, .04),
            ('moving_target', .04, 0.0)):
        baseline, _ = simulate(AxisPID(), seconds=30., target_speed=target_speed,
                               disturbance=disturbance, initial_angle=0.)
        integral, _ = simulate(AxisPID(ki=.5), seconds=30.,
                               target_speed=target_speed,
                               disturbance=disturbance, initial_angle=0.)
        assert .019 < abs(baseline[-1]) < .021, baseline[-1]
        assert abs(integral[-1]) < 1e-4
        assert abs(integral[-1]) < abs(baseline[-1]) / 100
        results[label] = {'p_error_rad': baseline[-1],
                          'pi_error_rad': integral[-1]}
        checks.append(label + '_I_reduces_steady_error')

    for sign in (-1., 1.):
        pid = AxisPID(ki=1.)
        for _ in range(2000):
            assert pid.update(sign, .02) == sign * .25
        assert pid.update(0., .02) == 0., 'saturated I must not accumulate'
        assert pid.update(-sign * .01, .02) * sign < 0
    checks.append('positive_and_negative_saturation_do_not_wind_up')

    pid = AxisPID(kp=0., ki=2., output_limit=1., integral_limit=.05)
    for _ in range(1000):
        assert abs(pid.update(.1, .02)) <= .05
    assert math.isclose(pid.update(0., .02), .05)
    pid.reset()
    assert pid.update(0., .02) == 0.
    checks.append('I_contribution_limit_and_reset')

    pid = AxisPID(kp=0., kd=1., output_limit=100., derivative_tau=.1)
    assert pid.update(1., .1) == 0., 'first sample D must be zero'
    assert math.isclose(pid.update(1.1, .1), .5, abs_tol=1e-12)
    assert math.isclose(pid.update(1.1, .1), .25, abs_tol=1e-12)
    pid.reset()
    assert pid.update(-10., .1) == 0.
    checks.append('D_low_pass_and_no_first_sample_kick')

    for error, dt in ((math.nan, .1), (math.inf, .1), (0., math.nan),
                      (0., math.inf), (0., 0.), (0., -.1),
                      (None, .1), (0., None), (1e308, 1e308)):
        pid = AxisPID(kp=0., ki=1., kd=.1)
        pid.update(.01, .1)
        assert pid.update(error, dt) == 0.
        assert pid.update(.01, .1) == .001, 'invalid input must clear history'
    checks.append('invalid_inputs_stop_and_clear_history')

    for params in ({'kp': -1}, {'ki': math.nan}, {'kd': math.inf},
                   {'output_limit': 0}, {'integral_limit': -1},
                   {'derivative_tau': -1}):
        try:
            AxisPID(**params)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid parameters accepted: ' + str(params))
    checks.append('invalid_parameters_rejected')

    print(json.dumps({'passed': True, 'checks': checks, 'results': results,
                      'physical_devices_opened': 0, 'ros_nodes_started': 0,
                      'limitations': ['ideal angular-speed simulation; real motor response and gains unverified']},
                     ensure_ascii=False, indent=2))
