"""验证G判轴所需的数学、黄金帧及只读显示计算；在 ROS 2 Humble 容器内运行，
本脚本不启动节点、不做DDS通信、不访问设备。"""
from pathlib import Path
import json
import math
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/gimbal_common'))
from gimbal_common.protocol import decode_imu
from watch_gimbal_axes import orientation_values


def golden_cases():
    case = {}
    for line in (ROOT/'protocol/golden_frames.txt').read_text().splitlines():
        if not line or line.startswith('#'):
            continue
        if line == 'end':
            yield case
            case = {}
        else:
            key, value = line.split(':', 1)
            case[key] = value.strip()


def main():
    checks = []
    g = 9.80665
    for label, xyzw, expected_angles, expected_force in [
        ('identity', [0,0,0,1], [0,0,0], [0,0,g]),
        ('roll_30', [math.sin(math.pi/12),0,0,math.cos(math.pi/12)], [30,0,0], [0,g*.5,g*math.sqrt(3)/2]),
        ('pitch_30', [0,math.sin(math.pi/12),0,math.cos(math.pi/12)], [0,30,0], [-g*.5,0,g*math.sqrt(3)/2]),
        ('yaw_90', [0,0,math.sqrt(.5),math.sqrt(.5)], [0,0,90], [0,0,g]),
    ]:
        value = orientation_values(xyzw)
        np.testing.assert_allclose(value['rpy_deg'], expected_angles, atol=1e-10)
        np.testing.assert_allclose(value['rest_specific_force_from_q'], expected_force, atol=1e-10)
        for multiplier in (-1., 1.5):
            equivalent = orientation_values([multiplier*v for v in xyzw])
            np.testing.assert_allclose(equivalent['rpy_deg'], expected_angles, atol=1e-10)
        checks.append(label+'_angles_force_and_quaternion_equivalence')
    assert orientation_values([0,0,0,0]) is None
    assert orientation_values([0,0,0,1], False) is None
    assert orientation_values([math.nan,0,0,1]) is None
    checks.append('invalid_or_unavailable_orientation_rejected')

    static_errors = {}
    moving_errors = {}
    flip_example = None
    for case in golden_cases():
        if case['expect'] != 'accept' or case['dir'] != 'mcu2pc':
            continue
        decoded = decode_imu(bytes.fromhex(case['hex']), t_mono=0.)
        assert decoded.valid
        w,x,y,z = decoded.quat_wxyz
        value = orientation_values([x,y,z,w])
        if case['group'] == 'static':
            expected = np.asarray(value['rest_specific_force_from_q'])
            error = float(np.linalg.norm(decoded.accel-expected))
            assert error < .05, (case['case'], error)
            static_errors[case['case']] = error
            if case['case'] == 'imu_static_tilt_pitch':
                flipped = decoded.accel*np.array([-1.,1.,1.])
                wrong_error = float(np.linalg.norm(flipped-expected))
                assert wrong_error > 3.
                flip_example = {'default_residual_m_s2':error,'flip_x_accel_only_residual_m_s2':wrong_error}
        if case['group'] in ('physical_yaw_pos','physical_pitch_pos'):
            state = dict(item.split('=') for item in case['state'].split())
            theta, psid, thetad = (float(state[k]) for k in ('theta','psid','thetad'))
            expected = np.array([-psid*math.sin(theta),thetad,psid*math.cos(theta)])
            error = float(np.max(np.abs(decoded.gyro-expected)))
            assert error < .002, (case['case'], error)
            moving_errors[case['case']] = error
    assert static_errors and moving_errors and flip_example is not None
    checks.extend(['production_decoder_static_force_matches_q',
                   'production_decoder_gyro_matches_tilted_yaw_pitch_model',
                   'flipping_accel_axis_without_rotating_q_breaks_consistency'])
    report = {'passed':True,'checks':checks,'static_case_residuals_m_s2':static_errors,
              'moving_case_max_axis_errors_rad_s':moving_errors,'sign_flip_counterexample':flip_example,
              'physical_devices_opened':0,'ros_nodes_started':0,
              'limitations':['黄金帧/已知模型，不是现场固件或真实IMU',
                             '不确认板型、芯片到PCB安装或实际S到F矩阵',
                             '只读工具的实际DDS通信与实机联调待现场验证']}
    output = ROOT/'build/offline/gimbal_axes_result.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
