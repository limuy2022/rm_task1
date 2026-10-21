"""1800 virtual ticks through real PnP, geometry, filter, sampler, advice, writer.

Synthetic known geometry, 40ms image transport delay, pixel noise and a 0.8s
occlusion. Runs inside the ROS 2 Humble container but starts no ROS nodes and
exchanges no DDS messages: no physical device, official scoring or NUC deadline.
Outputs are explicitly synthetic and live under build/offline/pipeline only.
"""
import csv
import json
import math
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for pkg in ('gimbal_common','gimbal_perception','gimbal_localization','gimbal_estimation',
            'gimbal_judge_io','chassis_advisor','grid_mapper'):
    sys.path.insert(0, str(ROOT/'src'/pkg))
from gimbal_common import frames as FR
from gimbal_perception.pnp_solver import PnpSolver
from gimbal_localization.gimbal_kinematics import GimbalKinematics
from gimbal_estimation.observation_builder import ObservationBuilder
from gimbal_estimation.stamped_tracker import StampedTracker
from gimbal_judge_io.tick_sampler import TickSampler
from gimbal_judge_io.tick_csv import TickCsvWriter
from chassis_advisor.free_dir import FreeDirSolver
from grid_mapper.pgm_io import write_map, read_grid
from grid_mapper.grid_builder import GridMap
from check_csv import check_csv
from score_csv import evaluate
from verify_kinematics_offline import calibration


def target(t): return np.array([2.5+.004*t, .3*math.sin(.15*t), .55])


def base(t): return np.array([.2*math.sin(.1*t), .1*math.sin(.13*t), .02*math.sin(.2*t)])


def base_velocity(t): return np.array([.02*math.cos(.1*t), .013*math.cos(.13*t)])


def camera(model, t):
    R = FR.rpy_to_mat([.02*math.sin(.1*t), .03*math.sin(.12*t), .2*math.sin(.05*t)])
    return FR.compose_T(R, base(t))@model.forward(.08*math.sin(.5*t), .03*math.sin(.3*t)).base_optical


def check_acceptance(report):
    """完整180秒均参与可用率/评分验收，精度诊断的排除窗口不能掩盖失效。"""
    invalid = set(report['invalid_ticks'])
    normal_invalid = [k for k in invalid if k >= 20 and not 600 <= k < 630]
    valid_fraction = 1. - len(invalid) / report['ticks']
    longest = consecutive = 0
    for tick in range(report['ticks']):
        consecutive = consecutive + 1 if tick in invalid else 0
        longest = max(longest, consecutive)
    recovery = next((k for k in range(609, report['ticks']) if k not in invalid), None)
    checks = {
        'all_ticks_present': report['ticks'] == 1800,
        'valid_fraction': valid_fraction >= .99,
        'normal_window_available': not normal_invalid,
        'bounded_loss': longest <= 6,
        'loss_exercised': any(k in invalid for k in range(605, 608)),
        'recovered_after_occlusion': recovery is not None and recovery <= 612,
        'score_budget': report['synthetic_score'] is not None
                        and math.isfinite(report['synthetic_score']) and 0 <= report['synthetic_score'] <= .04,
    }
    report.update(checks=checks, valid_fraction=valid_fraction, longest_invalid_ticks=longest,
                  first_recovered_tick=recovery, acceptance_score_max=.04)
    return all(checks.values())


def main():
    start = time.perf_counter(); output = ROOT/'build/offline/pipeline'; output.mkdir(parents=True, exist_ok=True)
    # Never overwrite/resume a real submission; this is an owned synthetic fixture.
    submitted = output/'synthetic_submission.csv'
    if submitted.exists(): submitted.unlink()
    c = calibration()
    c['base_mount_rpy'] = [0.,0.,0.]; c['pitch_gimbal_rpy'] = [0.,0.,0.]
    c['gimbal_camera_rpy'] = [0.,0.,0.]; c['joint_zero_rad'] = [0.,0.]
    model = GimbalKinematics(c)
    cam = yaml.safe_load((ROOT/'src/gimbal_tracker_bringup/config/camera_team47.yaml').read_text())['camera']
    pnp = PnpSolver.from_camera_config(cam, marker_size_m=.10)
    builder = ObservationBuilder(); tracker = StampedTracker(); sampler = TickSampler()
    arr = np.zeros((200, 120), np.int8); arr[0,:] = -1; arr[160:162,80:92] = 100
    free = FreeDirSolver(block_unknown=True); free.set_grid(arr, [-2.,-2.], .05)
    rng = np.random.default_rng(47)
    writer = TickCsvWriter(str(submitted)); assert writer.open()
    errors, pred_errors, base_errors, invalid = [], [], [], []
    reference = []; image_index = 0; n_pnp = 0
    for tick in range(1800):
        t = tick*.1; ros_t = 1000.+t
        while image_index/30 <= t-.04+1e-9:
            image_t = image_index/30; image_index += 1
            if 60. <= image_t < 60.8: continue
            T = camera(model, image_t)
            optical = np.linalg.inv(T)@np.r_[target(image_t), 1.]
            uv, _ = cv2.projectPoints(pnp._obj, np.array([math.pi,0.,0.]), optical[:3], pnp.K, pnp.D)
            uv = uv.reshape(4,2) + rng.normal(0, .1, (4,2))
            result = pnp.solve(uv); assert result.ok, result.reason
            obs = builder.build_from_transform(result.tvec_opt, np.diag([1e-5,1e-5,.001]), T)
            assert obs.valid
            tracker.observe(1000.+image_t, obs.pos_map[:2], obs.R_map, now=ros_t, marker_id=3)
            n_pnp += 1
        state = tracker.sample(ros_t-.001)
        sampler.put_target(ros_t-.001, state.position, state.velocity, state.acceleration,
                           valid=state.valid, observation_age=state.age)
        sampler.put_base(ros_t-.01, base(t-.01)[:2], base_velocity(t-.01))
        direction, _, feasible = free.solve_unit(base(t)[:2], state.position-base(t)[:2])
        sampler.put_advice(ros_t-.001, direction*.3 if feasible and state.valid else [0,0],
                           valid=feasible and state.valid)
        snap = sampler.sample(ros_t)
        assert writer.write_tick(snap.prediction, snap.current, snap.base, snap.advice, ros_t)
        # A second reader sees the flushed complete row before writer.close().
        if tick in (0, 99, 1799):
            with submitted.open() as stream: assert sum(1 for _ in stream) == tick+1
        truth = target(t); future = target(t+.1); b = base(t)[:2]
        angle = math.degrees(math.atan2(truth[1]-b[1], truth[0]-b[0]))
        reference.append([f'{ros_t:.6f}', *future[:2], *truth[:2], *b, angle])
        if not snap.target_valid: invalid.append(tick)
        # Explicit warm-up exclusions apply only to accuracy diagnostics; the
        # official-rule synthetic score below still includes ALL 1800 ticks.
        if snap.target_valid and t >= 2. and not 60. <= t < 63.:
            errors.append(snap.current-truth[:2]); pred_errors.append(snap.prediction-future[:2])
            base_errors.append(snap.base-b)
    writer.close()
    ref = output/'synthetic_reference.csv'
    with ref.open('w', newline='') as stream:
        out = csv.writer(stream); out.writerow(['t_tick','pred_x','pred_y','cur_x','cur_y','base_x','base_y','std_dir_deg']); out.writerows(reference)
    # Occlusion necessarily creates intentional zero advice and checker stop
    # warnings, so check the structural reader separately from that diagnostic.
    from check_csv import _read_submission
    rows, issues = _read_submission(submitted); assert len(rows) == 1800 and not issues
    grid = GridMap(grid=arr, resolution=.05, origin_xy=np.array([-2.,-2.]))
    paths = write_map(str(output), grid, image_name='synthetic_map.pgm', yaml_name='synthetic_map.yaml')
    np.testing.assert_array_equal(read_grid(paths['pgm']).grid, arr)
    metadata = yaml.safe_load(Path(paths['yaml']).read_text())
    np.testing.assert_allclose(metadata['origin'], [-2., -2., 0.])
    rmse = lambda a: np.sqrt(np.mean(np.asarray(a)**2, axis=0)).tolist()
    report = dict(kind='synthetic_no_hardware', virtual_duration_s=180., ticks=1800,
        pnp_images=n_pnp, pixel_noise_std=.1, image_delay_s=.04, invalid_ticks=invalid,
        diagnostic_exclusions='t<2s and 60<=t<63s; formal synthetic score includes all ticks',
        current_xy_rmse_m=rmse(errors), prediction_xy_rmse_m=rmse(pred_errors), base_xy_rmse_m=rmse(base_errors),
        synthetic_score=evaluate(submitted, ref, field_long_side_m=10.)['total'],
        wall_duration_s=time.perf_counter()-start,
        not_verified=['DDS node-to-node transport','NUC 80ms deadline','real imagery/TF/map/geometry','physical control'])
    assert max(report['current_xy_rmse_m']) < .04, report
    assert max(report['prediction_xy_rmse_m']) < .05, report
    assert max(report['base_xy_rmse_m']) < 1e-5, report
    accepted = check_acceptance(report)
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    assert accepted, report
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
