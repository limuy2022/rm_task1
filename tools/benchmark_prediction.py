"""Measure production StampedTracker at t and t + 100 ms; no ROS required.

Default: deterministic synthetic positions with 2 cm observation noise, latency,
jitter, random drops and one 0.7 s occlusion. Predictions see only observations
whose arrival time is <= the tick. Truth is used for scoring only. An optional
PointStamped teaching bag or CSV can supply recorded truth/observations.

CSV columns: t,x,y[,obs_x,obs_y,arrival_t]. Times are seconds in one clock.
Rows define truth; absent obs columns generate seeded noisy observations at the
requested rates. With obs columns, blank observations mean no observation;
arrival_t defaults to t. Covariance remains --noise-std squared. No tuning is
performed. Outputs are offline baselines, not a NUC accuracy certification.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/team43_estimation'))
from team43_estimation.ca_kalman import CAKalman
from team43_estimation.stamped_tracker import StampedTracker


@dataclass
class Observation:
    stamp: float
    arrival: float
    position: np.ndarray


class RecordedTruth:
    def __init__(self, stamps, xy):
        self.stamps = np.asarray(stamps, float)
        self.xy = np.asarray(xy, float).reshape(-1, 2)
        if (len(self.stamps) < 2 or len(self.stamps) != len(self.xy)
                or not np.isfinite(self.stamps).all() or not np.isfinite(self.xy).all()
                or np.any(np.diff(self.stamps) <= 0)):
            raise ValueError('truth must contain finite, strictly increasing timestamps')

    def __call__(self, stamp):
        if stamp < self.stamps[0] or stamp > self.stamps[-1]:
            return None  # Never extrapolate scoring truth beyond the recording.
        return np.array([np.interp(stamp, self.stamps, self.xy[:, i]) for i in (0, 1)])


def synthetic_truth(name):
    def truth(t):
        if name == 'stationary': return np.array([3., .5])
        if name == 'constant_velocity': return np.array([1. + .6*t, .3 - .2*t])
        if name == 'constant_acceleration':
            return np.array([1. + .2*t + .02*t*t, .3 + .1*t - .0125*t*t])
        if name == 'turning': return np.array([2.*math.cos(.4*t), 2.*math.sin(.4*t)])
        if name == 'stop_start':
            # Continuous position, abrupt velocity changes at 4, 8, 12, 16 s.
            x = .7*min(t, 4.) + .7*max(0., min(t-8., 4.))
            y = -.4*max(0., min(t-12., 4.))
            return np.array([1.+x, .3+y])
        raise ValueError(name)
    return truth


def generated_observations(truth, duration, rate, args, seed):
    rng = np.random.default_rng(seed)
    result, counts = [], Counter()
    for stamp in np.arange(0., duration+1e-8, 1./rate):
        counts['scheduled'] += 1
        if 9. <= stamp < 9.7:
            counts['occlusion_dropped'] += 1; continue
        if rng.random() < args.dropout:
            counts['random_dropped'] += 1; continue
        position = truth(float(stamp))
        if position is None: continue
        arrival = stamp + max(0., args.latency + rng.normal(0., args.jitter))
        result.append(Observation(float(stamp), float(arrival),
                                  position + rng.normal(0., args.noise_std, 2)))
    return sorted(result, key=lambda o: (o.arrival, o.stamp)), dict(counts)


def stats(errors):
    if not errors: return {'n': 0, 'rmse_m': None, 'mean_m': None, 'p95_m': None, 'max_m': None}
    e = np.asarray(errors, float)
    norm = np.linalg.norm(e, axis=1)
    return {'n': len(norm), 'rmse_m': float(np.sqrt(np.mean(norm**2))),
            'mean_m': float(norm.mean()), 'p95_m': float(np.percentile(norm, 95)),
            'max_m': float(norm.max()), 'axis_rmse_m': np.sqrt(np.mean(e**2, axis=0)).tolist()}


def feed_until(observations, index, tick, tracker, covariance, history, counts):
    while index < len(observations) and observations[index].arrival <= tick + 1e-10:
        obs = observations[index]; index += 1
        # now is the actual arrival, not the later scoring tick.
        accepted = tracker.observe(obs.stamp, obs.position, covariance, now=obs.arrival)
        counts['accepted' if accepted else 'rejected'] += 1
        counts['result_'+tracker.reason] += 1
        # Baselines use the same delivered observations, without a truth oracle.
        if obs.arrival - obs.stamp <= tracker.max_delay and obs.stamp <= obs.arrival:
            if not history or obs.stamp > history[-1].stamp:
                history.append(obs)
                history[:] = [o for o in history if o.stamp >= obs.stamp-1.]
    return index


def evaluate(name, truth, observations, duration, args, generation=None, rate=None):
    tracker = StampedTracker(CAKalman())  # Preserve production default parameters.
    covariance = np.eye(2)*args.noise_std**2
    counts, history, rows = Counter(), [], []
    errors = {part: {method: [] for method in ('ca_current', 'ca_100ms',
              'hold_100ms', 'cv_fit_100ms')} for part in ('all', 'fresh', 'coasting', 'maneuver')}
    index = 0
    for tick in np.arange(0., duration-args.horizon+1e-8, 1./args.tick_hz):
        index = feed_until(observations, index, tick, tracker, covariance, history, counts)
        sample = tracker.sample(float(tick), args.horizon)
        counts['ticks_total'] += 1
        if tick < args.warmup:
            counts['ticks_warmup'] += 1; continue
        current_truth, future_truth = truth(float(tick)), truth(float(tick+args.horizon))
        if current_truth is None or future_truth is None:
            counts['ticks_no_truth'] += 1; continue
        if not sample.valid:
            counts['ticks_tracker_invalid'] += 1; continue
        if len(history) < 2 or tick-history[-1].stamp > tracker.lost_timeout:
            counts['ticks_baseline_invalid'] += 1; continue
        counts['ticks_evaluated'] += 1
        status = 'fresh' if sample.status == 1 else 'coasting'
        counts['ticks_'+status] += 1
        latest = history[-1]
        fit = [o for o in history if o.stamp >= latest.stamp-.5]
        if len(fit) < 2: fit = history[-2:]
        times = np.asarray([o.stamp-latest.stamp for o in fit])
        values = np.asarray([o.position for o in fit])
        # Least squares CV using measured positions only. Intercept is denoised.
        velocity, intercept = np.linalg.lstsq(np.column_stack([times, np.ones(len(times))]),
                                            values, rcond=None)[0]
        cv_prediction = intercept + velocity*(tick+args.horizon-latest.stamp)
        estimates = {'ca_current': sample.position-current_truth,
                     'ca_100ms': sample.prediction-future_truth,
                     'hold_100ms': latest.position-future_truth,
                     'cv_fit_100ms': cv_prediction-future_truth}
        maneuver = ((name == 'turning') or
                    (name == 'stop_start' and any(abs(tick-s) <= .6 for s in (4, 8, 12, 16))))
        for method, error in estimates.items():
            errors['all'][method].append(error)
            errors[status][method].append(error)
            if maneuver: errors['maneuver'][method].append(error)
        rows.append([name, rate if rate is not None else '', float(tick), sample.age,
                     sample.status, *sample.position, *sample.prediction, *future_truth,
                     *(float(np.linalg.norm(estimates[m])) for m in estimates)])
    return {'name': name, 'observation_hz': rate, 'duration_s': duration,
            'generation': generation or {}, 'counts': dict(counts),
            'metrics': {part: {method: stats(e) for method, e in methods.items()}
                        for part, methods in errors.items()}}, rows


def read_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        records = list(csv.DictReader(stream))
    stamps = [float(r['t']) for r in records]
    origin = stamps[0]
    truth = RecordedTruth(np.asarray(stamps)-origin,
                          [[float(r['x']), float(r['y'])] for r in records])
    observations = None
    if 'obs_x' in records[0] or 'obs_y' in records[0]:
        observations = []
        for record, stamp in zip(records, stamps):
            if not record.get('obs_x', '').strip() or not record.get('obs_y', '').strip(): continue
            arrival = float(record.get('arrival_t') or stamp)
            xy = np.array([float(record['obs_x']), float(record['obs_y'])])
            if arrival < stamp or not np.isfinite(xy).all():
                raise ValueError('CSV observations require arrival_t >= t and finite obs_x/obs_y')
            observations.append(Observation(stamp-origin, arrival-origin, xy))
        observations.sort(key=lambda o: (o.arrival, o.stamp))
    return truth, observations, float(truth.stamps[-1])


def decode_point_stamped(data):
    """Decode only standard XCDR1 PointStamped; reject other encodings/types."""
    if len(data) < 40 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('only XCDR1 PointStamped is supported')
    endian = '<' if data[1] == 1 else '>'
    sec, nanosec, length = struct.unpack_from(endian+'iII', data, 4)
    if length < 1 or 16+length > len(data) or nanosec >= 1_000_000_000:
        raise ValueError('invalid PointStamped header')
    frame = data[16:16+length-1].decode('utf-8')
    offset = 4 + ((12+length+7)//8)*8  # Align float64 relative to CDR payload.
    xyz = struct.unpack_from(endian+'ddd', data, offset)
    return sec+nanosec*1e-9, frame, np.array(xyz[:2])


def read_teaching_bag(path):
    connection = sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True)
    streams = {}
    try:
        for name in ('/class2/marker_world', '/virtual_gimbal/target_gt'):
            topic = connection.execute('SELECT id,type,serialization_format FROM topics WHERE name=?',
                                       (name,)).fetchone()
            if topic is None or topic[1:] != ('geometry_msgs/msg/PointStamped', 'cdr'):
                raise ValueError(f'expected CDR PointStamped topic {name}')
            values = []
            for arrival_ns, data in connection.execute(
                    'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic[0],)):
                stamp, frame, xy = decode_point_stamped(data)
                if frame != 'world' or not np.isfinite(xy).all():
                    raise ValueError('teaching bag requires finite, world-frame PointStamped positions')
                values.append((stamp, arrival_ns*1e-9, xy))
            streams[name] = values
    finally: connection.close()
    gt = sorted(streams['/virtual_gimbal/target_gt'], key=lambda item: item[0])
    # A repeated stamp represents the last value; truth interpolation is scoring only.
    gt = list({row[0]: row for row in gt}.values())
    origin = gt[0][0]
    truth = RecordedTruth([g[0]-origin for g in gt], [g[2] for g in gt])
    observations = [Observation(stamp-origin, arrival-origin, xy)
                    for stamp, arrival, xy in streams['/class2/marker_world']]
    return truth, observations, float(truth.stamps[-1])


def self_checks():
    # The production prediction is from scoring tick + 0.1, not image stamp + 0.1.
    tr = StampedTracker()
    for i in range(150):
        t = i/30.
        assert tr.observe(t, [1.+.6*t, .3-.2*t], np.eye(2)*1e-4, now=t+.04)
    before, source = tr.kf.x.copy(), tr.kf.t_last
    s = tr.sample(t+.04)
    np.testing.assert_allclose(s.prediction, [1.+.6*(t+.14), .3-.2*(t+.14)], atol=.003)
    np.testing.assert_allclose(s.prediction-s.position, [.06, -.02], atol=.001)
    np.testing.assert_array_equal(tr.kf.x, before)
    assert tr.kf.t_last == source
    tr = StampedTracker(); counts, history = Counter(), []
    pending = [Observation(0., .04, np.array([1., 2.])),
               Observation(.1, .15, np.array([99., 99.]))]
    index = feed_until(pending, 0, .1, tr, np.eye(2)*.01, history, counts)
    assert index == 1 and tr.last_observation == 0.
    np.testing.assert_allclose(tr.sample(.1).position, [1., 2.])
    truth = RecordedTruth([0., 1.], [[0., 0.], [1., 2.]])
    assert truth(-.01) is None and truth(1.01) is None
    np.testing.assert_allclose(truth(.5), [.5, 1.])
    return {'100ms_from_tick_including_latency': 'PASS', 'no_future_observation_leakage': 'PASS',
            'sample_does_not_advance_filter': 'PASS', 'truth_not_extrapolated': 'PASS'}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output-dir', type=Path, default=ROOT/'build/offline/prediction')
    parser.add_argument('--csv', type=Path)
    parser.add_argument('--teaching-bag', type=Path)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--rates', type=float, nargs='+', default=[10., 30.])
    parser.add_argument('--noise-std', type=float, default=.02)
    parser.add_argument('--latency', type=float, default=.04)
    parser.add_argument('--jitter', type=float, default=.015)
    parser.add_argument('--dropout', type=float, default=.03)
    parser.add_argument('--warmup', type=float, default=2.)
    parser.add_argument('--tick-hz', type=float, default=10.)
    parser.add_argument('--horizon', type=float, choices=[.1], default=.1)
    parser.add_argument('--seed', type=int, default=47)
    args = parser.parse_args()
    if (not all(math.isfinite(v) and v > 0 for v in
                (args.duration, args.noise_std, args.tick_hz, args.horizon, *args.rates))
            or not all(math.isfinite(v) and v >= 0 for v in (args.latency,args.jitter,args.warmup))
            or not 0 <= args.dropout < 1 or args.duration <= args.warmup+args.horizon):
        parser.error('invalid finite/range parameter')
    cases, all_rows = [], []
    checks = self_checks()
    names = ('stationary', 'constant_velocity', 'constant_acceleration', 'turning', 'stop_start')
    for i, name in enumerate(names):
        truth = synthetic_truth(name)
        for rate in args.rates:
            observations, generation = generated_observations(truth,args.duration,rate,args,args.seed+i*100+int(rate))
            result, rows = evaluate(name,truth,observations,args.duration,args,generation,rate)
            cases.append(result); all_rows.extend(rows)
    sources = []
    for kind, path, reader in (('user_csv',args.csv,read_csv),
                               ('teaching_virtual_gimbal_bag',args.teaching_bag,read_teaching_bag)):
        if path is None: continue
        truth, observations, duration = reader(path)
        generation = {'observations': 'recorded; arrival time respected'}
        if observations is None:
            observations, generation = generated_observations(truth,duration,args.rates[-1],args,args.seed)
        result, rows = evaluate(kind,truth,observations,duration,args,generation)
        cases.append(result); all_rows.extend(rows)
        sources.append({'kind':kind,'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                        'truth_max_interpolation_gap_s':float(np.diff(truth.stamps).max()),
                        'observations_total':len(observations)})
    kf_defaults, tracker_defaults = CAKalman(), StampedTracker()
    report = {'kind':'offline prediction accuracy baseline', 'real_nuc_accuracy_verified':False,
              'self_checks':checks, 'parameters':{k:v for k,v in vars(args).items() if k not in
                  ('output_dir','csv','teaching_bag')},
              'production_filter':{'state':'px,py,vx,vy,ax,ay',
                  **{key:getattr(kf_defaults,key) for key in ('jerk_std','max_dt','nis_gate',
                    'cv_degrade_enable','init_sigma_p','init_sigma_v','init_sigma_a')},
                  'max_observation_delay_s':tracker_defaults.max_delay,
                  'lost_timeout_s':tracker_defaults.lost_timeout,
                  'fresh_timeout_s':tracker_defaults.fresh_timeout},
              'metric_definition':'Euclidean XY error in metres; CA current scored at tick; all 100ms '
                  f'methods scored at tick+0.1. Same CA-valid tick mask after {args.warmup}s warmup. CV fits only '
                  'delivered measured positions in trailing 0.5s; no true velocity. Invalid and warmup '
                  'ticks counted separately. Recorded truth is linearly interpolated for scoring only.',
              'limitations':['Synthetic noise is independent XY Gaussian; it does not model PnP depth '
                  'bias, TF/extrinsic error, clock drift or NUC CPU latency.',
                  'Abrupt maneuvers challenge a constant-acceleration model. Do not claim all targets '
                  'have the same accuracy; inspect maneuver and dropout counts.',
                  'Teaching bag is a virtual gimbal recording, not a real chassis validation.'],
              'sources':sources,'cases':cases}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    (args.output_dir/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with (args.output_dir/'tick_errors.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['scenario','observation_hz','tick','last_obs_age','track_status','cur_x','cur_y',
                         'pred_x','pred_y','truth_future_x','truth_future_y','ca_current_error_m',
                         'ca_100ms_error_m','hold_100ms_error_m','cv_fit_100ms_error_m'])
        writer.writerows(all_rows)
    def display(value): return 'N/A' if value is None else f'{value:.4f}m'
    for case in cases:
        metrics=case['metrics']['all']; counts=case['counts']
        print(f"{case['name']:28} {str(case['observation_hz']):>4}Hz "
              f"valid={counts.get('ticks_evaluated',0):4} invalid={counts.get('ticks_tracker_invalid',0):3} "
              f"CA100={display(metrics['ca_100ms']['rmse_m'])} "
              f"hold={display(metrics['hold_100ms']['rmse_m'])} CVfit={display(metrics['cv_fit_100ms']['rmse_m'])}")
    print(f'Report: {args.output_dir}/summary.json (offline; not NUC precision)')
    return 0


if __name__ == '__main__': raise SystemExit(main())
