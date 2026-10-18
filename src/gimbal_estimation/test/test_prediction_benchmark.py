"""Check benchmark clock separation and its recorded-data adapters, without ROS."""
from collections import Counter
import importlib.util
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('benchmark_prediction', ROOT/'tools/benchmark_prediction.py')
benchmark = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = benchmark
spec.loader.exec_module(benchmark)


def test_production_clock_and_future_leakage_self_checks():
    assert set(benchmark.self_checks().values()) == {'PASS'}


def test_loss_invalid_ticks_accounted_and_do_not_disappear_from_denominator():
    args = SimpleNamespace(noise_std=.01, horizon=.1, tick_hz=10., warmup=.2)
    truth = benchmark.synthetic_truth('constant_velocity')
    observations = [benchmark.Observation(t,t+.04,truth(t)) for t in np.arange(0.,.5,.1)]
    result, rows = benchmark.evaluate('constant_velocity',truth,observations,2.,args)
    counts = result['counts']
    assert counts['ticks_tracker_invalid'] > 0
    assert counts['ticks_total'] == (counts['ticks_warmup'] + counts['ticks_evaluated']
                                      + counts['ticks_tracker_invalid'])
    assert len(rows) == counts['ticks_evaluated']
    for method in result['metrics']['all'].values():
        assert method['n'] == counts['ticks_evaluated']


@pytest.mark.parametrize('endian,encapsulation',[('<',b'\x00\x01\x00\x00'),('>',b'\x00\x00\x00\x00')])
def test_point_stamped_cdr_alignment_and_endian(endian,encapsulation):
    frame = b'world\x00'
    payload = struct.pack(endian+'iII',123,250000000,len(frame)) + frame
    payload += b'\x00' * ((-len(payload)) % 8)
    payload += struct.pack(endian+'ddd',1.25,-2.5,3.)
    stamp, frame, xy = benchmark.decode_point_stamped(encapsulation+payload)
    assert stamp == 123.25 and frame == 'world'
    np.testing.assert_allclose(xy,[1.25,-2.5])


def test_csv_arrival_preserved_and_bad_clock_rejected(tmp_path):
    path = tmp_path/'trajectory.csv'
    path.write_text('t,x,y,obs_x,obs_y,arrival_t\n100,1,2,1.1,1.9,100.2\n101,2,3,,,\n')
    truth, observations, duration = benchmark.read_csv(path)
    assert duration == 1. and len(observations) == 1
    assert observations[0].stamp == 0.
    assert observations[0].arrival == pytest.approx(.2)
    np.testing.assert_allclose(truth(.5),[1.5,2.5])
    path.write_text('t,x,y,obs_x,obs_y,arrival_t\n100,1,2,1,2,99\n101,2,3,,,\n')
    with pytest.raises(ValueError,match='arrival_t'): benchmark.read_csv(path)
