"""短时评分位置补偿：不续期、不跨目标/会话、不放开建议速度。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from team43_judge_io.tick_sampler import TickSampler


def put(s, t=10., **kwargs):
    s.put_target(t, [2., 3.], [.4, -.2], [.2, .1], **kwargs)


def test_invalid_packets_use_last_valid_prediction_without_enabling_advice():
    s=TickSampler();put(s)
    s.put_target(10.1,[0.,0.],[0.,0.],[0.,0.],valid=False)
    snap=s.sample(10.2)
    np.testing.assert_allclose(snap.current,[2.084,2.962])
    np.testing.assert_allclose(snap.prediction,[2.129,2.9445])
    assert snap.target_fallback and not snap.target_valid
    # 新无效包不能把补偿期限不断推后。
    for t in (10.3,10.4,10.5,10.6):
        s.put_target(t,[0.,0.],[0.,0.],[0.,0.],valid=False)
    assert not s.sample(10.6).target_fallback
    np.testing.assert_array_equal(s.sample(10.7).current,[0.,0.])


def test_missing_state_uses_bounded_prediction_then_expires():
    s=TickSampler();put(s)
    assert s.sample(10.3).target_fallback
    assert s.sample(10.5).target_fallback
    assert not s.sample(10.5001).target_fallback


def test_observation_age_is_not_reset_by_publishing_coasting_states():
    s=TickSampler();put(s,10.49,observation_age=.49)
    assert s.sample(10.8).target_fallback
    # 已外推0.49秒的最后有效状态，另补最多0.5秒，不能重新变成新观测。
    assert not s.sample(11.01).target_fallback


def test_bad_motion_keeps_finite_position_and_zero_is_legitimate():
    s=TickSampler()
    s.put_target(10.,[0.,3.],[float('nan'),0.],[0.,0.])
    snap=s.sample(10.1)
    np.testing.assert_array_equal(snap.current,[0.,3.])
    np.testing.assert_array_equal(snap.prediction,[0.,3.])
    assert snap.target_fallback and not snap.target_valid


def test_future_samples_cannot_leak_into_earlier_tick():
    s=TickSampler();put(s,11.)
    snap=s.sample(10.9)
    assert not snap.target_fallback and not snap.target_valid
    np.testing.assert_array_equal(snap.current,[0.,0.])


def test_clear_and_source_barrier_reject_old_track_even_with_new_message_stamp():
    s=TickSampler();put(s)
    s.clear_target(10.1)
    put(s,10.2,observation_age=.2)
    assert not s.sample(10.25).target_fallback
    put(s,10.3)
    assert s.sample(10.35).target_valid
    s.clear(10.4)
    assert not s.sample(10.45).target_fallback


def test_clock_rewind_clears_cached_positions():
    s=TickSampler();put(s);assert s.sample(10.1).target_valid
    assert not s.sample(9.9).target_fallback
    assert not s.sample(10.2).target_fallback


def test_disabled_fallback_preserves_zero_behavior_and_rejects_unbounded_limit():
    s=TickSampler(target_fallback_timeout=0.);put(s)
    assert not s.sample(10.3).target_fallback
    for bad in (-1.,.501,float('inf'),float('nan')):
        with pytest.raises(ValueError):TickSampler(target_fallback_timeout=bad)


def harness(tmp_path):
    root=Path(__file__).resolve().parents[3]
    spec=importlib.util.spec_from_file_location('fallback_timing',root/'tools/verify_timing_offline.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.judge_node(str(tmp_path)),module


def test_real_judge_writes_fallback_but_no_velocity_and_flushes(tmp_path):
    app,m=harness(tmp_path);put(app._sampler,1000.)
    app._sampler.put_target(1000.1,[0.,0.],[0.,0.],[0.,0.],valid=False)
    app._sampler.put_base(1000.15,[0.,0.],[0.,0.])
    app._sampler.put_advice(1000.15,[.3,0.])
    app._on_tick(NS(stamp=m.stamp(1000.2),frame_id='0'))
    values=list(map(float,(tmp_path/'43.csv').read_text().strip().split(',')))
    assert len(values)==9 and values[2]>2 and values[3]>2.9
    assert values[6:8]==[0.,0.] and values[8]==1000.2
    assert not app._has_target and app._n_target_fallback==1
    app._writer.close()


def test_target_switch_epoch_and_origin_clear_real_judge_cache(tmp_path):
    app,m=harness(tmp_path);put(app._sampler,1000.1)
    event=NS(node='/aruco_detector_node',stamp=m.stamp(1000.2),new_parameters=[],
             changed_parameters=[NS(name='perception.target_digit')],deleted_parameters=[])
    app._on_target_parameters(event)
    assert not app._sampler.sample(1000.3).target_fallback
    put(app._sampler,1000.3);app._on_epoch(NS(data=1));app._on_epoch(NS(data=2))
    assert not app._sampler.sample(1000.4).target_fallback
    put(app._sampler,1000.5)
    app._freeze_origin(NS(stamp=m.stamp(1000.6),frame_id='record'))
    assert not app._sampler.sample(1000.7).target_fallback
    app._writer.close()


def test_late_old_switch_event_does_not_roll_back_new_target_barrier():
    s=TickSampler();s.clear_target(10.2);put(s,10.3)
    s.clear_target(10.1)
    assert s.sample(10.35).target_valid
    put(s,10.36,observation_age=.2)
    # 后到的旧观测不能替换10.3的新目标缓存。
    s.put_target(10.4,[0.,0.],[0.,0.],[0.,0.],valid=False)
    assert s.sample(10.45).target_fallback
    assert s.sample(10.46).current[0]>2.
