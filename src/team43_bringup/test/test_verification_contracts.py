"""验证器的反例回归：故障、空窗口和低可用率必须真正判失败。"""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def aiming():
    return load('verify_aiming_robustness')


def test_all_pnp_rejected_cannot_pass_static_scene(aiming, monkeypatch):
    monkeypatch.setattr(aiming, 'SCENARIOS', [('clean', {}, 1., .02)])
    monkeypatch.setattr(aiming.aim_sim, 'render_scene', lambda *a, **kw: (None, None))
    monkeypatch.setattr(aiming.VP, 'feed', lambda *a: (NS(valid=True, marker_id=3), NS(valid=False)))
    report, failures = {'static': {}}, []
    aiming.run_static(None, None, failures, report)
    assert report['static']['clean']['pnp_valid'] == 0
    assert any('PnP 有效率' in f for f in failures)


@pytest.mark.parametrize('values,stopped', [
    ([], False), ([1., 1.], False), ([1., 1., 0.], False),
    ([0., 0., 0.], True), ([.1, .05, 0., 0., 0.], True),
    ([.1, 0., 0., 0., .1], False), ([.1, .1, .1, 0., 0., 0.], False),
    ([0., 0., 0., 0.], False),
])
def test_loss_requires_actual_stop_and_silence(aiming, values, stopped):
    commands = [NS(angular=NS(z=value, y=0.)) for value in values]
    failures = []
    assert aiming.check_loss_commands(commands, 'loss', failures)['stopped'] is stopped
    assert bool(failures) is not stopped


@pytest.mark.parametrize('tail,passed', [
    ([1.] * 15, True), ([1.] * 14 + [None], True),
    ([1.] * 13 + [None] * 2, False), ([1.] * 14 + [20.], False),
    ([1.] + [None] * 14, False), ([float('nan')] * 15, False),
])
def test_final_error_requires_fresh_sustained_observations(aiming, tail, passed):
    failures = []
    assert aiming.check_settled(tail, 'closed', failures) is passed
    assert bool(failures) is not passed


@pytest.mark.parametrize('invalid,score,passed', [
    ([0, 605, 606, 607, 608], .022, True),
    ([0, 605, 606, 607, 608], .5, False),
    ([0, 605, 606, 607, 608], float('nan'), False),
    ([0], .001, False),
    ([0, *range(605, 630)], .022, False),
    ([0, 100, 605, 606, 607, 608], .022, False),
])
def test_pipeline_checks_score_availability_loss_and_recovery(invalid, score, passed):
    pipeline = load('verify_pipeline_offline')
    report = dict(ticks=1800, invalid_ticks=invalid, synthetic_score=score)
    assert pipeline.check_acceptance(report) is passed
