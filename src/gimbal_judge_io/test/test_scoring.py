"""Official Task 1 nine-column scoring regressions, using synthetic references only."""
from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/gimbal_judge_io'))
sys.path.insert(0, str(ROOT / 'tools'))
from gimbal_judge_io import scoring as S
import check_csv
import score_csv


def perfect_context(n=1):
    return {'pred': [(3., 4.)] * n, 'cur': [(3., 4.)] * n,
            'pred_true': [(3., 4.)] * n, 'cur_true': [(3., 4.)] * n,
            'err_dir_deg': [0.] * n, 'base': [(0., 0.)] * n,
            'base_true': [(0., 0.)] * n, 'field_long_side_m': 10.}


def synthetic_files(tmp_path, *, n=4, direction=True, missing=None, pred_error=.1, cur_error=.2):
    reference = tmp_path / 'synthetic_reference.csv'
    submitted = tmp_path / '47.csv'
    columns = 't_tick,pred_x,pred_y,cur_x,cur_y,base_x,base_y'
    with reference.open('w') as stream:
        stream.write(columns + (',std_dir_deg' if direction else '') + '\n')
        for k in range(n):
            stream.write(f'{k}.5,1,2,3,4,0,0' + (',0' if direction else '') + '\n')
    with submitted.open('w') as stream:
        for k in range(n):
            if k == missing:
                continue
            stream.write(f'{1 + pred_error:.6f},2,3,{4 + cur_error:.6f},0,0,.3,0,{k}.500000\n')
    return str(submitted), str(reference)


def test_weights_include_base_and_sum_to_one():
    assert S.W_HEAD4 == (.05, .05, .15, .15)
    assert S.W_HEAD6 == (.05, .05, .15, .15, .2, .2)
    assert S.W_DIR == .2
    assert sum(S.WEIGHTS.values()) == pytest.approx(1)
    assert S.WEIGHTS['base_x'] == S.WEIGHTS['base_y'] == .2


def test_duplicate_submission_is_ambiguous_not_silently_perfect(tmp_path):
    submitted, reference = synthetic_files(tmp_path, n=1, pred_error=0, cur_error=0)
    p = Path(submitted); p.write_text(p.read_text()*2)
    result = score_csv.evaluate(submitted, reference, field_long_side_m=10.)
    assert result['meta']['ambiguous_submission_rows'] == 1
    assert result['meta']['missing'] == 1
    assert result['total'] == pytest.approx(1.)


def test_heading_and_wrapped_angle():
    assert S.heading_deg(1, 0) == 0
    assert S.heading_deg(0, 1) == 90
    assert S.heading_deg(0, -1) == -90
    assert math.isnan(S.heading_deg(0, 0))
    assert S.angle_between_deg(-170, 170) == pytest.approx(20)
    assert S.angle_between_deg(350, 10) == pytest.approx(20)
    assert S.angle_between_deg(0, 180) == pytest.approx(180)


def test_direction_only_uses_heading_not_speed_magnitude():
    assert S.direction_error_deg(0, .3, 0, 3) == pytest.approx(90)
    assert S.direction_error_deg(0, 30, 0, 3) == pytest.approx(90)
    assert S.direction_error_deg(.019, .019, 45, 3) == pytest.approx(0)


def test_stop_trap_strict_speed_and_true_distance_boundaries():
    assert S.direction_error_deg(.019, 0, 0, 2) == 180
    assert S.direction_error_deg(0, 0, 123, .499999) == 0
    assert S.direction_error_deg(0, 0, 0, .5) == 180
    assert S.direction_error_deg(.02, 0, 180, .2) == 180
    assert S.direction_error_deg(.02, 0, 0, 2) == 0
    assert S.direction_error_deg(0, 0, 0, float('nan')) == 180
    assert S.direction_error_deg(float('nan'), 0, 0, .2) == 180


def test_unknown_reference_direction_is_not_direct_target_bearing():
    assert math.isnan(S.direction_error_deg(.3, 0, float('nan'), 3))


def test_direction_normalization_and_clip_before_rmse():
    term = S.direction_term([18] * 5)
    assert term['rmse_deg'] == pytest.approx(18)
    assert term['norm_rmse'] == pytest.approx(.1)
    assert term['contribution'] == pytest.approx(.02)
    term = S.direction_term([360, 0])
    assert term['norm_rmse'] == pytest.approx(math.sqrt(.5))
    assert term['contribution'] == pytest.approx(.2 * math.sqrt(.5))
    assert not S.direction_term([])['complete']


def test_perfect_all_seven_items_score_zero():
    result = S.score(**perfect_context(10))
    assert result['total'] == pytest.approx(0)
    assert result['weight_sum'] == pytest.approx(1)
    assert result['degraded'] == []


def test_target_errors_are_divided_by_true_distance():
    context = perfect_context(2)
    context['pred'] = [(3.1, 4)] * 2
    context['cur'] = [(3, 4.2)] * 2
    result = S.score(**context)
    assert result['terms']['pred_x']['rmse'] == pytest.approx(.1)
    assert result['terms']['pred_x']['norm_rmse'] == pytest.approx(.02)
    assert result['terms']['pred_x']['contribution'] == pytest.approx(.001)
    assert result['terms']['cur_y']['contribution'] == pytest.approx(.006)
    assert result['total'] == pytest.approx(.007)


def test_true_distance_has_one_meter_floor():
    context = perfect_context()
    context.update(pred=[(1, 0)], pred_true=[(.5, 0)], cur=[(.5, 0)], cur_true=[(.5, 0)])
    result = S.score(**context)
    assert result['terms']['pred_x']['norm_rmse'] == pytest.approx(.5)
    assert result['total'] == pytest.approx(.025)


def test_submitted_positions_cannot_inflate_normalization_distance():
    context = perfect_context()
    context.update(pred=[(8, 4)], base=[(1000, 1000)])
    result = S.score(**context)
    assert result['terms']['pred_x']['norm_rmse'] == pytest.approx(1)
    assert result['terms']['base_x']['norm_rmse'] == pytest.approx(1)
    assert result['terms']['base_y']['norm_rmse'] == pytest.approx(1)
    assert result['total'] == pytest.approx(.45)


def test_target_axes_are_independent_not_euclidean_rmse():
    context = perfect_context()
    context['pred'] = [(6, 8)]
    result = S.score(**context)
    assert result['terms']['pred_x']['norm_rmse'] == pytest.approx(.6)
    assert result['terms']['pred_y']['norm_rmse'] == pytest.approx(.8)
    assert result['total'] == pytest.approx(.07)


def test_base_errors_use_field_long_side_and_have_weight():
    context = perfect_context()
    context['base'] = [(2, -3)]
    result = S.score(**context)
    assert result['terms']['base_x']['norm_rmse'] == pytest.approx(.2)
    assert result['terms']['base_y']['norm_rmse'] == pytest.approx(.3)
    assert result['total'] == pytest.approx(.1)


def test_normalize_every_tick_before_rmse_for_changing_distance():
    context = perfect_context(2)
    context.update(pred=[(2, 0), (11, 0)], pred_true=[(1, 0), (10, 0)],
                   cur=[(1, 0), (10, 0)], cur_true=[(1, 0), (10, 0)])
    result = S.score(**context)
    expected = math.sqrt((1 + .01) / 2)
    assert result['terms']['pred_x']['norm_rmse'] == pytest.approx(expected)
    assert result['total'] == pytest.approx(.05 * expected)


def test_clip_each_tick_before_rmse_for_position_and_base():
    context = perfect_context(2)
    context['pred'] = [(103, 4), (3, 4)]
    context['base'] = [(100, 0), (0, 0)]
    result = S.score(**context)
    expected = math.sqrt(.5)
    assert result['terms']['pred_x']['norm_rmse'] == pytest.approx(expected)
    assert result['terms']['base_x']['norm_rmse'] == pytest.approx(expected)
    assert result['total'] == pytest.approx(.25 * expected)


def test_invalid_submitted_item_gets_maximum_not_filtered_away():
    context = perfect_context()
    context['pred'] = [(float('nan'), 4)]
    result = S.score(**context)
    assert result['terms']['pred_x']['missing_estimates'] == 1
    assert result['terms']['pred_x']['norm_rmse'] == 1
    assert result['total'] == pytest.approx(.05)


def test_all_estimates_missing_gets_full_error_with_known_reference():
    context = perfect_context(3)
    context.update(pred=[], cur=[], base=[], err_dir_deg=[180] * 3)
    assert S.score(**context)['total'] == pytest.approx(1)


def test_empty_or_incomplete_reference_never_reports_zero_total():
    result = S.score([], [], [], [], [])
    assert result['total'] is None
    assert set(result['degraded']) == set(S.WEIGHTS)
    context = perfect_context()
    context['base_true'] = None
    result = S.score(**context)
    assert result['total'] is None
    assert 'pred_x' in result['degraded'] and 'base_x' in result['degraded']


def test_missing_field_length_is_not_silently_assumed():
    context = perfect_context()
    context['field_long_side_m'] = None
    result = S.score(**context)
    assert result['total'] is None
    assert result['degraded'] == ['base_x', 'base_y']


def test_missing_obstacle_reference_direction_leaves_total_undefined():
    context = perfect_context()
    context['err_dir_deg'] = None
    result = S.score(**context)
    assert result['total'] is None
    assert result['degraded'] == ['direction']


def test_report_exposes_base_and_missing_context():
    context = perfect_context()
    context['err_dir_deg'] = None
    result = S.score(**context)
    report = S.format_report(result, meta={'队伍': '47'})
    for text in ('base_x', 'direction', '合计', '未评估', '47', '部分项之和不作为总分'):
        assert text in report


def test_score_csv_end_to_end(tmp_path):
    submitted, reference = synthetic_files(tmp_path)
    result = score_csv.evaluate(submitted, reference, tol=.05, field_long_side_m=10)
    assert result['meta']['matched'] == 4
    assert result['terms']['pred_x']['rmse'] == pytest.approx(.1)
    assert result['terms']['cur_y']['rmse'] == pytest.approx(.2)
    assert result['dir']['rmse_deg'] == pytest.approx(0)
    assert result['total'] == pytest.approx(.007)
    assert '80ms' in result['meta']['latency']


def test_score_csv_missing_tick_receives_maximum_instead_of_disappearing(tmp_path):
    submitted, reference = synthetic_files(tmp_path, missing=2, pred_error=0, cur_error=0)
    result = score_csv.evaluate(submitted, reference, field_long_side_m=10)
    assert result['meta']['matched'] == 3
    assert result['meta']['missing'] == 1
    assert result['terms']['base_x']['norm_rmse'] == pytest.approx(.5)
    assert result['total'] == pytest.approx(.5)


def test_score_csv_without_direction_or_field_length_is_partial(tmp_path):
    submitted, reference = synthetic_files(tmp_path, direction=False)
    result = score_csv.evaluate(submitted, reference, field_long_side_m=10)
    assert result['degraded'] == ['direction']
    assert result['total'] is None
    assert '未提供障碍参考方向' in result['meta']['direction_reference']
    submitted, reference = synthetic_files(tmp_path)
    assert score_csv.evaluate(submitted, reference)['total'] is None


def test_score_csv_uses_reference_distance_for_stopping(tmp_path):
    submitted, reference = synthetic_files(tmp_path, n=1, pred_error=0, cur_error=0)
    Path(submitted).write_text('1,2,0,0,0,0,0,0,0.500000\n')
    result = score_csv.evaluate(submitted, reference, field_long_side_m=10)
    assert result['dir']['norm_rmse'] == 1  # estimated distance=0, TRUE distance=5m


def test_score_csv_does_not_replace_obstacle_direction_with_target_bearing(tmp_path):
    submitted, reference = synthetic_files(tmp_path, n=1, pred_error=0, cur_error=0)
    # TRUE target is north of base, but explicit obstacle-feasible reference is east.
    Path(reference).write_text('t_tick,pred_x,pred_y,cur_x,cur_y,base_x,base_y,std_dir_deg\n'
                               '.5,0,5,0,5,0,0,0\n')
    result = score_csv.evaluate(submitted, reference, field_long_side_m=10)
    assert result['dir']['norm_rmse'] == 0


def test_score_csv_ambiguous_multiple_rows_cannot_reuse_one_reference_tick(tmp_path):
    submitted, reference = synthetic_files(tmp_path, n=1, pred_error=0, cur_error=0)
    Path(submitted).write_text('1,2,3,4,0,0,.3,0,0.450000\n'
                               '1,2,3,4,0,0,.3,0,0.550000\n')
    result = score_csv.evaluate(submitted, reference, tol=.05, field_long_side_m=10)
    assert result['meta']['matched'] == 0
    assert result['meta']['ambiguous_submission_rows'] == 1
    assert result['total'] == pytest.approx(1)


def test_score_csv_invalid_estimate_is_penalized_but_invalid_reference_rejected(tmp_path):
    submitted, reference = synthetic_files(tmp_path, n=1, pred_error=0, cur_error=0)
    Path(submitted).write_text('nan,2,3,4,0,0,.3,0,0.500000\n')
    result = score_csv.evaluate(submitted, reference, field_long_side_m=10)
    assert result['total'] == pytest.approx(.05)
    assert result['meta']['format_issues']
    Path(reference).write_text('t_tick,pred_x,pred_y,cur_x,cur_y,base_x,base_y\n'
                               '.5,nan,2,3,4,0,0\n')
    with pytest.raises(ValueError, match='非有限'):
        score_csv.evaluate(submitted, reference, field_long_side_m=10)


def test_check_csv_smoke(tmp_path):
    good = tmp_path / 'good.csv'
    good.write_text(''.join(f'1,2,1.4,2.1,{.02*k:.6f},0,.3,0,{1000+.1*k:.6f}\n' for k in range(12)))
    ok, lines = check_csv.check_csv(good, cols=9)
    assert ok, lines
    bad = tmp_path / 'bad.csv'
    bad.write_text('1,2,1.4,2.1,0,0,.001,0,1000.000000\n'
                   'nan,2,1.4,2.1,0,0,.3,0,1000.100000\n')
    ok, lines = check_csv.check_csv(bad, cols=9)
    assert not ok
    assert any('停车陷阱' in line and '❌' in line for line in lines)
    assert any('nan' in line and '❌' in line for line in lines)


@pytest.mark.parametrize('rows,expected', [
    ('1,2,3,4,0,0,.3,0\n', '九列'),
    ('1,2,3,4,0,0,.3,0,1000.1\n', '六位小数'),
    ('1,2,3,4,0,0,.3,0,1e3\n', '六位小数'),
    ('1,2,3,4,0,0,.3,0,1.000000\n1,2,3,4,0,0,.3,0,1.000000\n', '严格递增'),
    ('1,2,3,4,0,0,.3,0,2.000000\n1,2,3,4,0,0,.3,0,1.000000\n', '严格递增'),
    ('inf,2,3,4,0,0,.3,0,1.000000\n', '非有限'),
    ('', '没有可对齐'),
])
def test_check_csv_format_failures(tmp_path, rows, expected):
    path = tmp_path / 'test.csv'; path.write_text(rows)
    ok, lines = check_csv.check_csv(path)
    assert not ok
    assert any(expected in line for line in lines)


def test_check_csv_norm_boundary_and_near_target(tmp_path):
    path = tmp_path / 'test.csv'
    path.write_text('1,2,0.49,0,0,0,0,0,1.000000\n'
                    '1,2,3,0,0,0,.019,.019,1.100000\n'
                    '1,2,3,0,0,0,.02,0,1.200000\n')
    assert check_csv.check_csv(path)[0]
    path.write_text('1,2,0.5,0,0,0,0,0,1.000000\n')
    assert not check_csv.check_csv(path)[0]


def test_protected_directory_is_rejected_before_reading():
    with pytest.raises(ValueError, match='保护目录'):
        check_csv._safe_path('/home/example/ws_system/synthetic.csv')
    with pytest.raises(ValueError, match='保护目录'):
        check_csv._safe_path('/home/example/juger_runs/synthetic.csv')
