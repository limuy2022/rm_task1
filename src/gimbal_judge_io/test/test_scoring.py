"""scoring 单测：口述权重、方向角误差、停车陷阱、总分与报告。"""

from __future__ import annotations

import math

from gimbal_judge_io import scoring as S


def test_weights():
    assert S.W_HEAD4 == (0.10, 0.20, 0.25, 0.25)
    assert S.W_DIR == 0.20
    assert abs(sum(S.W_HEAD4) + S.W_DIR - 1.0) < 1e-12
    assert S.WEIGHTS['pred_x'] == 0.10 and S.WEIGHTS['cur_y'] == 0.25


def test_heading_and_angle():
    assert abs(S.heading_deg(1.0, 0.0) - 0.0) < 1e-9
    assert abs(S.heading_deg(0.0, 1.0) - 90.0) < 1e-9
    assert abs(S.heading_deg(0.0, -1.0) + 90.0) < 1e-9
    assert math.isnan(S.heading_deg(0.0, 0.0))
    # 跨 ±180 的夹角
    assert abs(S.angle_between_deg(-170.0, 170.0) - 20.0) < 1e-9
    assert abs(S.angle_between_deg(350.0, 10.0) - 20.0) < 1e-9
    assert abs(S.angle_between_deg(0.0, 180.0) - 180.0) < 1e-9


def test_direction_error_normal():
    # 建议速度朝向 90°，标准方向 0° → 90°
    e = S.direction_error_deg(0.0, 0.3, 0.0, dist_to_target_m=3.0)
    assert abs(e - 90.0) < 1e-9
    # 已到位且停车 → 0
    e = S.direction_error_deg(0.0, 0.0, 123.0, dist_to_target_m=0.2)
    assert e == 0.0


def test_direction_error_stop_trap():
    # 停车 + 远离目标 → 满分误差 180°
    assert S.direction_error_deg(0.019, 0.0, 0.0, 2.0) == 180.0
    # 停车 + 已到位 → 0
    assert S.direction_error_deg(0.0, 0.0, 0.0, 0.49) == 0.0
    # 恰好 0.02 不算停车（阈值是严格小于）
    assert abs(S.direction_error_deg(0.02, 0.0, 180.0, 2.0) - 180.0) < 1e-9
    # 距离无效（NaN）时按满分误差处理
    assert S.direction_error_deg(0.0, 0.0, 0.0, float('nan')) == 180.0


def test_direction_term_values():
    d = S.direction_term([18.0] * 5)
    assert abs(d['rmse_deg'] - 18.0) < 1e-9
    assert abs(d['norm_rmse'] - 0.1) < 1e-12
    assert abs(d['contribution'] - 0.02) < 1e-12
    # 1° RMSE 的总分价值
    d1 = S.direction_term([1.0])
    assert abs(d1['contribution'] - 0.2 / 180.0) < 1e-15
    # 空输入
    assert math.isnan(S.direction_term([])['contribution'])


def test_score_zero_when_perfect():
    n = 10
    pred = [(1.0, 2.0)] * n
    cur = [(1.5, 2.5)] * n
    res = S.score(pred, cur, pred, cur, [0.0] * n)
    assert abs(res['total']) < 1e-12
    assert abs(res['weight_sum'] - 1.0) < 1e-12
    assert res['degraded'] == []


def test_score_contributions():
    # pred_x 恒定差 0.1 → RMSE 0.1 → 0.1*0.10 = 0.01
    # pred_y 恒定差 0.1 → 0.1*0.20 = 0.02
    # cur_y  恒定差 0.2 → 0.2*0.25 = 0.05
    pred = [(0.1, 0.1), (0.1, 0.1)]
    cur = [(0.0, 0.2), (0.0, 0.2)]
    pred_true = [(0.0, 0.0), (0.0, 0.0)]
    cur_true = [(0.0, 0.0), (0.0, 0.0)]
    res = S.score(pred, cur, pred_true, cur_true, [0.0, 0.0])
    assert abs(res['terms']['pred_x']['contribution'] - 0.010) < 1e-12
    assert abs(res['terms']['pred_y']['contribution'] - 0.020) < 1e-12
    assert abs(res['terms']['cur_y']['contribution'] - 0.050) < 1e-12
    assert abs(res['total'] - 0.080) < 1e-12


def test_score_direction_dominates_at_full_error():
    # 全部停车且远离目标 → 方向项 0.2*1.0 = 0.2
    errs = [S.direction_error_deg(0.0, 0.0, 0.0, 3.0)] * 4
    res = S.score([(0, 0)] * 4, [(0, 0)] * 4, [(0, 0)] * 4, [(0, 0)] * 4, errs)
    assert abs(res['dir']['contribution'] - 0.20) < 1e-12
    assert abs(res['total'] - 0.20) < 1e-12


def test_score_degraded_on_missing():
    res = S.score([], [], [], [], [])
    assert 'pred_x' in res['degraded'] and 'direction' in res['degraded']
    assert res['total'] == 0.0


def test_report_renders():
    res = S.score([(0.1, 0.0)], [(0.0, 0.2)], [(0.0, 0.0)], [(0.0, 0.0)], [0.0])
    txt = S.format_report(res, meta={'队伍': 'T7'})
    assert 'pred_x' in txt and 'direction' in txt and '合计' in txt
    assert 'T7' in txt


def test_score_csv_end_to_end(tmp_path):
    """自评工具：本队 CSV vs 真值 CSV，按 t_tick 对齐复算总分。"""
    import os
    import sys
    tools = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         '..', '..', '..', 'tools'))
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import score_csv

    truth = str(tmp_path / 'truth.csv')
    pred = str(tmp_path / 'T7.csv')
    with open(truth, 'w', encoding='utf-8') as f:
        f.write('t_tick,pred_x,pred_y,cur_x,cur_y,base_x,base_y,std_dir_deg\n')
        for k in range(4):
            f.write('%d.5,1.0,2.0,3.0,4.0,0.0,0.0,0.0\n' % k)
    with open(pred, 'w', encoding='utf-8') as f:
        for k in range(4):
            # pred_x +0.1、cur_y +0.2；速度朝向 = 标准方向 0°（vx>0）
            f.write('1.100000,2.000000,3.000000,4.200000,0.000000,0.000000,'
                    '0.300000,0.000000,%d.500000\n' % k)
    res = score_csv.evaluate(pred, truth, tol=0.05)
    assert res['meta']['matched'] == 4
    assert abs(res['terms']['pred_x']['rmse'] - 0.1) < 1e-9
    assert abs(res['terms']['cur_y']['rmse'] - 0.2) < 1e-9
    assert abs(res['dir']['rmse_deg']) < 1e-9
    assert abs(res['total'] - 0.06) < 1e-9


def test_check_csv_smoke(tmp_path):
    """体检脚本：正常产物 PASS，列数/nan/停车陷阱异常 FAIL。"""
    import os
    import sys
    tools = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         '..', '..', '..', 'tools'))
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import check_csv

    good = str(tmp_path / 'good.csv')
    with open(good, 'w', encoding='utf-8') as f:
        for k in range(12):
            f.write('1.0,2.0,1.4,2.1,%.6f,0.0,0.300000,0.0,%.6f\n'
                    % (0.02 * k, 1000.0 + 0.1 * k))
    ok, lines = check_csv.check_csv(good, cols=9)
    assert ok is True, lines

    bad = str(tmp_path / 'bad.csv')
    with open(bad, 'w', encoding='utf-8') as f:
        f.write('1.0,2.0,1.4,2.1,0.0,0.0,0.001,0.0,1000.0\n')   # 停车且远离
        f.write('nan,2.0,1.4,2.1,0.0,0.0,0.3,0.0,1000.1\n')      # nan
    ok, lines = check_csv.check_csv(bad, cols=9)
    assert ok is False
    assert any('停车陷阱' in x and '❌' in x for x in lines)
    assert any('nan' in x and '❌' in x for x in lines)
