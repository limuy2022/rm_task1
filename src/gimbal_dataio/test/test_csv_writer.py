"""csv_writer / run_meta 单测（不依赖 rclpy）。

覆盖 DESIGN §7-T04 / §9.3-T-C5 验收点：
- 主文件 `np.loadtxt(path, delimiter=',')` 形状必须是 **(N, 8)**（默认严格 8 列）
- `csv_extra_cols=true` 时为 **(N, 10)**，且前 8 列语义不变
- 带表头副本首行是表头、列数与主文件一致
- NaN / inf 回退到上一次有效值，且 `track_status` 置 0
- 每 1s flush（未 close 也能读到内容）
- run_meta.json 含参数快照与产物清单
"""

from __future__ import annotations

import json
import math
import os
import time

import numpy as np
import pytest

from gimbal_dataio.csv_writer import COLS8, COLS_EXTRA, CsvWriter
from gimbal_dataio.run_meta import RunMeta


def _rows(n: int, start: int = 0):
    """生成 n 行测试数据（前 8 列为 0..7 的偏移，后 2 列为 ts/status）。"""
    out = []
    for k in range(n):
        i = start + k
        out.append([1.0 + i, 2.0 + i, 3.0 + i, 4.0 + i,
                    5.0 + i, 6.0 + i, 0.1 * i, -0.2 * i,
                    0.05 * i, (i % 3)])
    return out


def _load(path: str) -> np.ndarray:
    """np.loadtxt 并保证返回二维 (N, C)。"""
    arr = np.loadtxt(path, delimiter=',')
    return np.atleast_2d(arr)


# ---------------------------------------------------------------------------
# 1. 严格 8 列（默认）
# ---------------------------------------------------------------------------
def test_default_is_exactly_8_columns(tmp_path):
    """默认 `csv_extra_cols=false`：主文件每行恰好 8 列，形状 (N, 8)。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False)
    assert w.open(p)
    for r in _rows(60):
        assert w.write_row(r)
    w.close()

    arr = _load(p)
    assert arr.shape == (60, 8)
    assert np.all(np.isfinite(arr))

    # 文本层面：每行 8 个逗号分隔字段，无表头
    lines = [ln for ln in open(p, encoding='utf-8').read().splitlines() if ln.strip()]
    assert len(lines) == 60
    for ln in lines:
        assert len(ln.split(',')) == 8
        for tok in ln.split(','):
            assert len(tok.split('.')[1]) == 6      # %.6f
    # 首行是数据不是表头
    assert float(lines[0].split(',')[0]) == pytest.approx(1.0)


def test_column_order(tmp_path):
    """列序必须是 pred_x, pred_y, cur_x, cur_y, base_x, base_y, cmd_vx, cmd_vy。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False)
    w.open(p)
    w.write_estimate(pred_xy=(1.0, 2.0), cur_xy=(3.0, 4.0),
                     base_xy=(5.0, 6.0), cmd_vxy=(7.0, 8.0))
    w.close()
    arr = _load(p)
    assert np.allclose(arr[0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
    assert COLS8 == ['pred_x', 'pred_y', 'cur_x', 'cur_y',
                     'base_x', 'base_y', 'cmd_vx', 'cmd_vy']


def test_header_copy_has_header_and_same_rows(tmp_path):
    """`result_with_header.csv`：首行表头，列数与主文件一致，数据行相同。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False)
    w.open(p)
    for r in _rows(5):
        w.write_row(r)
    w.close()

    hp = str(tmp_path / 'result_with_header.csv')
    assert os.path.isfile(hp)
    lines = open(hp, encoding='utf-8').read().splitlines()
    assert lines[0] == ','.join(COLS8)
    assert len(lines) == 6
    main = open(p, encoding='utf-8').read().splitlines()
    assert lines[1:] == main


# ---------------------------------------------------------------------------
# 2. 附加列（第 9/10 列，不挤占前 8 列）
# ---------------------------------------------------------------------------
def test_extra_cols_appends_9_and_10(tmp_path):
    """`csv_extra_cols=true`：形状 (N, 10)，前 8 列语义不变。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=True)
    w.open(p)
    for r in _rows(30):
        w.write_row(r)
    w.close()

    arr = _load(p)
    assert arr.shape == (30, 10)
    # 前 8 列与默认模式完全一致
    base = np.array([r[:8] for r in _rows(30)])
    assert np.allclose(arr[:, :8], base)
    # 第 9 列 timestamp、第 10 列 track_status ∈ {0,1,2}
    assert np.allclose(arr[:, 8], [r[8] for r in _rows(30)])
    assert set(np.unique(arr[:, 9]).tolist()) <= {0.0, 1.0, 2.0}

    hp = str(tmp_path / 'result_with_header.csv')
    head = open(hp, encoding='utf-8').read().splitlines()[0]
    assert head == ','.join(COLS8 + COLS_EXTRA)


def test_extra_cols_default_is_false():
    """默认构造必须是 8 列（主理人拍板 3）。"""
    w = CsvWriter()
    assert w.extra_cols is False
    assert w.stats()['n_cols'] == 8


# ---------------------------------------------------------------------------
# 3. 有限性守卫
# ---------------------------------------------------------------------------
def test_nan_falls_back_to_last_valid(tmp_path):
    """NaN / inf → 用上一次有效值替代，行数不变。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=True)
    w.open(p)
    w.write_row([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 0.0, 1])
    w.write_row([float('nan'), 2.5, float('inf'), 4.5, 5.5, 6.5, 7.5, 8.5, 0.05, 2])
    w.write_row([9.0, 9.5, 9.25, 4.75, 5.75, 6.75, 7.75, 8.75, 0.10, 1])
    w.close()

    arr = _load(p)
    assert arr.shape == (3, 10)
    # 非有限的第 1、3 列回退到上一次有效值（1.0 / 3.0），有限的照常更新
    assert arr[1, 0] == pytest.approx(1.0)
    assert arr[1, 2] == pytest.approx(3.0)
    assert arr[1, 1] == pytest.approx(2.5)
    assert arr[1, 3] == pytest.approx(4.5)
    # track_status 被置 0
    assert arr[1, 9] == pytest.approx(0.0)
    assert w.n_guarded == 1
    assert np.all(np.isfinite(arr))


def test_nan_in_8col_mode_still_guarded(tmp_path):
    """8 列模式下出现 NaN 也回退，且列数仍是 8。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False)
    w.open(p)
    w.write_row([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    w.write_row([float('nan')] * 8)
    w.close()
    arr = _load(p)
    assert arr.shape == (2, 8)
    assert np.allclose(arr[1], np.ones(8))
    assert w.n_guarded == 1


def test_missing_values_are_guarded(tmp_path):
    """传入不足 8 个值时，缺位也走守卫（回退上一次有效值）。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False)
    w.open(p)
    w.write_row([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
    w.write_row([1.5, 2.5])       # 只有 2 个值
    w.close()
    arr = _load(p)
    assert arr.shape == (2, 8)
    assert np.allclose(arr[1], [1.5, 2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
    assert w.n_guarded == 1


# ---------------------------------------------------------------------------
# 4. flush / close
# ---------------------------------------------------------------------------
def test_flush_without_close(tmp_path):
    """flush 后即使不 close，文件里也能读到已写内容。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False, flush_period_s=1e9)   # 关闭自动 flush
    w.open(p)
    w.write_row([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
    w.write_row([2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0])
    w.flush()
    txt = open(p, encoding='utf-8').read()
    assert txt.count('\n') == 2
    assert w.row_count == 2
    w.close()
    assert w.is_open is False
    assert w.row_count == 2


def test_auto_flush_by_period(tmp_path):
    """`flush_period_s` 到期自动 flush（无需显式调用）。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False, flush_period_s=0.05)
    w.open(p)
    w.write_row([1.0] * 8)
    time.sleep(0.08)
    w.write_row([2.0] * 8)
    assert open(p, encoding='utf-8').read().count('\n') == 2
    w.close()


def test_write_after_close_is_noop(tmp_path):
    """close 后写行返回 False，不抛异常。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter()
    w.open(p)
    w.write_row([1.0] * 8)
    w.close()
    assert w.write_row([2.0] * 8) is False
    assert _load(p).shape == (1, 8)


# ---------------------------------------------------------------------------
# 5. 30s / 20Hz 规模
# ---------------------------------------------------------------------------
def test_600_rows_shape(tmp_path):
    """30s @20Hz ≈ 600 行，形状 (600, 8)，全有限。"""
    p = str(tmp_path / 'result.csv')
    w = CsvWriter(extra_cols=False)
    w.open(p)
    for k in range(600):
        w.write_row([0.001 * k] * 8)
    w.close()
    arr = _load(p)
    assert arr.shape == (600, 8)
    assert np.all(np.isfinite(arr))
    assert w.stats()['rows'] == 600


# ---------------------------------------------------------------------------
# 6. run_meta
# ---------------------------------------------------------------------------
def test_run_meta_content(tmp_path):
    """run_meta.json 含 run_id / 参数快照 / 产物清单 / 版本。"""
    out = str(tmp_path / 'outputs')
    meta = RunMeta(run_id='latest', out_dir=out)
    meta.set_params({'csv.rate_hz': 20.0, 'csv.csv_extra_cols': False})
    meta.add_artifact('result.csv', os.path.join(out, 'latest', 'result.csv'))

    csv_path = os.path.join(out, 'latest', 'result.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w', encoding='utf-8') as f:
        f.write('1,2,3,4,5,6,7,8\n')

    path = meta.write()
    assert os.path.isfile(path)
    data = RunMeta.read(path)
    assert data is not None
    assert data['run_id'] == 'latest'
    assert data['params']['csv.rate_hz'] == 20.0
    assert data['artifacts'][0]['name'] == 'result.csv'
    assert data['versions']['python']
    assert data['duration_s'] >= 0.0
    # 可 JSON 序列化（含 numpy 标量也能落盘）
    json.dumps(data)


def test_run_meta_jsonable_numpy(tmp_path):
    """numpy 标量 / 数组参数也能正确落盘。"""
    meta = RunMeta(run_id='r1', out_dir=str(tmp_path))
    meta.set_params({'a': np.float64(1.5), 'b': np.array([1.0, 2.0]),
                     'c': [1, 2, 3], 'd': True, 'e': None})
    p = meta.write()
    data = RunMeta.read(p)
    assert data['params']['a'] == 1.5
    assert data['params']['b'] == [1.0, 2.0]
    assert data['params']['c'] == [1, 2, 3]
    assert data['params']['d'] is True
    assert data['params']['e'] is None
