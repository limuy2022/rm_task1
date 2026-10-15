"""tick_csv 单测：9 列、追加、t_tick 有限性守卫、nan 兜底。"""

from __future__ import annotations

import math
import os

from gimbal_judge_io.tick_csv import COLS9, N_COLS, TickCsvWriter, format_row


def test_cols():
    assert N_COLS == 9
    assert COLS9 == ['pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y',
                     'vx', 'vy', 't_tick']


def test_format_row_nine_cols():
    line = format_row((1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (0.5, -0.25),
                      1760120000.123456)
    parts = line.split(',')
    assert len(parts) == 9
    assert parts[8] == '1760120000.123456'
    assert parts[0] == '1.000000' and parts[7] == '-0.250000'


def test_nonfinite_becomes_nan():
    line = format_row((float('nan'), 2.0), (3.0, 4.0), (5.0, 6.0), (0.0, 0.0), 1.0)
    assert line.split(',')[0] == 'nan'


def test_writer_append_and_flush(tmp_path):
    p = str(tmp_path / 'T7.csv')
    w = TickCsvWriter(p)
    assert w.open()
    assert w.write_tick((1, 2), (3, 4), (5, 6), (0.1, 0.2), 100.5)
    assert w.write_tick((1, 2), (3, 4), (5, 6), (0.1, 0.2), 100.6)
    # 不 close 也要能看到（flush 生效）
    with open(p, 'r', encoding='utf-8') as f:
        lines = f.read().strip().split('\n')
    assert len(lines) == 2
    assert len(lines[0].split(',')) == 9
    w.close()
    assert w.rows == 2

    # 追加模式：重启后不覆盖旧行
    w2 = TickCsvWriter(p)
    w2.open()
    w2.write_tick((0, 0), (0, 0), (0, 0), (0, 0), 200.0)
    w2.close()
    with open(p, 'r', encoding='utf-8') as f:
        lines = f.read().strip().split('\n')
    assert len(lines) == 3
    assert lines[0].split(',')[8] == '100.500000'


def test_eight_col_mode(tmp_path):
    """judge.emit_t_tick=false → 只写 8 列数据（口述形态）。"""
    from gimbal_judge_io.tick_csv import COLS8, format_row, header_line
    assert len(COLS8) == 8 and COLS8[-1] == 'vy'
    line8 = format_row((1, 2), (3, 4), (5, 6), (0.1, 0.2), 100.5,
                       with_tick=False)
    assert len(line8.split(',')) == 8
    assert header_line(False) == ','.join(COLS8)
    p = str(tmp_path / 'T8col.csv')
    w = TickCsvWriter(p, with_tick=False)
    assert w.n_cols == 8
    w.open()
    assert w.write_tick((1, 2), (3, 4), (5, 6), (0.1, 0.2), 100.5)
    w.close()
    with open(p, 'r', encoding='utf-8') as f:
        assert len(f.read().strip().split(',')) == 8


def test_bad_tick_dropped(tmp_path):
    p = str(tmp_path / 'T8.csv')
    w = TickCsvWriter(p)
    w.open()
    assert w.write_tick((0, 0), (0, 0), (0, 0), (0, 0), float('nan')) is False
    assert w.rows == 0
    assert w.n_dropped == 1
    w.close()


def test_guard_uses_last_valid(tmp_path):
    p = str(tmp_path / 'T9.csv')
    w = TickCsvWriter(p)
    w.open()
    w.write_tick((1.0, 1.0), (2.0, 2.0), (0.0, 0.0), (0.5, 0.0), 1.0)
    w.write_tick((float('nan'), float('nan')), (3.0, 3.0), (0.1, 0.0),
                 (0.5, 0.0), 1.1)
    w.close()
    with open(p, 'r', encoding='utf-8') as f:
        lines = f.read().strip().split('\n')
    r2 = [float(x) for x in lines[1].split(',')]
    assert r2[0] == 1.0 and r2[1] == 1.0      # pred 用了上一次有效值
    assert r2[2] == 3.0                        # cur 用本次
    assert w.n_guarded == 1


def test_write_failure_not_counted_as_success():
    """磁盘写入或flush失败时，外层必须收到失败，不能增加成功行数。"""
    class BrokenFile:
        def __init__(self, fail_at):
            self.fail_at = fail_at
        def write(self, text):
            if self.fail_at == 'write':
                raise OSError('simulated write failure')
            return len(text)
        def flush(self):
            if self.fail_at == 'flush':
                raise OSError('simulated flush failure')
    for fail_at in ('write', 'flush'):
        writer = TickCsvWriter('unused.csv')
        writer._fh = BrokenFile(fail_at)
        assert writer.write_tick((1,2), (3,4), (5,6), (.3,.4), 100.) is False
        assert writer.rows == 0
