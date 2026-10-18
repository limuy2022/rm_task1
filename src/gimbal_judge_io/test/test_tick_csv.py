"""tick_csv 单测：9 列、追加、t_tick 有限性守卫、nan 兜底。"""

from __future__ import annotations

import math
import os
import inspect
import subprocess
import sys

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


def test_restart_recovers_last_tick_and_guard_values(tmp_path):
    path = tmp_path / 'restart.csv'
    first = TickCsvWriter(str(path))
    assert first.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 100.5)
    first.close()
    original = path.read_bytes()
    second = TickCsvWriter(str(path))
    assert second.open()
    assert second.stats()['recovered_rows'] == 1
    assert not second.write_tick((9, 9), (9, 9), (9, 9), (9, 9), 100.5)
    assert not second.write_tick((9, 9), (9, 9), (9, 9), (9, 9), 100.4)
    assert second.rows == 0
    assert path.read_bytes() == original
    assert second.write_tick((float('nan'), float('nan')), (7, 8), (5, 6), (.1, .2), 100.6)
    second.close()
    assert path.read_bytes().startswith(original)
    last = list(map(float, path.read_text().splitlines()[-1].split(',')))
    assert last[:4] == [1., 2., 7., 8.]


def test_process_crash_releases_lock_and_recovers_flushed_tick(tmp_path):
    path = tmp_path / 'crash.csv'
    package_root = os.path.dirname(os.path.dirname(inspect.getfile(TickCsvWriter)))
    script = '''import os, sys
sys.path.insert(0, %r)
from gimbal_judge_io.tick_csv import TickCsvWriter
writer = TickCsvWriter(sys.argv[1])
assert writer.write_tick((1,2),(3,4),(5,6),(.1,.2),100.)
os._exit(23)  # Do not call close or Python finalizers: simulate process loss.
''' % package_root
    result = subprocess.run([sys.executable, '-B', '-c', script, str(path)], check=False)
    assert result.returncode == 23
    before = path.read_bytes()
    recovered = TickCsvWriter(str(path))
    assert recovered.open()
    assert recovered.stats()['recovered_rows'] == 1
    assert not recovered.write_tick((1,2), (3,4), (5,6), (.1,.2), 100.)
    assert path.read_bytes() == before
    assert recovered.write_tick((1,2), (3,4), (5,6), (.1,.2), 100.1)
    recovered.close()
    assert len(path.read_text().splitlines()) == 2


def test_deduplicate_by_serialized_six_decimals(tmp_path):
    path = tmp_path / 'microseconds.csv'
    writer = TickCsvWriter(str(path))
    assert writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 1.0000001)
    assert not writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 1.0000004)
    assert writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 1.000001)
    writer.close()
    assert [line.split(',')[-1] for line in path.read_text().splitlines()] == ['1.000000', '1.000001']


def test_corrupt_or_incomplete_existing_file_is_never_repaired(tmp_path):
    valid = (format_row((1, 2), (3, 4), (5, 6), (.1, .2), 100.) + '\n').encode()
    corrupt = [
        valid[:-1],                    # Even a parseable row needs its newline.
        valid + b'1,2,3',              # Half row must not receive another tick.
        b'not,a,csv\n',
        valid + valid,                 # Existing duplicate is corruption.
        valid + (format_row((1, 2), (3, 4), (5, 6), (.1, .2), 99.) + '\n').encode(),
        valid.replace(b'100.000000', b'100.0'),
        valid.replace(b'100.000000', b'nan'),
        valid.replace(b'1.000000', b'nan', 1),
        b'\xff\n',
        b'\n',
    ]
    for index, content in enumerate(corrupt):
        path = tmp_path / ('corrupt_%d.csv' % index)
        path.write_bytes(content)
        writer = TickCsvWriter(str(path))
        assert not writer.open(), index
        assert not writer.is_open
        assert not writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 101.)
        assert writer.rows == 0
        assert path.read_bytes() == content


def test_flush_failure_latches_and_restart_does_not_repeat_uncertain_row(tmp_path):
    path = tmp_path / 'flush_failure.csv'
    writer = TickCsvWriter(str(path))
    assert writer.open()

    class FlushFailure:
        def __init__(self, real):
            self.real = real
        def write(self, text):
            return self.real.write(text)
        def flush(self):
            raise OSError('simulated flush failure after buffered full write')
        def close(self):
            self.real.close()  # Closing may expose the uncertain full row.

    writer._fh = FlushFailure(writer._fh)
    assert not writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 100.)
    assert writer.rows == 0 and writer.stats()['failed']
    assert not writer.write_tick((9, 9), (9, 9), (9, 9), (9, 9), 101.)
    assert writer._last == [0.] * 8  # Unconfirmed write cannot poison guard state.
    writer.close()
    assert len(path.read_text().splitlines()) == 1
    before = path.read_bytes()
    recovered = TickCsvWriter(str(path))
    assert recovered.open()
    assert not recovered.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 100.)
    assert path.read_bytes() == before
    recovered.close()


def test_short_write_stops_and_preserves_partial_tail(tmp_path):
    path = tmp_path / 'partial_write.csv'
    writer = TickCsvWriter(str(path))
    assert writer.open()

    class ShortWrite:
        def __init__(self, real):
            self.real = real
        def write(self, text):
            return self.real.write(text[:10])
        def flush(self):
            self.real.flush()
        def close(self):
            self.real.close()

    writer._fh = ShortWrite(writer._fh)
    assert not writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 100.)
    assert not writer.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 101.)
    assert writer.rows == 0
    writer.close()
    before = path.read_bytes()
    assert len(before) == 10 and not before.endswith(b'\n')
    recovered = TickCsvWriter(str(path))
    assert not recovered.open()
    assert path.read_bytes() == before


def test_only_one_append_writer_can_hold_file(tmp_path):
    path = tmp_path / 'exclusive.csv'
    first, second = TickCsvWriter(str(path)), TickCsvWriter(str(path))
    assert first.open()
    assert not second.open()
    assert first.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 100.)
    before = path.read_bytes()
    first.close()
    assert second.open()
    assert not second.write_tick((1, 2), (3, 4), (5, 6), (.1, .2), 100.)
    assert path.read_bytes() == before
    second.close()


def test_nonempty_legacy_eight_columns_cannot_resume_without_timestamp(tmp_path):
    path = tmp_path / 'legacy.csv'
    path.write_text('1,2,3,4,5,6,.1,.2\n')
    before = path.read_bytes()
    recovered = TickCsvWriter(str(path))
    assert not recovered.open()
    assert path.read_bytes() == before
