"""离线核验真实写出器与地图序列化；需要numpy，不需要ROS。"""
import ast
import csv
import importlib.util
import inspect
import sys
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parents[1]
for package in ('gimbal_judge_io', 'grid_mapper'):
    sys.path.insert(0, str(root / 'src' / package))
from gimbal_judge_io.tick_csv import TickCsvWriter
from grid_mapper.pgm_io import write_pgm, read_grid, read_pgm
import numpy as np

spec = importlib.util.spec_from_file_location('tick_tests', root / 'src/gimbal_judge_io/test/test_tick_csv.py')
tests = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tests)
with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    for name, fn in inspect.getmembers(tests, inspect.isfunction):
        if name.startswith('test_'):
            p = directory / name
            p.mkdir()
            fn(p) if inspect.signature(fn).parameters else fn()
            print('PASS', name)
    expected = []
    for k in range(1800):
        dt = k / 10
        expected.append(','.join(f'{v:.6f}' for v in
            (1+.4*(dt+.1), 2-.2*(dt+.1), 1+.4*dt, 2-.2*dt,
             .5, -.25, .3, .4, 1792000000+dt)))
    p = directory / '47.csv'
    writer = TickCsvWriter(str(p))
    assert writer.open()
    for line in expected:
        v = list(map(float, line.split(',')))
        assert writer.write_tick(v[:2], v[2:4], v[4:6], v[6:8], v[8])
    # 不关闭写出器，独立文件句柄也必须可读全部行。
    assert p.read_text().splitlines() == expected
    writer.close()
    print('PASS 1800 rows: actual writer, order, timestamp, visibility before close')
    grid = np.array([[100,0,0,-1],[0,100,0,0],[0,0,100,0]], dtype=np.int8).T
    pgm = directory / '47_map.pgm'
    write_pgm(str(pgm), grid)
    assert np.array_equal(read_grid(str(pgm)).grid, grid)
    image, _ = read_pgm(str(pgm))
    # 独立预期像素，避免仅通过互逆读写掩盖共同翻转错误。
    assert image.tolist() == [[254,254,0,254],[254,0,254,254],[0,254,254,205]]
    print('PASS asymmetric PGM pixels and occupancy roundtrip')
for path in root.glob('src/**/*.py'):
    ast.parse(path.read_bytes(), filename=str(path))
print('PASS Python syntax')
print('NOT VERIFIED: ROS communication, Humble launch, NUC timing, physical accuracy')
