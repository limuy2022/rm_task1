"""离线核验真实写出器与地图序列化；需要numpy。在 ROS 2 Humble 容器内运行，
但不启动 ROS 节点、不做 DDS 实机通信。"""
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[3]
for package in ('gimbal_common', 'gimbal_judge_io', 'grid_mapper'):
    sys.path.insert(0, str(root / 'src' / package))
from gimbal_judge_io.tick_csv import TickCsvWriter
from grid_mapper.pgm_io import write_pgm, read_grid, read_pgm
import numpy as np

def test_1800_rows_visible_before_close(tmp_path):
    directory = tmp_path
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


def test_asymmetric_map_pixels(tmp_path):
    directory = tmp_path
    grid = np.array([[100,0,0,-1],[0,100,0,0],[0,0,100,0]], dtype=np.int8).T
    pgm = directory / '47_map.pgm'
    write_pgm(str(pgm), grid)
    assert np.array_equal(read_grid(str(pgm)).grid, grid)
    image, _ = read_pgm(str(pgm))
    # 独立预期像素，避免仅通过互逆读写掩盖共同翻转错误。
    assert image.tolist() == [[254,254,0,254],[254,0,254,254],[0,254,254,205]]
