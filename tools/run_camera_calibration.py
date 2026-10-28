"""Team-local entry for the installed ROS camera_calibration GUI.

Calibration/detection remains in camera_calibration. This process only redirects
SAVE to a new team file and blocks COMMIT/SetCameraInfo. No driver is started.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime
import importlib
import json
import math
from pathlib import Path
import sys
import tarfile
import uuid


def create_archive(calibrator, output_dir: Path) -> Path:
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    tag = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
    filename = output_dir / ('calibrationdata_' + tag + '.tar.gz')
    # Exclusive create: do not overwrite any calibration archive, including our own.
    with tarfile.open(filename, 'x:gz') as archive:
        calibrator.do_tarfile_save(archive)
    (output_dir / 'latest_saved.txt').write_text(str(filename.resolve()) + '\n')
    print(f'本队SAVE完成：{filename.resolve()}', flush=True)
    print('已保存采集图像、ost.yaml和ost.txt；没有写入相机驱动。', flush=True)
    return filename


@contextmanager
def local_gui_actions(calibration_module, gui_module, output_dir):
    base = calibration_module.Calibrator
    gui_base = gui_module.CalibrationNode
    old_save, old_upload = base.do_save, gui_base.do_upload

    def save(self):
        create_archive(self, output_dir)

    def disabled_upload(self):
        print('本队入口已禁用COMMIT：请点击SAVE保存到本队目录，不写相机配置。', flush=True)
        return False

    base.do_save, gui_base.do_upload = save, disabled_upload
    try:
        yield
    finally:
        base.do_save, gui_base.do_upload = old_save, old_upload


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--cols', type=int, required=True, help='横向内角点数，不是方格数')
    parser.add_argument('--rows', type=int, required=True, help='纵向内角点数，不是方格数')
    parser.add_argument('--square-mm', type=float, required=True, help='单格实测边长，毫米')
    args = parser.parse_args(argv)
    if not (args.cols >= 3 and args.rows >= 3):
        parser.error('内角点每个方向至少3个；请重新数交叉点，不要填外侧格线数。')
    if not (math.isfinite(args.square_mm) and 0 < args.square_mm <= 1000):
        parser.error('单格边长须为0到1000之间的毫米数；例如25毫米填25。')
    args.output_dir = args.output_dir.expanduser().resolve()
    calibration = importlib.import_module('camera_calibration.calibrator')
    gui = importlib.import_module('camera_calibration.camera_calibrator')
    cli = importlib.import_module('camera_calibration.nodes.cameracalibrator')
    ros_args = ['cameracalibrator', '--no-service-check', '--size',
                f'{args.cols}x{args.rows}', '--square', format(args.square_mm / 1000, '.12g'),
                '--camera_name', 'team43_hik', '--ros-args',
                '-r', 'image:=/image_raw', '-r', '__node:=team43_cameracalibrator',
                '-r', '__ns:=/team43_calibration']
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'session.json').write_text(json.dumps({
        'inner_corners': [args.cols, args.rows], 'square_mm': args.square_mm,
        'square_m': args.square_mm / 1000, 'image_topic': '/image_raw',
        'commit_disabled': True, 'save_dir': str(args.output_dir),
        'calibrator_module': str(calibration.__file__),
    }, ensure_ascii=False, indent=2) + '\n')
    print(f'内角点：{args.cols}×{args.rows}；单格：{args.square_mm:g}mm；只读/image_raw。', flush=True)
    print(f'SAVE目录：{args.output_dir}；COMMIT已禁用。', flush=True)
    previous = sys.argv
    try:
        sys.argv = ros_args
        with local_gui_actions(calibration, gui, args.output_dir):
            cli.main()
    except KeyboardInterrupt:  # 用户关闭标定；上下文管理器仍恢复本进程的GUI方法。
        pass
    finally:
        sys.argv = previous
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
