"""Compare calibration files without changing either file or selecting a winner.

Supports our scalar camera YAML and ROS camera_calibration ost YAML. Optional
corners JSON can be evaluated through the production PnpSolver. Truth requires
an independent measurement; low reprojection error alone cannot verify K.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/gimbal_perception'))


def read_calibration(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict):
        raise ValueError('calibration must be a YAML mapping')
    if 'camera' in data:
        cam = data['camera']
        width, height = int(cam['width']), int(cam['height'])
        k = np.array([[cam['fx'], 0, cam['cx']], [0, cam['fy'], cam['cy']], [0, 0, 1]], float)
        d = np.asarray(cam.get('distortion', [0]*5), float).reshape(-1)
    else:
        if data.get('distortion_model', 'plumb_bob') not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('this comparison does not accept a fisheye calibration as pinhole')
        width, height = int(data['image_width']), int(data['image_height'])
        k = np.asarray(data['camera_matrix']['data'], float).reshape(3, 3)
        d = np.asarray(data['distortion_coefficients']['data'], float).reshape(-1)
    if (width <= 0 or height <= 0 or not np.all(np.isfinite(k)) or
            k[0,0] <= 0 or k[1,1] <= 0 or not np.all(np.isfinite(d)) or
            d.size not in (4,5,8,12,14)):
        raise ValueError('invalid calibration values')
    if not np.allclose(k[2], [0,0,1]) or abs(k[0,1]) > 1e-12 or abs(k[1,0]) > 1e-12:
        raise ValueError('unsupported nonstandard intrinsic matrix')
    cam = {'width': width, 'height': height, 'fx': float(k[0,0]), 'fy': float(k[1,1]),
           'cx': float(k[0,2]), 'cy': float(k[1,2]), 'distortion': d.tolist()}
    return {'path': str(path), 'camera': cam,
            'nominal_centered_fov_deg_ignores_distortion':
                [math.degrees(2*math.atan(width/(2*k[0,0]))),
                 math.degrees(2*math.atan(height/(2*k[1,1])))],
            'raw_input_uses': 'camera_matrix K plus distortion D; not projection_matrix P',
            'projection_matrix_present': 'projection_matrix' in data}


def compare(paths: list[Path], corners=None, marker_size=.10) -> dict:
    cameras = [read_calibration(p) for p in paths]
    if len(cameras) != 2:
        raise ValueError('exactly two calibrations required')
    a, b = [item['camera'] for item in cameras]
    result = {'calibrations': cameras,
              'same_resolution': (a['width'],a['height']) == (b['width'],b['height']),
              'first_over_second_focal_ratio': [a['fx']/b['fx'],a['fy']/b['fy']],
              'selected_calibration': None,
              'limitation': 'Files alone cannot establish the current camera/lens or measurement accuracy.'}
    if corners is not None:
        from gimbal_perception.pnp_solver import PnpSolver
        for item in cameras:
            r = PnpSolver.from_camera_config(item['camera'], marker_size_m=marker_size).solve(corners)
            item['pnp'] = {'valid': bool(r.ok), 'reason': r.reason,
                           'xyz_m': r.tvec_opt.tolist(),
                           'reprojection_error_px': float(r.reproj_err_px) if math.isfinite(r.reproj_err_px) else None}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('first', type=Path)
    parser.add_argument('second', type=Path)
    parser.add_argument('--corners-json', type=Path)
    parser.add_argument('--marker-size', type=float, default=.10)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    corners = json.loads(args.corners_json.read_text()) if args.corners_json else None
    if isinstance(corners, dict):
        corners = corners['corners']
    report = compare([args.first,args.second],corners,args.marker_size)
    text = json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text)
    print(text,end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
