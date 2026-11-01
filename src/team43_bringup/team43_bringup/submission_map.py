"""选定地图原样交付；仅修正图片引用，不做坐标重定位。"""
import hashlib
import math
from pathlib import Path

import yaml
from team43_grid_mapper.pgm_io import read_pgm, _publish_files


def copy_submission_map(source_yaml, output_dir):
    source = Path(source_yaml).expanduser().resolve()
    meta = yaml.safe_load(source.read_text())
    if not isinstance(meta,dict) or not isinstance(meta.get('image'),str):
        raise ValueError('地图YAML缺少图片引用')
    image = Path(meta['image'])
    if image.name != meta['image']:
        raise ValueError('地图图片必须与YAML位于同一目录')
    pgm = source.parent/image
    read_pgm(str(pgm))
    if not math.isfinite(float(meta['resolution'])) or float(meta['resolution']) <= 0:
        raise ValueError('地图分辨率无效')
    origin = meta['origin']
    if len(origin)!=3 or not all(math.isfinite(float(v)) for v in origin):
        raise ValueError('地图origin无效')
    meta['image'] = '43_map.pgm'
    out = Path(output_dir).expanduser().resolve()
    pixels = pgm.read_bytes()
    # 复用既有两文件暂存和失败回滚；不重新栅格化PGM，不改origin/frame。
    _publish_files({str(out/'43_map.pgm'):pixels,
                    str(out/'43_map.yaml'):yaml.safe_dump(meta,allow_unicode=True,sort_keys=False).encode()})
    return {'source_yaml':str(source),'source_pgm':str(pgm),
            'output_yaml':str(out/'43_map.yaml'),'output_pgm':str(out/'43_map.pgm'),
            'pgm_sha256':hashlib.sha256(pixels).hexdigest(),
            'origin':origin,'coordinate_transform_applied':False}
