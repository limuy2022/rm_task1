#!/usr/bin/env python3
"""容器中估计每张地图的两侧长墙宽度；输出证据，不直接覆盖比赛配置。"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from team43_grid_mapper.saved_map import SavedMap, rotation
from team43_grid_mapper.corridor import fit_parallel_walls


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('maps',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    rows = []
    for path in args.maps:
        saved = SavedMap(path)
        cells = np.argwhere(saved.grid == 100)
        points = (cells+.5)*saved.resolution @ rotation(saved.origin[2]).T+saved.origin[:2]
        fit = fit_parallel_walls(points)
        meta = yaml.safe_load(path.read_text())
        files = [path,path.parent/meta['image']]
        rows.append({'map_yaml':str(path), 'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},**fit})
    report = {'fits':rows,'estimated_width_m':float(np.median([r['width_m'] for r in rows])),
              'width_spread_m':float(np.ptp([r['width_m'] for r in rows])),
              'limitation':'只估计侧墙宽度；不确定门、端点、纵向位置；不修改LIO'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
