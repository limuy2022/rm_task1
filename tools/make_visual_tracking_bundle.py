"""Create the small source-only NUC workspace; never starts ROS or hardware."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]
NAME = '47-visual-tracking-20261018-r3'
PACKAGES = {**{f'src/{p}': f'src/47-{p}' for p in (
    'gimbal_common', 'gimbal_interfaces', 'gimbal_perception',
    'gimbal_control', 'gimbal_tracker_bringup', 'gimbal_judge_io', 'grid_mapper',
    'gimbal_localization', 'gimbal_estimation', 'chassis_advisor')},
    'src/42-gimbal_serial': 'src/47-gimbal_serial'}


def build(output):
    files = {'COLCON_IGNORE': b'',
             'README.md': ('# 47队 NUC 源码包\n\n'
                           '包含本队独立工作区的十一个包，需在 NUC 编译；不含 Point-LIO 或传感器驱动。\n'
                           '按 docs/教程/云台跟踪.md 传输、编译、启动和停止；调参另见 docs/教程/上机调参检查清单.md。\n'
                           '默认运动关闭。本包不修改系统环境，不自动安装依赖。\n').encode()}
    for name in ('build_here.bash', 'use_here.bash', 'run_here.bash'):
        files[name] = (ROOT/'tools/visual_tracking'/name).read_bytes()
    files['protocol/PROTOCOL.md'] = (ROOT/'protocol/PROTOCOL.md').read_bytes()
    def source_link(match):
        label, package, path = match.groups()
        if f'src/{package}' in PACKAGES:
            target = PACKAGES[f'src/{package}'] + f'/{path}'
        else:
            target = f'https://github.com/limuy2022/rm_task1/blob/main/src/{package}/{path}'
        if not target.startswith('https:'):
            target = '../../' + target
        return f'[{label}]({target})'
    for folder in ('教程', '原理', '记录', '记录模板', '验证'):
        for path in sorted((ROOT/'docs'/folder).rglob('*')):
            if not path.is_file(): continue
            data = path.read_bytes()
            if path.suffix == '.md':
                doc = re.sub(r'\[([^\]]+)\]\(../../src/([^/]+)/([^)]+)\)', source_link, data.decode())
                doc = doc.replace('(../../gimbal-edu-tools/', '(https://github.com/limuy2022/rm_task1/blob/main/gimbal-edu-tools/')
                data = doc.encode()
            files[path.relative_to(ROOT).as_posix()] = data
    for name in ('submission_test_source.py', 'observe_submission.py',
                 'benchmark_submission_realtime.py', 'benchmark_prediction.py',
                 'run_camera_calibration.py', 'watch_gimbal_axes.py'):
        files['tools/'+name] = (ROOT/'tools'/name).read_bytes()
    for source, dest in PACKAGES.items():
        for path in sorted((ROOT/source).rglob('*')):
            rel = path.relative_to(ROOT/source)
            if any(part in {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log'} for part in rel.parts):
                continue
            if path.suffix in ('.pyc', '.pyo'): continue
            if path.is_symlink(): raise ValueError('Do not package symbolic link: '+str(path))
            if path.is_file(): files[f'{dest}/{rel.as_posix()}'] = path.read_bytes()
    manifest = {'workspace': NAME, 'source_base': 'e52edf73f00f0154e3b8d8f1828850be8c0f23df',
                'files': {name: hashlib.sha256(data).hexdigest() for name, data in sorted(files.items())}}
    files['manifest.json'] = (json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode()
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(files.items()): archive.writestr(NAME+'/'+name, data)
    with zipfile.ZipFile(output) as archive:
        assert archive.testzip() is None
        for name, digest in manifest['files'].items():
            assert hashlib.sha256(archive.read(NAME+'/'+name)).hexdigest() == digest
    result = {'zip': str(output.resolve()), 'files': len(files), 'bytes': output.stat().st_size,
              'sha256': hashlib.sha256(output.read_bytes()).hexdigest()}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    build(parser.parse_args().output)
