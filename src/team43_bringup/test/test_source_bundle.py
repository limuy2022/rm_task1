"""源码包版本追溯回归：真实 Git 状态与实际归档内容保持一致。"""
import hashlib
import importlib.util
import json
from pathlib import Path
import shlex
import subprocess
import zipfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]


def test_bundle_records_actual_revision_and_local_changes(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'source_bundle', ROOT / 'tools/make_visual_tracking_bundle.py')
    bundle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bundle)
    source = tmp_path / 'source'
    source.mkdir()
    monkeypatch.setattr(bundle, 'ROOT', source)
    monkeypatch.setattr(bundle, 'PACKAGES', {'src/example': 'src/team43_example'})
    names = ['protocol/PROTOCOL.md', 'src/example/example.py']
    names += ['tools/visual_tracking/' + name for name in
              ('build_here.bash', 'use_here.bash', 'run_here.bash')]
    names += ['tools/' + name for name in
              ('submission_test_source.py', 'observe_submission.py',
               'benchmark_submission_realtime.py', 'benchmark_prediction.py',
               'run_camera_calibration.py', 'watch_gimbal_axes.py')]
    for name in names:
        path = source / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# 测试源码\n')
    assert bundle.source_version() == {'source_base': None, 'source_dirty': None}

    def git(*args):
        return subprocess.run(['git', '-C', str(source), *args], check=True,
                              capture_output=True, text=True).stdout.strip()

    git('init')
    git('add', '.')
    git('-c', 'user.name=源码包测试', '-c', 'user.email=bundle@example.invalid',
        '-c', 'commit.gpgsign=false', 'commit', '-m', 'test: 初始化源码包样例')
    commit = git('rev-parse', 'HEAD')
    assert bundle.source_version() == {'source_base': commit, 'source_dirty': False}
    changed = source / 'src/example/example.py'
    changed.write_text('# 尚未提交的新内容\n')
    (source / 'local_notes.txt').write_text('未跟踪文件也应标记工作区已修改\n')
    output = tmp_path / 'bundle.zip'
    bundle.build(output)
    with zipfile.ZipFile(output) as archive:
        prefix = bundle.NAME + '/'
        manifest = json.loads(archive.read(prefix + 'manifest.json'))
        assert manifest['source_base'] == commit
        assert manifest['source_dirty'] is True
        data = archive.read(prefix + 'src/team43_example/example.py')
        assert data == changed.read_bytes()
        assert manifest['files']['src/team43_example/example.py'] == hashlib.sha256(data).hexdigest()


def test_bundle_contains_complete_lio_source_and_runtime_files(tmp_path):
    """交付包离线包含定位算法、许可证及运行配置，无需额外安装器或算法归档。"""
    spec = importlib.util.spec_from_file_location(
        'source_bundle_lio', ROOT / 'tools/make_visual_tracking_bundle.py')
    bundle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bundle)
    package = 'src/team43_lio_backend'
    assert bundle.NAME == '43-visual-tracking'
    for source, destination in bundle.PACKAGES.items():
        name = ET.parse(ROOT / source / 'package.xml').findtext('name')
        assert source == destination == 'src/' + name
    script = (ROOT / 'tools/visual_tracking/build_here.bash').read_text()
    selection = script.split('--packages-select ', 1)[1].split('--build-base', 1)[0]
    selected = set(shlex.split(selection.replace('\\\n', ' ')))
    assert selected == {Path(source).name for source in bundle.PACKAGES}
    assert package in bundle.PACKAGES
    destination = bundle.PACKAGES[package]
    output = tmp_path / 'with-lio.zip'
    bundle.build(output)
    with zipfile.ZipFile(output) as archive:
        prefix = bundle.NAME + '/'
        names = {name.removeprefix(prefix) for name in archive.namelist()}
        manifest = json.loads(archive.read(prefix + 'manifest.json'))
        required = {
            'package.xml', 'CMakeLists.txt', 'backend.cpp', 'upstream.json',
            'include/sensor_input_audit.hpp', 'launch/lio.launch.py',
            'config/small_point_lio_mid360.yaml', 'vendor/COLCON_IGNORE',
            'vendor/small_point_lio/LICENSE.txt',
            'vendor/small_point_lio/src/small_point_lio/small_point_lio.cpp',
            'vendor/small_point_lio/src/small_point_lio/estimator.cpp',
            'vendor/small_point_lio/src/util/voxelgrid_sampling.cpp',
        }
        for relative in required:
            name = destination + '/' + relative
            assert name in names
            data = archive.read(prefix + name)
            assert data == (ROOT / package / relative).read_bytes()
            assert manifest['files'][name] == hashlib.sha256(data).hexdigest()
        # 上游源码全部随包，不能只包含几个能在开发机现有目录中编译的文件。
        vendor = ROOT / package / 'vendor/small_point_lio'
        for source in vendor.rglob('*'):
            if source.is_file():
                relative = source.relative_to(ROOT / package).as_posix()
                assert archive.read(prefix + destination + '/' + relative) == source.read_bytes()
        upstream = json.loads(archive.read(prefix + destination + '/upstream.json'))
        assert upstream['vendored_source'] == 'vendor/small_point_lio'
        assert upstream['algorithm_source_modified'] is False
        assert not any(name.startswith('tools/lio/') for name in names)
        assert not any(name.startswith('archives/') for name in names)
