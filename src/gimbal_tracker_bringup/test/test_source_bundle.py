"""源码包版本追溯回归：真实 Git 状态与实际归档内容保持一致。"""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import zipfile


ROOT = Path(__file__).resolve().parents[3]


def test_bundle_records_actual_revision_and_local_changes(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'source_bundle', ROOT / 'tools/make_visual_tracking_bundle.py')
    bundle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bundle)
    source = tmp_path / 'source'
    source.mkdir()
    monkeypatch.setattr(bundle, 'ROOT', source)
    monkeypatch.setattr(bundle, 'PACKAGES', {'src/example': 'src/47-example'})
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
        data = archive.read(prefix + 'src/47-example/example.py')
        assert data == changed.read_bytes()
        assert manifest['files']['src/47-example/example.py'] == hashlib.sha256(data).hexdigest()
