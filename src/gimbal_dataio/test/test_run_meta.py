"""运行元数据必须可读回；不支持的数据和IO失败必须明确抛出。"""
import json
import os
import numpy as np
import pytest
from gimbal_dataio.run_meta import RunMeta


def test_run_meta_content(tmp_path):
    """run_meta.json 含 run_id / 参数快照 / 产物清单 / 版本。"""
    out = str(tmp_path / 'outputs')
    meta = RunMeta(run_id='latest', out_dir=out)
    meta.set_params({'csv.rate_hz': 20.0, 'csv.csv_extra_cols': False})
    meta.add_artifact('result.csv', os.path.join(out, 'latest', 'result.csv'))

    csv_path = os.path.join(out, 'latest', 'result.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w', encoding='utf-8') as f:
        f.write('1,2,3,4,5,6,7,8\n')

    path = meta.write()
    assert os.path.isfile(path)
    data = RunMeta.read(path)
    assert data is not None
    assert data['run_id'] == 'latest'
    assert data['params']['csv.rate_hz'] == 20.0
    assert data['artifacts'][0]['name'] == 'result.csv'
    assert data['versions']['python']
    assert data['duration_s'] >= 0.0
    # 可 JSON 序列化（含 numpy 标量也能落盘）
    json.dumps(data)


def test_run_meta_jsonable_numpy(tmp_path):
    """numpy 标量 / 数组参数也能正确落盘。"""
    meta = RunMeta(run_id='r1', out_dir=str(tmp_path))
    meta.set_params({'a': np.float64(1.5), 'b': np.array([1.0, 2.0]),
                     'c': [1, 2, 3], 'd': True, 'e': None})
    p = meta.write()
    data = RunMeta.read(p)
    assert data['params']['a'] == 1.5
    assert data['params']['b'] == [1.0, 2.0]
    assert data['params']['c'] == [1, 2, 3]
    assert data['params']['d'] is True
    assert data['params']['e'] is None


def test_run_meta_rejects_unknown_types_and_bad_json(tmp_path):
    meta = RunMeta(out_dir=str(tmp_path))
    with pytest.raises(TypeError, match='不支持的元数据类型'):
        meta.set_extra('unsupported', object())
    damaged = tmp_path / 'broken.json'
    damaged.write_text('{broken')
    with pytest.raises(json.JSONDecodeError):
        RunMeta.read(str(damaged))


def test_io_errors_propagate_instead_of_reporting_success(tmp_path):
    blocked = tmp_path / 'not_a_directory'
    blocked.write_text('occupied')
    with pytest.raises(OSError):
        RunMeta().write(str(blocked / 'run_meta.json'))
