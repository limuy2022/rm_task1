"""配置入口必须在缺失或非法结构时终止，不能静默使用默认参数。"""
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from team43_common.config_loader import ConfigError, load_ros_params, load_yaml


def test_missing_and_empty_documents(tmp_path):
    path = tmp_path / 'config.yaml'
    with pytest.raises(ConfigError):
        load_yaml(path)
    assert load_yaml(path, required=False) == {}
    path.write_text('', encoding='utf-8')
    assert load_yaml(path) == {}
    with pytest.raises(ConfigError):
        load_yaml(path, allow_empty=False)
    with pytest.raises(ConfigError):
        load_ros_params(path)


@pytest.mark.parametrize('content', [
    '[]', 'null', '{}', '/**: []', '/**: {}',
    '/**: {ros__parameters: []}', '/**: {ros__parameters: null}',
    'node: {ros__parameters: {enabled: false}}',
])
def test_reject_invalid_parameter_structure(tmp_path, content):
    path = tmp_path / 'config.yaml'
    path.write_text(content, encoding='utf-8')
    with pytest.raises(ConfigError):
        load_ros_params(path)


def test_parameters_preserve_types_and_allow_empty_mapping(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text('/**:\n  ros__parameters:\n    enabled: false\n    axes: [1, 2]\n', encoding='utf-8')
    assert load_ros_params(path) == {'enabled': False, 'axes': [1, 2]}
    path.write_text('/**: {ros__parameters: {}}', encoding='utf-8')
    assert load_ros_params(path) == {}
    path.write_text('/**: [broken', encoding='utf-8')
    with pytest.raises(yaml.YAMLError):
        load_ros_params(path)
