"""安装资源须支持另存场景，并通过当前安装前缀解析 package URI。"""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest


@pytest.mark.parametrize('world_name', ['lio_room.sdf', 'lio_room_regression.sdf'])
def test_installed_world_survives_relocation(tmp_path, world_name):
    share = Path(get_package_share_directory('team43_gazebo'))
    prefix = Path(get_package_prefix('team43_gazebo'))
    tree = ET.parse(share/'worlds'/world_name)
    output = tmp_path/'world.sdf'
    tree.write(output, encoding='unicode')
    relocated = ET.parse(output)
    search_paths = [Path(path) for path in os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '').split(os.pathsep)
                    if path]
    # 用安装前缀计算，不能 resolve 到 --symlink-install 指向的源码目录。
    assert prefix/'share' in search_paths
    for uri in relocated.findall('.//mesh/uri'):
        assert uri.text.startswith('package://team43_gazebo/')
        relative = uri.text.removeprefix('package://')
        assert any((root/relative).is_file() for root in search_paths)
    assert (share/'meshes/scene.stl').is_file()
    plugin = relocated.find('.//plugin[@name="rm_task1::ForceController"]')
    assert plugin.attrib['filename'] == 'librm_task1_force_controller.so'
