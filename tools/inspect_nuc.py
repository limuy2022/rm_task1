"""Read-only field checks. Never opens a serial port or starts a ROS node.

Uses filesystem metadata, ament lookup and message type-support imports only.
A type-support import is not a DDS transport or sensor-data test.
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
from pathlib import Path
import stat
import traceback


def inspect_serial_alias(path: Path) -> dict:
    result = {'path': str(path), 'opens_serial_device': False}
    try:
        info = path.lstat()
    except FileNotFoundError:
        return dict(result, status='missing', detail='路径本身不存在')
    except OSError as exc:
        return dict(result, status='metadata_error', detail=str(exc))
    result['is_symlink'] = stat.S_ISLNK(info.st_mode)
    if result['is_symlink']:
        result['link_text'] = os.readlink(path)
    try:
        resolved = path.resolve(strict=True)
        target_info = resolved.stat()
    except FileNotFoundError:
        return dict(result, status='dangling', detail='链接存在，最终目标不存在；这是当前状态')
    except (OSError, RuntimeError) as exc:
        return dict(result, status='resolve_error', detail=str(exc))
    result.update(resolved_target=str(resolved), mode=oct(stat.S_IMODE(target_info.st_mode)))
    if not stat.S_ISCHR(target_info.st_mode):
        return dict(result, status='not_character_device', detail='目标不是字符设备，不能当真实TTY使用')
    result.update(status='character_device', device_major=os.major(target_info.st_rdev),
                  device_minor=os.minor(target_info.st_rdev),
                  detail='字符设备存在；不证明对端身份、固件状态或是否收到协议数据')
    return result


def inspect_ros(expected_install: Path | None = None) -> dict:
    result = {
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'python_executable': os.sys.executable,
        'starts_ros_node': False,
    }
    try:
        packages = importlib.import_module('ament_index_python.packages')
        prefix = packages.get_package_prefix('gimbal_interfaces')
        result['interfaces_prefix'] = prefix
        result['perception_prefix'] = packages.get_package_prefix('gimbal_perception')
        if expected_install is not None:
            root = expected_install.resolve()
            result['expected_install'] = str(root)
            result['prefix_in_expected_install'] = all(
                Path(result[key]).resolve().is_relative_to(root)
                for key in ('interfaces_prefix', 'perception_prefix'))
        utilities = importlib.import_module('rosidl_runtime_py.utilities')
        support = importlib.import_module('rclpy.type_support')
        for name in ('MarkerObservation', 'MarkerCorners'):
            cls = utilities.get_message('gimbal_interfaces/msg/' + name)
            support.check_for_type_support(cls)
            result[name] = {'python_module': cls.__module__, 'type_support_imported': True}
        result['status'] = 'type_support_ok'
    except Exception as exc:
        result.update(status='import_failed', error=f'{type(exc).__name__}: {exc}',
                      traceback=traceback.format_exc())
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial-link', type=Path, default=Path('/dev/gimbal'))
    parser.add_argument('--expected-install', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    report = {'serial': inspect_serial_alias(args.serial_link),
              'ros': inspect_ros(args.expected_install),
              'limitations': ['No device open/read/write, no ROS node/DDS test, no motor command.']}
    print('串口路径：', report['serial']['status'], '—', report['serial']['detail'])
    print('当前终端：', report['ros']['status'])
    if report['ros']['status'] == 'import_failed':
        print('消息类型导入失败；先source本队安装环境，再运行本工具。若仍失败，保留完整JSON。')
        print(report['ros']['error'])
    else:
        print('消息类型支持可加载；仍须保持视觉运行，才能实际echo到消息。')
        if report['ros'].get('prefix_in_expected_install') is False:
            print('注意：包路径不在预期安装目录，先纠正环境，不继续视觉验收。')
    content = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content)
        print('完整结果已保存：', args.output)
    else:
        print(content)
    return 0 if (report['ros']['status'] == 'type_support_ok' and
                 report['ros'].get('prefix_in_expected_install') is not False) else 1


if __name__ == '__main__':
    raise SystemExit(main())
