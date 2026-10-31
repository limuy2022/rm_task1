"""生成单台MID360网络配置；端口定义沿用仓库归档的Livox官方格式。"""
from __future__ import annotations

import argparse
import ipaddress
import json
from pathlib import Path


def ipv4(value: str) -> str:
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_loopback or int(address) == 0xffffffff:
        raise ValueError('请填写雷达专用网络的实际单播IPv4地址，不能使用回环或通配地址')
    return str(address)


def mid360_config(host_ip: str, lidar_ip: str) -> dict:
    """不修改网卡，不扫描网络；雷达原始坐标由LIO负责转换。"""
    host_ip, lidar_ip = ipv4(host_ip), ipv4(lidar_ip)
    if host_ip == lidar_ip:
        raise ValueError('NUC网卡IP与雷达IP不能相同')
    return {
        'lidar_summary_info': {'lidar_type': 8},
        'MID360': {
            'lidar_net_info': dict(zip(
                ('cmd_data_port', 'push_msg_port', 'point_data_port', 'imu_data_port', 'log_data_port'),
                (56100, 56200, 56300, 56400, 56500))),
            'host_net_info': {
                'cmd_data_ip': host_ip, 'cmd_data_port': 56101,
                'push_msg_ip': host_ip, 'push_msg_port': 56201,
                'point_data_ip': host_ip, 'point_data_port': 56301,
                'imu_data_ip': host_ip, 'imu_data_port': 56401,
                'log_data_ip': '', 'log_data_port': 56501,
            },
        },
        'lidar_configs': [{
            'ip': lidar_ip, 'pcl_data_type': 1, 'pattern_mode': 0,
            'extrinsic_parameter': dict.fromkeys(('roll', 'pitch', 'yaw', 'x', 'y', 'z'), 0),
        }],
    }


def load_driver_config(path: str) -> tuple[Path, dict]:
    """启动前检查型号与坐标约定，不要求重复人工检查全部点云字段。"""
    config_path = Path(path).expanduser().resolve(strict=True)
    data = json.loads(config_path.read_text(encoding='utf-8'))
    if data.get('lidar_summary_info', {}).get('lidar_type') != 8 or 'MID360' not in data:
        raise ValueError('当前硬件入口支持MID360配置，请先确认实物型号')
    devices = data.get('lidar_configs', [])
    if len(devices) != 1:
        raise ValueError('本队入口要求恰好一台雷达')
    lidar_ip = ipv4(devices[0]['ip'])
    host = data['MID360']['host_net_info']
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        if ipv4(host[key]) == lidar_ip:
            raise ValueError('NUC接收IP不能与雷达IP相同')
    extrinsic = devices[0].get('extrinsic_parameter', {})
    if set(extrinsic) != {'roll', 'pitch', 'yaw', 'x', 'y', 'z'} or any(float(v) != 0. for v in extrinsic.values()):
        raise ValueError('驱动extrinsic_parameter必须为零；雷达到内部IMU的外参仅在LIO中应用')
    return config_path, data


def main(argv=None):
    parser = argparse.ArgumentParser(description='根据现场实测网络地址生成单台MID360配置，不操作网卡或设备')
    parser.add_argument('--model', default='mid360', choices=['mid360'])
    parser.add_argument('--host-ip', default='192.168.1.50', help='NUC网卡IPv4，默认值来自校内赛环境页')
    parser.add_argument('--lidar-ip', default='192.168.1.3', help='雷达IPv4，默认值来自校内赛环境页')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        data = mid360_config(args.host_ip, args.lidar_ip)
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(f'已生成雷达配置：{output}')
    print(f'启动：ros2 launch team43_bringup livox.launch.py config_file:={output}')


if __name__ == '__main__':
    main()
