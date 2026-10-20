#!/usr/bin/env python3
"""审计仿真 Livox/IMU 记录，并用压缩包原版 Eigen 平面函数审核初始化邻域。

只生成独立诊断，不修改原 bag，不给定位器提供真值。须在 gazebo-sim 容器中运行。
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from scipy.spatial import cKDTree
import yaml


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(2**20), b''): digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--lio-source', type=Path, default=Path('/opt/lio_sim/src/point_lio'))
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source/'sensor_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sim/gazebo/livox/lidar', '/sim/gazebo/imu',
                      '/clock', '/sim/gazebo/lio/aft_mapped_to_init']))
    clouds, imus, first, offsets = [], [], None, []
    latest_clock, pose_delays = None, []
    timebase_errors, backward_offsets = 0, 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/clock':
            latest_clock = seconds(deserialize_message(data, Clock).clock)
        elif topic.endswith('/aft_mapped_to_init'):
            msg = deserialize_message(data, Odometry)
            if latest_clock is not None:
                pose_delays.append([seconds(msg.header.stamp), latest_clock-seconds(msg.header.stamp)])
        elif topic.endswith('/imu'):
            msg = deserialize_message(data, Imu)
            a = msg.linear_acceleration; w = msg.angular_velocity
            imus.append([seconds(msg.header.stamp), a.x, a.y, a.z, w.x, w.y, w.z])
        else:
            msg = deserialize_message(data, CustomMsg)
            clouds.append([seconds(msg.header.stamp), msg.point_num])
            off = np.array([p.offset_time for p in msg.points], dtype=np.int64)
            timebase_errors += int(msg.timebase != msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            backward_offsets += int(np.any(np.diff(off) < 0))
            offsets.append(int(off.max()))
            if first is None:
                first = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
    if first is None or len(imus) < 100: raise RuntimeError('记录缺少传感器数据')
    config = yaml.safe_load((source/'point_lio.yaml').read_text())['/**']['ros__parameters']
    offset = np.array(config['mapping']['extrinsic_T'])
    # 复现 Livox 原预处理的索引过滤；静止期各批云坐标相同。
    selected = first[config['point_filter_num']::config['point_filter_num']]
    initial_map = (selected+offset).astype(np.float32)
    keys = np.floor(selected/config['filter_size_surf']).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    query = np.array([selected[inverse == i].mean(axis=0) for i in range(inverse.max()+1)])
    query = (query+offset).astype(np.float32)
    neighbors = initial_map[cKDTree(initial_map).query(query, k=5)[1]]
    boxes = []
    for model in ET.parse(source/'world.sdf').findall('.//world/model'):
        if model.findtext('static') != 'true': continue
        visual = model.find('.//visual')
        size = visual.find('geometry/box/size') if visual is not None else None
        if size is None: continue
        center = np.array(list(map(float, model.findtext('pose').split())))[:3]-[0., 0., .7]
        center += np.array(list(map(float, visual.findtext('pose', '0 0 0 0 0 0').split())))[:3]
        boxes.append((center, np.array(list(map(float, size.text.split())))/2))
    flat = neighbors.reshape(-1, 3)
    face_distances, face_axes = [], []
    for center, half in boxes:
        for axis in range(3):
            others = [k for k in range(3) if k != axis]
            inside = np.all(np.abs(flat[:, others]-center[others]) <= half[others]+.02, axis=1)
            for sign in (-1, 1):
                distance = np.abs(flat[:, axis]-(center[axis]+sign*half[axis]))
                face_distances.append(np.where(inside, distance, np.inf)); face_axes.append(axis)
    labels = np.argmin(face_distances, axis=0).reshape(-1, 5)
    best_distance = np.min(face_distances, axis=0).reshape(-1, 5)
    homogeneous = np.all(labels == labels[:, :1], axis=1) & np.all(best_distance < .02, axis=1)
    # 探针直接编译所用版本的原函数，避免 numpy 求解器与 Eigen QR 的差异。
    original = (args.lio_source/'include/common_lib.h').read_text()
    start = original.index('template <typename T>\nbool esti_plane(')
    end = original.index('\ninline double get_time_sec', start)
    function = original[start:end]
    probe = '''#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
using namespace Eigen;
#define NUM_MATCH_POINTS 5
struct Point {float x,y,z;};
using PointVector = std::vector<Point>;
'''+function+'''
int main() {
  PointVector points(5); int axis;
  std::cout << "accepted,collinear,normal_error_deg\\n";
  while (std::cin >> axis) {
    Matrix<float,5,3> A;
    for(int i=0;i<5;++i) { std::cin >> points[i].x >> points[i].y >> points[i].z;
      A.row(i) << points[i].x,points[i].y,points[i].z; }
    auto centered = (A.rowwise()-A.colwise().mean()).eval();
    SelfAdjointEigenSolver<Matrix3f> eig(centered.transpose()*centered);
    auto e=eig.eigenvalues(); bool bad=e(2)<1e-9f || e(1)<1e-4f*e(2);
    Matrix<float,4,1> p; bool accepted=esti_plane(p,points,0.1f);
    double cosine=std::min(1.,std::abs(double(p(axis))));
    std::cout << accepted << "," << bad << "," << std::acos(cosine)*180/3.141592653589793 << "\\n";
  }
}
'''
    (output/'original_plane_probe.cpp').write_text(probe)
    executable = output/'original_plane_probe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3', str(output/'original_plane_probe.cpp'), '-o', str(executable)], check=True)
    rows = [str(face_axes[label[0]])+' '+' '.join(map(str, patch.reshape(-1)))
            for patch, label in zip(neighbors[homogeneous], labels[homogeneous])]
    result = subprocess.run([str(executable)], input='\n'.join(rows)+'\n', text=True, capture_output=True, check=True)
    (output/'plane_neighborhoods.csv').write_text(result.stdout)
    stats = np.loadtxt(output/'plane_neighborhoods.csv', delimiter=',', skiprows=1)
    imu, cloud = np.array(imus), np.array(clouds)
    normal_axes = np.array(face_axes)[labels[:, 0]]
    ideal_normals = np.eye(3)[normal_axes[homogeneous]]
    ideal_jacobian = np.column_stack([ideal_normals, np.cross(query[homogeneous], ideal_normals)])
    ideal_information = ideal_jacobian.T@ideal_jacobian/len(ideal_jacobian)
    report = {
        'bag_sha256': {path.name: file_sha256(path) for path in sorted((source/'sensor_bag').glob('*.db3'))},
        'lidar_frames': len(clouds), 'imu_samples': len(imus),
        'lidar_stamp_gap_max_s': float(np.max(np.diff(cloud[:, 0]))),
        'imu_stamp_gap_max_s': float(np.max(np.diff(imu[:, 0]))),
        'timebase_errors': timebase_errors, 'frames_with_backward_offsets': backward_offsets,
        'subscan_span_ns': [min(offsets), max(offsets)],
        'static_acceleration_mean': imu[imu[:, 0] < 3., 1:4].mean(axis=0).tolist(),
        'configured_raw_acceleration_saturation': config['mapping']['satu_acc'],
        'output_saturation_check_active': not config['use_imu_as_input'] and config['check_satu'],
        'static_acceleration_saturation_fraction_each_axis': np.mean(
            np.abs(imu[imu[:, 0] < 3., 1:4]) >= .99*config['mapping']['satu_acc'], axis=0).tolist(),
        'first_frame_points': len(first), 'first_frame_unique_coordinates': len(np.unique(first, axis=0)),
        'initial_map_after_index_filter': len(initial_map), 'initial_map_unique_coordinates': len(np.unique(initial_map, axis=0)),
        'homogeneous_true_surface_neighborhoods': int(np.sum(homogeneous)),
        'legacy_accepted_fraction': float(np.mean(stats[:, 0])),
        'collinear_fraction': float(np.mean(stats[:, 1])),
        'legacy_accepted_wrong_normal_over_10deg_fraction': float(np.mean((stats[:, 0] == 1) & (stats[:, 2] > 10))),
        'normal_error_p95_deg': float(np.percentile(stats[:, 2], 95)),
        'true_surface_axis_fractions': [float(np.mean(normal_axes[homogeneous] == axis)) for axis in range(3)],
        'ideal_pose_information_eigenvalues': np.linalg.eigvalsh(ideal_information).tolist(),
        'original_plane_function_sha256': hashlib.sha256(function.encode()).hexdigest(),
        'limitations': ['初始化体素查询由独立numpy近似PCL，不是运行中完整iVox调用记录',
                        '平面函数使用原版Eigen代码；静止真实表面标签只用于离线审核']}
    if pose_delays:
        delay = np.asarray(pose_delays)
        report['recorded_pose_arrival_delay'] = {
            'p50_s': float(np.percentile(delay[:, 1], 50)),
            'p95_s': float(np.percentile(delay[:, 1], 95)),
            'max_s': float(np.max(delay[:, 1])),
            'fraction_over_0_3s': float(np.mean(delay[:, 1] > .3)),
            'basis': 'bag到达顺序中的最近/clock减定位header.stamp，近似消费者可用时延'}
        np.savetxt(output/'pose_arrival_delay.csv', delay, delimiter=',', header='source_stamp,arrival_delay_s', comments='')
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
