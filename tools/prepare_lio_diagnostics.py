#!/usr/bin/env python3
"""复制 Point-LIO 到容器 /tmp 并加入可切换诊断；不修改导航压缩包或安装版。

先 source ROS 与 /opt/lio_sim/install，再用 colcon 单线程构建复制的包。
诊断开关仅由该实验进程的环境变量启用，默认保留原算法行为。
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


PROBE = r'''
// 本仓库附加诊断：只检查原算法已经接受的平面；保留原函数和许可证。
struct RmLioPlaneDiagnostics {
  std::ofstream trace;
  size_t accepted = 0, collinear = 0;
  bool guard = std::getenv("RM_LIO_PLANE_RANK_GUARD") != nullptr;
  RmLioPlaneDiagnostics() {
    if (const char *path = std::getenv("RM_LIO_PLANE_TRACE")) {
      trace.open(path); trace << "legacy_accepted,collinear\n";
    }
  }
  ~RmLioPlaneDiagnostics() {
    if (trace.is_open()) trace << accepted << "," << collinear << "\n";
  }
};
template <typename T>
bool rm_lio_plane_diagnostics(const Matrix<T, NUM_MATCH_POINTS, 3> &A) {
  static RmLioPlaneDiagnostics stats;
  if (!stats.trace.is_open() && !stats.guard) return true;
  Matrix<T, NUM_MATCH_POINTS, 3> centered = A.rowwise() - A.colwise().mean();
  Eigen::SelfAdjointEigenSolver<Matrix<T, 3, 3>> eig(centered.transpose()*centered);
  auto values = eig.eigenvalues();
  bool bad = eig.info() != Eigen::Success || values(2) < T(1e-9)
             || values(1) < T(1e-4)*values(2);
  ++stats.accepted; if (bad) ++stats.collinear;
  if (stats.trace.is_open() && stats.accepted % 10000 == 0)
    stats.trace << stats.accepted << "," << stats.collinear << "\n";
  return !(stats.guard && bad);
}
'''

INPUT_PROBE = r'''
// 诊断回调实际收到的样本，区分 bag 完整与估计器实际收齐。
struct RmLioInputDiagnostics {
  std::ofstream out;
  size_t lidar = 0, imu = 0;
  double last_lidar = -1, last_imu = -1, lidar_gap = 0, imu_gap = 0;
  RmLioInputDiagnostics() {
    if (const char *path = std::getenv("RM_LIO_INPUT_TRACE")) out.open(path);
  }
  ~RmLioInputDiagnostics() {
    if (out.is_open()) out << "lidar_received,imu_received,lidar_max_gap_s,imu_max_gap_s\n"
      << lidar << "," << imu << "," << lidar_gap << "," << imu_gap << "\n";
  }
};
inline void rm_lio_record_input(bool is_imu, double stamp) {
  static RmLioInputDiagnostics stats;
  if (!stats.out.is_open()) return;
  double &last = is_imu ? stats.last_imu : stats.last_lidar;
  double &gap = is_imu ? stats.imu_gap : stats.lidar_gap;
  size_t &count = is_imu ? stats.imu : stats.lidar;
  if (last >= 0) gap = std::max(gap, stamp-last);
  last = stamp; ++count;
}
'''


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('源码形状与已审阅版本不同，拒绝自动修改: '+old[:100])
    return text.replace(old, new)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('/opt/lio_sim/src/point_lio'))
    parser.add_argument('--destination', type=Path, default=Path('/tmp/lio_diagnostics/src/point_lio'))
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    source, destination = args.source.resolve(), args.destination.resolve()
    if not destination.is_relative_to(Path('/tmp')) or destination == source:
        parser.error('诊断源码只能创建在容器 /tmp 新目录中')
    shutil.copytree(source, destination)
    names = ['include/common_lib.h', 'src/laserMapping.cpp', 'src/li_initialization.cpp']
    manifest = {'source': str(source), 'destination': str(destination), 'files': {}}
    for name in names:
        path = destination/name
        text = path.read_text()
        entry = {'original_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        if name.endswith('common_lib.h'):
            signature = 'template <typename T>\nbool esti_plane('
            start = text.index(signature)
            tail = text.index('\ninline double get_time_sec', start)
            function = text[start:tail]
            function = replace_once(function, '  return true;\n}', '  return rm_lio_plane_diagnostics(A);\n}')
            text = '#include <fstream>\n#include <cstdlib>\n#include <Eigen/Eigenvalues>\n'+text[:start]+PROBE+function+text[tail:]
        elif name.endswith('laserMapping.cpp'):
            old = '            ivox_->AddPoints(init_feats_world->points);'
            new = '''            if (std::getenv("RM_LIO_INIT_VOXEL")) {
              PointCloudXYZI unique;
              downSizeFilterMap.setInputCloud(init_feats_world);
              downSizeFilterMap.filter(unique);
              ivox_->AddPoints(unique.points);
            } else {
              ivox_->AddPoints(init_feats_world->points);
            }'''
            text = replace_once(text, old, new)
            if text.count('rclcpp::SensorDataQoS()') != 3:
                raise ValueError('传感器QoS调用与已审阅版本不同')
            text = text.replace('rclcpp::SensorDataQoS()', 'rm_sensor_qos')
            marker = '  /*** ROS subscribe initialization ***/'
            text = replace_once(text, marker, '''  auto rm_sensor_qos = rclcpp::SensorDataQoS();
  if (const char *depth = std::getenv("RM_LIO_QOS_DEPTH"))
    rm_sensor_qos.keep_last(std::stoul(depth));
'''+marker)
        else:
            text = '#include <fstream>\n#include <cstdlib>\n'+INPUT_PROBE+text
            for signature, name, is_imu in (
                ('void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)', 'msg', 'false'),
                ('void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in)', 'msg_in', 'true')):
                text = replace_once(text, signature+'\n{', signature+'\n{\n  rm_lio_record_input('+is_imu+', rclcpp::Time('+name+'->header.stamp).seconds());')
        path.write_text(text)
        entry['diagnostic_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest['files'][name] = entry
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(destination)


if __name__ == '__main__':
    main()
