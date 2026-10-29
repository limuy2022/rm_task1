#pragma once

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace rm_task1 {

// 默认互斥回调组内计数；在估计器实际入口审核，不能用桥接端数量代替。
struct InputAudit {
  std::ofstream trace;
  std::size_t count[2] = {0, 0}, backward[2] = {0, 0};
  double last[2] = {-1, -1}, gap[2] = {0, 0};
  double first[2] = {-1, -1};

  void configure(const std::string& path) {
    if (path.empty()) return;
    trace.open(path);
    if (!trace) throw std::runtime_error("无法创建传感器接收审核文件");
  }
  void record(bool imu, double stamp) {
    if (!trace.is_open()) return;
    const int index = imu ? 1 : 0;
    if (count[index] == 0) first[index] = stamp;
    if (last[index] >= 0) {
      gap[index] = std::max(gap[index], stamp - last[index]);
      if (stamp <= last[index]) ++backward[index];
    }
    last[index] = stamp;
    ++count[index];
  }
  ~InputAudit() {
    if (trace.is_open())
      trace << std::setprecision(17)
            << "lidar_received,imu_received,lidar_max_gap_s,imu_max_gap_s,lidar_backward,imu_backward,"
               "lidar_first_stamp_s,lidar_last_stamp_s,imu_first_stamp_s,imu_last_stamp_s\n"
            << count[0] << ',' << count[1] << ',' << gap[0] << ',' << gap[1] << ','
            << backward[0] << ',' << backward[1] << ','
            << first[0] << ',' << last[0] << ',' << first[1] << ',' << last[1] << '\n';
  }
};

}  // namespace rm_task1
