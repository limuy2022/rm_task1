// 直接测试归档/修复后的 iVox 头文件，覆盖同一输出容器被跨查询复用。
#include <iostream>
#include <numeric>
#include <string>

#include "ivox/ivox3d.h"

using Map = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, pcl::PointXYZ>;

bool require(bool condition, const char* description) {
  if (!condition) std::cerr << "FAIL: " << description << '\n';
  return condition;
}

int main(int argc, char** argv) {
  const bool legacy = argc == 2 && std::string(argv[1]) == "--expect-legacy";
  Map::Options options;
  options.resolution_ = 1.f;
  Map map(options);
  Map::PointVector points{{.1f,.1f,.1f}, {.2f,.1f,.1f}, {.1f,.2f,.1f},
                          {.2f,.2f,.1f}, {.3f,.2f,.1f}};
  map.AddPoints(points);
  const pcl::PointXYZ near(.15f, .15f, .1f), far(10.f, 10.f, 10.f);
  Map::PointVector reused;
  if (!require(map.GetClosestPoint(near, reused, 5) && reused.size() == 5,
               "近处查询应找到地图中的五个点")) return 1;
  const bool far_found = map.GetClosestPoint(far, reused, 5);
  if (legacy) {
    if (!require(!far_found && reused.size() == 5,
                 "原始头文件应复现失败查询遗留五个旧近邻")) return 2;
    std::cout << "PASS: 原始 iVox 头文件复现失败查询遗留旧近邻\n";
    return 0;
  }
  if (!require(!far_found && reused.empty(),
               "没有候选网格时，失败查询必须清空旧近邻")) return 3;
  if (!require(map.GetClosestPoint(near, reused, 5) && reused.size() == 5,
               "失败之后仍可重新查询到正确近邻")) return 4;
  if (!require(!map.GetClosestPoint(near, reused, 5, .001) && reused.empty(),
               "候选网格存在但超出半径时，也必须清空旧近邻")) return 5;
  Map empty_map(options);
  reused = points;
  if (!require(!empty_map.GetClosestPoint(near, reused, 5) && reused.empty(),
               "空地图查询必须清空调用方预填充的输出")) return 6;
  std::cout << "PASS: iVox 实际头文件的成功查询、网格外查询、半径外查询、空地图查询\n";
}
