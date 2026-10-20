#!/usr/bin/env python3
"""在构建副本上修复 Point-LIO 平面有效性检查，保留原归档与源码。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


ORIGINAL_SHA256 = 'b60ab7ffa402e0eb7dc72cfaf4c5e5efe988e88883ce253f4c40fc8056f4f230'
ESTIMATOR_SHA256 = 'ad89c28616161ec0553ce76ee8911c19f67f0669730fc4e8d6609114e58f1ddf'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    path = args.source/'include/common_lib.h'
    original = path.read_bytes()
    if hashlib.sha256(original).hexdigest() != ORIGINAL_SHA256:
        raise ValueError('Point-LIO 源码与已审阅版本不同，拒绝自动修改')
    text = original.decode()
    start = text.index('template <typename T>\nbool esti_plane(')
    end = text.index('\ninline double get_time_sec', start)
    function = text[start:end]
    if function.count('  return true;\n}') != 1:
        raise ValueError('平面函数结构与已审阅版本不同')
    # 首先取5点时，重复快照会把两个位置重复凑满5个近邻，静止时全部退化。
    # 在同一iVox搜索范围内多取一倍候选，并让拟合真正使用全部返回点。
    fixed = function[:function.index('{')+1]+'''
  if (point.size() < NUM_MATCH_POINTS) return false;
  Matrix<T, Dynamic, 3> A(point.size(), 3);
  for (size_t j = 0; j < point.size(); ++j) {
    A.row(j) << point[j].x, point[j].y, point[j].z;
  }
  return rm_task1::fit_plane(A, pca_result, threshold);
}
'''
    guard = Path(__file__).with_name('point_lio_plane_guard.hpp')
    shutil.copyfile(guard, path.with_name(guard.name))
    path.write_text('#include "point_lio_plane_guard.hpp"\n'+text[:start]+fixed+text[end:])
    estimator = args.source/'src/Estimator.cpp'
    estimator_original = estimator.read_bytes()
    if hashlib.sha256(estimator_original).hexdigest() != ESTIMATOR_SHA256:
        raise ValueError('Point-LIO 估计器源码与已审阅版本不同，拒绝自动修改')
    estimator_text = estimator_original.decode()
    old_query = 'ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);'
    if estimator_text.count(old_query) != 2:
        raise ValueError('Point-LIO 近邻查询结构与已审阅版本不同')
    estimator.write_text(estimator_text.replace(old_query,
        'ivox_->GetClosestPoint(point_world_j, points_near, 2 * NUM_MATCH_POINTS);'))

    # 直接编译归档的实际函数与修复函数，避免只测试一个相似的替身。
    with tempfile.TemporaryDirectory() as folder:
        probe = Path(folder)/'plane_test.cpp'
        code = '''#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include "point_lio_plane_guard.hpp"
using namespace Eigen;
constexpr int NUM_MATCH_POINTS = 5;
struct PointType { float x, y, z; };
using PointVector = std::vector<PointType>;
'''+function.replace('esti_plane(', 'legacy_plane(')+fixed+'''
int main() {
  Matrix<float, 4, 1> plane;
  PointVector line{{0,0,2},{0,0,2},{1,0,2},{1,0,2},{2,0,2}};
  if (!legacy_plane(plane, line, .1f) || esti_plane(plane, line, .1f)) return 1;
  PointVector identical(5, {0,0,2});
  if (esti_plane(plane, identical, .1f)) return 2;
  PointVector near_line{{0,0,2},{1,.0001f,2},{2,0,2},{3,-.0001f,2},{4,0,2}};
  if (esti_plane(plane, near_line, .1f)) return 3;
  PointVector surface{{0,0,2},{1,0,2},{0,1,2},{1,1,2},{.5f,.5f,2}};
  if (!esti_plane(plane, surface, .1f) || !plane.allFinite()) return 4;
  Matrix3f rotation = AngleAxisf(.7f, Vector3f(1,2,3).normalized()).toRotationMatrix();
  for (auto& point : surface) {
    Vector3f p = rotation * Vector3f(point.x, point.y, point.z) + Vector3f(2,-1,3);
    point = {p.x(), p.y(), p.z()};
  }
  if (!esti_plane(plane, surface, .1f)) return 5;
  surface[0].z += 3;
  if (esti_plane(plane, surface, .1f)) return 6;
  Matrix<float, 5, 3> invalid = Matrix<float, 5, 3>::Zero();
  invalid(0,0) = std::numeric_limits<float>::quiet_NaN();
  if (rm_task1::fit_plane(invalid, plane, .1f)) return 7;
  // 前五点重复共线，新增相邻扫描线后才能确定平面；须使用全部候选。
  PointVector expanded = line;
  for (const auto& point : line) expanded.push_back({point.x, 1, point.z});
  if (!esti_plane(plane, expanded, .1f) || std::abs(plane(2)) < .999f) return 8;
  PointVector origin{{0,0,0},{1,0,0},{0,1,0},{1,1,0},{.5f,.5f,0}};
  if (!esti_plane(plane, origin, .1f) || std::abs(plane(3)) > 1e-6f) return 9;
  PointVector warped{{-.2f,-.2f,2},{.2f,-.2f,2},{-.2f,.2f,2},{.2f,.2f,2},{0,0,2.1f}};
  if (!legacy_plane(plane, warped, .1f) || esti_plane(plane, warped, .1f)) return 10;
  std::cout << "PASS: 共线、重复、近共线、平面、刚体变换、离群点、非有限数、扩展近邻、过原点平面、非平面低残差邻域\\n";
}
'''
        probe.write_text(code)
        executable = Path(folder)/'plane_test'
        subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
                        '-I'+str(path.parent), str(probe), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    args.manifest.write_text(json.dumps({
        'variant': 'plane_pca_guard', 'original_common_lib_sha256': ORIGINAL_SHA256,
        'patched_common_lib_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'guard_sha256': hashlib.sha256(guard.read_bytes()).hexdigest(),
        'original_estimator_sha256': hashlib.sha256(estimator_original).hexdigest(),
        'patched_estimator_sha256': hashlib.sha256(estimator.read_bytes()).hexdigest(),
        'maximum_plane_neighbors': 10,
        'minimum_second_to_largest_eigenvalue_ratio': 1e-4,
        'maximum_smallest_to_second_eigenvalue_ratio': .01,
        'geometry_regression_cases': 10,
    }, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
