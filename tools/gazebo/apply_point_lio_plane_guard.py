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
    fixed = function.replace('  return true;\n}',
                            '  return pca_result.allFinite() && rm_task1::valid_plane_neighborhood(A);\n}')
    guard = Path(__file__).with_name('point_lio_plane_guard.hpp')
    shutil.copyfile(guard, path.with_name(guard.name))
    path.write_text('#include "point_lio_plane_guard.hpp"\n'+text[:start]+fixed+text[end:])

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
  if (rm_task1::valid_plane_neighborhood(invalid)) return 7;
  std::cout << "PASS: 共线、重复、近共线、平面、刚体变换、离群点、非有限数\\n";
}
'''
        probe.write_text(code)
        executable = Path(folder)/'plane_test'
        subprocess.run(['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3',
                        '-I'+str(path.parent), str(probe), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    args.manifest.write_text(json.dumps({
        'variant': 'plane_rank_guard', 'original_common_lib_sha256': ORIGINAL_SHA256,
        'patched_common_lib_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'guard_sha256': hashlib.sha256(guard.read_bytes()).hexdigest(),
        'minimum_second_to_largest_eigenvalue_ratio': 1e-4,
        'geometry_regression_cases': 7,
    }, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
