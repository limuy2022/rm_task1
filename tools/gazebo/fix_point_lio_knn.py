#!/usr/bin/env python3
"""修复构建副本中 iVox 查询失败后遗留旧近邻的问题，并验证实际头文件。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


ORIGINAL_SHA256 = '476fab14fe0f42f4328364cbb1d61d186ec45adcadcb173260c6160572479069'
FUNCTION_START = '''bool IVox<dim, node_type, PointType>::GetClosestPoint(
  const PointType & pt, PointVector & closest_pt, int max_num, double max_range)
{
'''


def compile_probe(include_directory, output):
    # ROS 的依赖安装含 PCL 头文件，但未必包含 pcl_common.pc；仅编译头文件
    # 探针时无需链接 PCL 动态库，按容器里实际存在的版本定位包含目录。
    pcl_directories = [path for path in Path('/usr/include').glob('pcl-*')
                       if (path/'pcl/point_types.h').is_file()]
    if len(pcl_directories) != 1:
        raise ValueError('必须能唯一定位容器中的 PCL 头文件目录')
    subprocess.run([
        'g++', '-std=c++17', '-O2', '-D_GLIBCXX_ASSERTIONS',
        '-I'+str(include_directory), '-I'+str(pcl_directories[0]), '-I/usr/include/eigen3',
        str(Path(__file__).with_name('test_point_lio_knn.cpp')), '-o', str(output),
    ], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    header = args.source/'include/ivox/ivox3d.h'
    original = header.read_bytes()
    if hashlib.sha256(original).hexdigest() != ORIGINAL_SHA256:
        raise ValueError('iVox 源码与已审阅版本不同，拒绝自动修改')
    text = original.decode()
    if text.count(FUNCTION_START) != 1:
        raise ValueError('iVox 多近邻查询结构与已审阅版本不同')
    fixed = text.replace(FUNCTION_START, FUNCTION_START+'''  // 查询失败也必须清空复用的输出，避免估计器继续使用上一帧近邻。
  closest_pt.clear();
''')
    with tempfile.TemporaryDirectory() as folder:
        temporary = Path(folder)
        original_include = temporary/'original_include'
        shutil.copytree(args.source/'include/ivox', original_include/'ivox')
        legacy_probe = temporary/'legacy_knn_test'
        compile_probe(original_include, legacy_probe)
        subprocess.run([str(legacy_probe), '--expect-legacy'], check=True)
        header.write_text(fixed)
        fixed_probe = temporary/'fixed_knn_test'
        compile_probe(args.source/'include', fixed_probe)
        subprocess.run([str(fixed_probe)], check=True)
    args.manifest.write_text(json.dumps({
        'variant': 'clear_knn_output_before_query',
        'original_ivox_sha256': ORIGINAL_SHA256,
        'patched_ivox_sha256': hashlib.sha256(header.read_bytes()).hexdigest(),
        'legacy_stale_output_reproduced': True,
        'actual_ivox_header_regression_cases': 4,
    }, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
