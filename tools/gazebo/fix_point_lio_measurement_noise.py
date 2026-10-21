#!/usr/bin/env python3
"""修复 Point-LIO 卡尔曼更新两条计算路径对测量方差的不同解释。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


ORIGINAL_SHA256 = 'f30e55220a06cd465bb49b9c8d83d788b66b61be9e1510d6637c9c468ab56d97'
RELATIVE_HEADER = Path('include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp')
REPLACEMENTS = {
    'Matrix<scalar_type, 12, 12> HTH = m_noise * h_x.transpose() * h_x;':
        'Matrix<scalar_type, 12, 12> HTH = h_x.transpose() * h_x / m_noise;',
    'K_ = P_inv.template block<n, 12>(0, 0) * h_x.transpose() * m_noise;':
        'K_ = P_inv.template block<n, 12>(0, 0) * h_x.transpose() / m_noise;',
}


def compile_probe(header, output):
    subprocess.run([
        'g++', '-std=c++17', '-O0', '-D_GLIBCXX_ASSERTIONS', '-I/usr/include/eigen3',
        '-DRM_TEST_EKF_HEADER="'+str(header.resolve())+'"',
        str(Path(__file__).with_name('test_point_lio_measurement_noise.cpp')),
        '-o', str(output),
    ], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    header = args.source/RELATIVE_HEADER
    original = header.read_bytes()
    if hashlib.sha256(original).hexdigest() != ORIGINAL_SHA256:
        raise ValueError('Point-LIO 滤波器源码与已审阅版本不同，拒绝自动修改')
    fixed = original.decode()
    for old, new in REPLACEMENTS.items():
        if fixed.count(old) != 1:
            raise ValueError('Point-LIO 测量更新公式与已审阅结构不同')
        fixed = fixed.replace(old, new)
    with tempfile.TemporaryDirectory() as folder:
        temporary = Path(folder)
        legacy_probe = temporary/'legacy_measurement_test'
        compile_probe(header, legacy_probe)
        subprocess.run([str(legacy_probe), '--expect-legacy'], check=True)
        header.write_text(fixed)
        fixed_probe = temporary/'fixed_measurement_test'
        compile_probe(header, fixed_probe)
        subprocess.run([str(fixed_probe)], check=True)
    args.manifest.write_text(json.dumps({
        'variant': 'consistent_measurement_variance',
        'original_esekfom_sha256': ORIGINAL_SHA256,
        'patched_esekfom_sha256': hashlib.sha256(header.read_bytes()).hexdigest(),
        'regression_uses_actual_esekfom_header': True,
        'legacy_measurement_branch_disagreement_reproduced': True,
        'measurement_variances': [.01, 1., 100.],
        'regression_cases': 9,
        'cases': ['23个有效观测', '补零到24行跨分支', '重复到46行并使方差加倍'],
    }, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
