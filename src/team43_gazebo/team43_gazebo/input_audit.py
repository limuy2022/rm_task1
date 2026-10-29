"""对照源时间戳审核估计器实收，区分采样抖动和传输新增空洞。"""
import csv
import math


def source_max_gap(stamps):
    if len(stamps) < 2 or not all(math.isfinite(t) for t in stamps):
        raise ValueError('源输入须至少含两个有限时间戳')
    gaps = [b-a for a, b in zip(stamps, stamps[1:])]
    if min(gaps) <= 0:
        raise ValueError('源输入时间戳须严格递增')
    return max(gaps)


def audit_received_inputs(report, output, lidar_stamps, imu_stamps):
    """退出并刷新文件后审核；周期参照实际发布数据，容差仅用于数值转换。"""
    try:
        lidar_gap = source_max_gap(lidar_stamps)
        imu_gap = source_max_gap(imu_stamps)
        with (output/'lidar_published_stamps.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp_s'])
            writer.writerows((t,) for t in lidar_stamps)
        with (output/'input_audit.csv').open() as stream:
            row = next(csv.DictReader(stream))
        data = {key: float(value) if 'gap' in key or 'stamp' in key else int(value)
                for key, value in row.items()}
        data.update(lidar_published=len(lidar_stamps), imu_observed=len(imu_stamps),
                    lidar_published_max_gap_s=lidar_gap, imu_observed_max_gap_s=imu_gap,
                    gap_comparison_tolerance_s=1e-6, sensor_qos_depth=200,
                    imu_qos_reliable=True)
        data.update(lidar_source_first_stamp_s=lidar_stamps[0],
                    lidar_source_last_stamp_s=lidar_stamps[-1],
                    imu_source_first_stamp_s=imu_stamps[0],
                    imu_source_last_stamp_s=imu_stamps[-1])
        checks = {
            'estimator_received_lidar': data['lidar_received'] >= .999*len(lidar_stamps),
            'estimator_received_imu': data['imu_received'] >= .999*len(imu_stamps),
            'source_imu_continuity': imu_gap <= .010001,
            'estimator_imu_continuity': data['imu_max_gap_s'] <= imu_gap+1e-6,
            'estimator_lidar_continuity': data['lidar_max_gap_s'] <= lidar_gap+1e-6,
            'estimator_input_order': data['lidar_backward'] == data['imu_backward'] == 0,
        }
        report['sensor_input_audit'] = data
    except (OSError, StopIteration, ValueError, KeyError) as exc:
        report['sensor_input_audit'] = {'error': repr(exc)}
        checks = {'estimator_input_audit_available': False}
    report.setdefault('checks', {}).update(checks)
    report['passed'] = report['passed'] and all(checks.values())
