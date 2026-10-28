"""接收审核必须区分真实源抖动、接收丢样与源采样质量。"""
import csv
import pytest
from team43_gazebo import input_audit as audit


def run_audit(output, lidar=(0., .1, .226, .326), imu=(0., .005, .01, .015),
              received=None, passed=True):
    row = dict(lidar_received=len(lidar), imu_received=len(imu),
               lidar_max_gap_s=.126, imu_max_gap_s=.005,
               lidar_backward=0, imu_backward=0)
    row.update(received or {})
    with (output/'input_audit.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=row)
        writer.writeheader()
        writer.writerow(row)
    report = {'passed': passed}
    audit.audit_received_inputs(report, output, lidar, imu)
    return report


def test_complete_input_with_source_jitter_passes(tmp_path):
    report = run_audit(tmp_path)
    assert report['passed']
    assert report['sensor_input_audit']['lidar_published_max_gap_s'] == pytest.approx(.126)
    with (tmp_path/'lidar_published_stamps.csv').open() as stream:
        assert len(list(csv.DictReader(stream))) == 4


@pytest.mark.parametrize('received,failed', [
    ({'lidar_received': 3, 'lidar_max_gap_s': .226}, 'estimator_received_lidar'),
    ({'imu_received': 3}, 'estimator_received_imu'),
    ({'lidar_max_gap_s': .15}, 'estimator_lidar_continuity'),
    ({'imu_max_gap_s': .01}, 'estimator_imu_continuity'),
    ({'lidar_backward': 1}, 'estimator_input_order'),
    ({'imu_backward': 1}, 'estimator_input_order'),
])
def test_loss_added_gap_or_backward_input_fails(tmp_path, received, failed):
    report = run_audit(tmp_path, received=received)
    assert not report['passed']
    assert report['checks'][failed] is False


def test_source_imu_quality_limit_remains_independent(tmp_path):
    report = run_audit(tmp_path, imu=(0., .005, .025), received={'imu_max_gap_s': .02})
    assert not report['passed']
    assert report['checks']['estimator_imu_continuity']
    assert not report['checks']['source_imu_continuity']


@pytest.mark.parametrize('lidar', [(), (0.,), (0., float('nan')), (0., .1, .1)])
def test_missing_or_invalid_source_is_rejected(tmp_path, lidar):
    assert not run_audit(tmp_path, lidar=lidar)['passed']


def test_missing_receiver_audit_fails(tmp_path):
    report = {'passed': True}
    audit.audit_received_inputs(report, tmp_path, (0., .1), (0., .005))
    assert not report['passed']


def test_input_audit_cannot_override_existing_failure(tmp_path):
    assert not run_audit(tmp_path, passed=False)['passed']
