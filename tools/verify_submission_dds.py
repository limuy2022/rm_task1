"""隔离域的真实评分提交集成验收：测试源→DDS→生产节点→CSV→独立读者。

默认30拍快速回归；--count 1800 验证完整180秒。只用/test节拍及自有临时目录，
不发布/judge消息、不访问设备。测量当前容器主机，不代表公共NUC已通过。
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import yaml

from score_csv import evaluate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/grid_mapper'))
from grid_mapper.pgm_io import read_pgm  # noqa: E402


def verify(count, output):
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(100 + os.getpid() % 80))
    work = Path(tempfile.mkdtemp(prefix='submission_dds_'))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    processes, logs = {}, []
    report = dict(passed=False, ticks_requested=count, ros_domain_id=env['ROS_DOMAIN_ID'],
                  kind='real_dds_submission', checks={}, hardware_accessed=False,
                  limitations=['固定合成状态；不验证感知和预测精度', '当前容器主机的墙钟测量，不代表NUC截止时间'])

    def launch(name, args):
        log = (work / (name + '.log')).open('w')
        logs.append(log)
        processes[name] = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)

    try:
        launch('writer', ['ros2', 'launch', 'gimbal_tracker_bringup', 'submission.launch.py',
                          'output_dir:=' + str(work / 'data'), 'tick_topic:=/test/tick',
                          'replay_topic:=/test/replay_start'])
        launch('observer', [sys.executable, '-B', str(ROOT / 'tools/observe_submission.py'),
                            str(work / 'data/47.csv'), '--count', str(count),
                            '--timeout', str(count / 10. + 45.)])
        launch('source', [sys.executable, '-B', str(ROOT / 'tools/submission_test_source.py'),
                          '--count', str(count)])
        deadline = time.monotonic() + count / 10. + 50.
        while any(processes[name].poll() is None for name in ('source', 'observer')):
            if processes['writer'].poll() is not None:
                raise RuntimeError('评分节点提前退出')
            for name in ('source', 'observer'):
                if processes[name].poll() not in (None, 0):
                    raise RuntimeError(name + ' 失败，见对应日志')
            if time.monotonic() >= deadline:
                raise RuntimeError('真实提交验收超时')
            time.sleep(.05)
        observer = json.loads((work / 'observer.log').read_text())
        assert observer['passed'], observer
        assert processes['writer'].poll() is None, '独立读者必须在写出节点仍运行时看见全部行'
        report['observer'] = observer
        report['checks']['visible_while_writer_running'] = True

        with (work / 'data/47.csv').open() as stream:
            rows = list(csv.reader(stream))
        assert len(rows) == count
        stamps = [float(row[8]) for row in rows]
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        span = stamps[-1] - stamps[0]
        assert (count - 1) * .099 <= span <= count / 10. + 2., span
        assert max(gaps, default=0.) < .2, gaps
        report.update(tick_span_s=span, max_tick_gap_s=max(gaps, default=0.))
        reference = work / 'reference.csv'
        with reference.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t_tick', 'pred_x', 'pred_y', 'cur_x', 'cur_y', 'base_x', 'base_y', 'std_dir_deg'])
            for row in rows:
                writer.writerow([row[8], 1., 2., 1., 2., .5, -.25, math.degrees(math.atan2(.4, .3))])
        score = evaluate(work / 'data/47.csv', reference, field_long_side_m=10.)
        assert score['total'] is not None and score['total'] < 1e-6, score
        assert score['meta']['matched'] == count and not score['meta']['format_issues']
        report.update(score=score)
        report['checks']['all_ticks_and_nine_columns_score_correctly'] = True
        # 非对称像素与YAML预期独立于生产序列化公式。
        pixels, _ = read_pgm(str(work / 'data/47_map.pgm'))
        assert pixels.tolist() == [[254, 254, 0, 254], [254, 0, 254, 254], [0, 254, 254, 205]]
        metadata = yaml.safe_load((work / 'data/47_map.yaml').read_text())
        assert metadata['origin'] == [-1., -.5, 0.] and metadata['resolution'] == .5
        report['checks']['actual_map_pixels_and_metadata'] = True
        report['passed'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        for name, process in processes.items():
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        report['exit_codes'] = {name: process.returncode for name, process in processes.items()}
        for log in logs: log.close()
        report['clean_shutdown'] = (all(code == 0 for code in report['exit_codes'].values())
                                    and 'process has died' not in (work / 'writer.log').read_text())
        report['passed'] = report['passed'] and report['clean_shutdown']
        shutil.copytree(work, output, dirs_exist_ok=True)
        shutil.rmtree(work)
    (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=30)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'build/offline/submission_dds')
    args = parser.parse_args()
    if not 1 <= args.count <= 1800: parser.error('count须为1..1800')
    return verify(args.count, args.output_dir)


if __name__ == '__main__':
    raise SystemExit(main())
