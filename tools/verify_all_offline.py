"""Run owned regression checks in a sourced ROS2 Humble environment (see repo Dockerfile).

Uses the calling Python; expects /opt/ros + the workspace install to be sourced
so rclpy/launch/gimbal_interfaces are importable. C++ check compiles only
protocol+PTY test code under build/offline. Logs/results are saved there.
A missing prerequisite is a failed check, not an invented pass.
Run from any directory via this script's path.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-pytest', action='store_true',
                        help='标准全量流程已执行 pytest 时使用，避免重复运行')
    args = parser.parse_args()
    output = ROOT/'build/offline'; output.mkdir(parents=True,exist_ok=True)
    env = os.environ.copy()
    package_paths = [str(p) for p in (ROOT/'src').iterdir() if (p/'setup.py').exists()]
    env['PYTHONPATH'] = os.pathsep.join(package_paths + [env.get('PYTHONPATH', '')])
    names = ['perception','control','navigation','advisor','timing','kinematics','processing',
             'protocol','pipeline']
    # 按作业超时（秒）：默认 60；含真检测/闭环仿真的重作业放宽。
    timeouts = {'verify_visual_tracking_pipeline': 240, 'aiming_robustness': 240,
                'verify_visual_tracking_dds': 90, 'verify_submission_dds': 90}
    jobs = [] if args.skip_pytest else [
        ('pytest', [sys.executable,'-B','-m','pytest','-q','src','--import-mode=importlib'])]
    jobs.extend((name,[sys.executable,'-B',str(ROOT/'tools'/('verify_'+name+'_offline.py'))]) for name in names)
    jobs.extend((name,[sys.executable,'-B',str(ROOT/'tools'/(name+'.py'))])
                for name in ('verify_cloud_mapping','verify_calibration_entry',
                             'verify_visual_servo',
                             'verify_visual_tracking_pipeline',
                             'verify_visual_tracking_dds',
                             'verify_visual_tracking_launch',
                             'verify_tracking_robustness',
                             'verify_tracking_debug', 'verify_submission_dds'))
    jobs.append(('aiming_robustness',
                 [sys.executable,'-B',str(ROOT/'tools'/'verify_aiming_robustness.py'),
                  '--quick']))
    result = []
    for name, command in jobs:
        begin = time.perf_counter()
        try:
            proc = subprocess.run(command,cwd=ROOT,env=env,text=True,stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT,timeout=timeouts.get(name,60))
            code,text = proc.returncode,proc.stdout
        except (OSError,subprocess.TimeoutExpired) as exc:
            # 单项无法启动/超时是失败；保存原因后继续执行其它独立检查。
            code,text = 1,repr(exc)
        (output/(name+'.log')).write_text(text)
        row = dict(name=name,returncode=code,seconds=round(time.perf_counter()-begin,3))
        result.append(row); print(('PASS' if code==0 else 'FAIL'),name,flush=True)
        if code: print(text[-5000:],flush=True)
    report = {'kind':'ros_humble_container','python':sys.version,'checks':result,
              'passed':all(r['returncode']==0 for r in result),
              'not_verified':['real sensors and real-scene TF accuracy',
                              'physical gimbal control','NUC realtime deadlines']}
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
