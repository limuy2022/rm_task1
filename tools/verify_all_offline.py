"""Run owned offline checks with existing dependencies; no installs/ROS/hardware.

Uses the calling Python. C++ check compiles only protocol+PTY test code under
build/offline. Logs/results are saved there. A missing prerequisite is a failed
check, not an invented pass. Run from any directory via this script's path.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    output = ROOT/'build/offline'; output.mkdir(parents=True,exist_ok=True)
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join(str(p) for p in (ROOT/'src').iterdir() if (p/'setup.py').exists())
    names = ['perception','control','navigation','advisor','timing','kinematics','processing',
             'submission','protocol','pipeline']
    jobs = [('pytest', [sys.executable,'-B','-m','pytest','-q','src','--import-mode=importlib'])]
    jobs.extend((name,[sys.executable,'-B',str(ROOT/'tools'/('verify_'+name+'_offline.py'))]) for name in names)
    jobs.extend((name,[sys.executable,'-B',str(ROOT/'tools'/(name+'.py'))])
                for name in ('verify_vision_and_gate','verify_coordinate_regressions'))
    result = []
    for name, command in jobs:
        begin = time.perf_counter()
        try:
            proc = subprocess.run(command,cwd=ROOT,env=env,text=True,stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT,timeout=60)
            code,text = proc.returncode,proc.stdout
        except (OSError,subprocess.TimeoutExpired) as exc: code,text = 1,repr(exc)
        (output/(name+'.log')).write_text(text)
        row = dict(name=name,returncode=code,seconds=round(time.perf_counter()-begin,3))
        result.append(row); print(('PASS' if code==0 else 'FAIL'),name,flush=True)
        if code: print(text[-5000:],flush=True)
    report = {'kind':'offline_only','python':sys.version,'checks':result,
              'passed':all(r['returncode']==0 for r in result),
              'not_verified':['ROS2/Humble/DDS','NUC deadlines','real sensors/TF/accuracy','physical gimbal control']}
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
