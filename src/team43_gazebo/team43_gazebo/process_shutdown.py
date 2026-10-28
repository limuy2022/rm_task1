"""通过 Gazebo 停止服务正常收尾，再按进程组回收本次启动的子进程。"""
import os
from pathlib import Path
import signal
import subprocess


def stop_processes(processes, output, timeout=8.):
    # 暂停的无窗口 Gazebo 偶尔无法仅靠 SIGINT 结束渲染线程。
    # 先走服务端停止流程；服务失败仍保留信号回收，强杀仍算验收失败。
    for name, process in processes:
        if name != 'gazebo' or process.poll() is not None:
            continue
        try:
            result = subprocess.run([
                'ign', 'service', '-s', '/server_control',
                '--reqtype', 'ignition.msgs.ServerControl',
                '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000',
                '--req', 'stop: true',
            ], capture_output=True, text=True, timeout=5.)
            Path(output, 'gazebo_stop.log').write_text(result.stdout+result.stderr)
            if result.returncode == 0 and 'true' in result.stdout:
                process.wait(timeout=timeout)
        except (OSError, subprocess.TimeoutExpired) as exc:
            with Path(output, 'gazebo_stop.log').open('a') as log:
                log.write(repr(exc)+'\n')
    for _, process in reversed(processes):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    codes = {}
    for name, process in reversed(processes):
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3.)
        codes[name] = process.returncode
    return codes
