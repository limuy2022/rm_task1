"""Compile production C++ parser/SerialPort with sanitizers, then test OS-created PTY.

No physical port paths, automatic device selection, ROS, gtest or installation.
Uses CXX when set; otherwise discovers c++, g++, or clang++ on PATH.
PTY baud is 115200 for POSIX portability; this does not validate the 921600 wire,
rclcpp/DDS node behavior, firmware control, watchdog or real hardware safety.
"""
from pathlib import Path
import json
import os
import shlex
import shutil
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    configured = os.environ.get('CXX', '').strip()
    if configured:
        compiler = shlex.split(configured)
        if not compiler or shutil.which(compiler[0]) is None:
            raise SystemExit(f'CXX compiler not found: {configured}')
    else:
        executable_path = next((path for name in ('c++', 'g++', 'clang++')
                                if (path := shutil.which(name))), None)
        if executable_path is None:
            raise SystemExit('C++ compiler missing; set CXX or provide c++, g++, or clang++ on PATH')
        compiler = [executable_path]
    build = root / 'build' / 'offline'
    build.mkdir(parents=True, exist_ok=True)
    executable = build / 'verify_protocol_offline'
    command = compiler + ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
               '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-pthread',
               '-I', str(root / 'src/42-gimbal_serial/include'),
               str(root / 'tools/verify_protocol_offline.cpp'),
               str(root / 'src/42-gimbal_serial/src/serial_port.cpp'), '-o', str(executable)]
    subprocess.run(command, check=True, timeout=90)
    master, slave = os.openpty()  # OS allocates a fresh virtual terminal only.
    process = None
    try:
        path = os.ttyname(slave)
        process = subprocess.Popen([str(executable), str(root / 'protocol/golden_frames.txt'),
                                    path, str(master)], pass_fds=(master,),
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    finally:
        os.close(master)
        os.close(slave)
    try:
        stdout, stderr = process.communicate(timeout=30)
    except subprocess.TimeoutExpired:  # 协议测试子进程超时必须判失败，并保留输出供定位。
        process.kill()
        process.communicate()
        raise SystemExit('PTY verifier timed out; not verified')
    if stderr:
        print(stderr, file=sys.stderr, end='')
    if process.returncode:
        raise SystemExit(process.returncode)
    result = json.loads(stdout.strip())
    result['compiler'] = shlex.join(compiler)
    result['sanitizers'] = ['address', 'undefined']
    result['limitations'] = ['no ROS/DDS', 'no firmware/watchdog', 'no physical 921600 line']
    print(json.dumps(result, ensure_ascii=False, indent=2))
    (build / 'protocol_result.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
