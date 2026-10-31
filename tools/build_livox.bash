#!/usr/bin/env bash
# 从仓库固定归档构建Livox，源码与产物均保留在本队目录；不安装系统库。
set -eo pipefail
TRACK_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
LIVOX_WS="$TRACK_WS/.livox"

python3 - "$TRACK_WS" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
archive = root / 'archives/livox_ros_driver2.tar.gz'
expected = json.loads((root / 'archives/livox_ros_driver2.source.json').read_text())['archive_sha256']
if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
    raise SystemExit('Livox归档校验失败，请使用仓库配套归档')
PY

mkdir -p "$LIVOX_WS/src"
if [[ ! -d "$LIVOX_WS/src/livox_ros_driver2" ]]; then
    tar -xzf "$TRACK_WS/archives/livox_ros_driver2.tar.gz" -C "$LIVOX_WS/src"
fi
source /opt/ros/humble/setup.bash
colcon --log-base "$LIVOX_WS/log" build \
    --base-paths "$LIVOX_WS/src/livox_ros_driver2" \
    --build-base "$LIVOX_WS/build" --install-base "$LIVOX_WS/install" \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
python3 - "$LIVOX_WS" <<'PY'
from pathlib import Path
import platform
import shlex
import sys

workspace = Path(sys.argv[1])
architecture = {'x86_64': 'amd64', 'aarch64': 'arm64'}[platform.machine()]
sdk = workspace / 'src/livox_ros_driver2/Livox-SDK2/lib' / architecture
environment = workspace / 'env.bash'
environment.write_text(
    '# 本队Livox安装空间与SDK库，不修改系统环境文件。\n'
    f'source {shlex.quote(str(workspace / "install/setup.bash"))}\n'
    f'export LD_LIBRARY_PATH={shlex.quote(str(sdk))}"${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}"\n'
    'export ROS_LOCALHOST_ONLY=1\n')
print(f'Livox构建完成。各新终端先加载ROS，再执行：\nsource "{environment}"')
PY
