#!/usr/bin/env bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1
python3 ./checkenv.py
colcon build
echo "编译完成。下一步：bash \"$TRACK_WS/run_here.bash\" target_id:=0"
