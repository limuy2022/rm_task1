#!/usr/bin/env bash
# Extra PnP comparison only; uses the running detector's same corners.
# Does not change the existing PnP's calibration or threshold.
set -e
if [ "$#" -ne 2 ] || [ ! -f "$1" ]; then
    printf '用法：bash run_reference_pnp.bash 参考标定YAML路径 ArUco_ID\n' >&2
    exit 2
fi
case "$2" in ''|*[!0-9]*) printf 'ID必须是0到49的整数。\n' >&2; exit 2;; esac
if [ "$2" -gt 49 ]; then
    printf 'DICT_4X4_50的ID范围为0到49。\n' >&2
    exit 2
fi
exec ros2 run team43_perception marker_pnp_node --ros-args \
    -r __ns:=/team43_check -r __node:=marker_pnp_reference \
    -r /perception/marker_cam:=/team43_check/reference/marker_cam \
    -p "config.virtual_yaml:=$(realpath "$1")" \
    -p "perception.target_digit:=$2" \
    -p perception.marker_corners_topic:=/team43_check/marker_corners \
    -p perception.camera_info_topic:=/team43_check/reference/camera_info_unused \
    -p perception.camera_info_source:=yaml \
    -p perception.marker_size_m:=0.10 \
    -p perception.reproj_max_px:=2.0
