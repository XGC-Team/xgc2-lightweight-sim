#!/usr/bin/env bash
set -euo pipefail
library=/usr/lib/xgc2-lightweight-sim/liblightweight_vehicle.so
dpkg -s xgc2-lightweight-sim ros-noetic-xgc2-lightweight-sim-msgs >/dev/null
dpkg-query -S "$library" /usr/share/xgc2-lightweight-sim/fs150_native_flight_model.json
nm -D --defined-only "$library" | awk '$3 == "xgc_rt_plugin_v1" {found=1} END {exit !found}'
ldd "$library" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'
if readelf -d "$library" | grep -E 'NEEDED.*(ros|sss)' >/dev/null; then echo 'native model has ROS/SSS link dependency' >&2; exit 1; fi
set +u
source /opt/ros/noetic/setup.bash
set -u
test "$(rospack find xgc2_lightweight_sim_msgs)" = /opt/ros/noetic/share/xgc2_lightweight_sim_msgs
test -f /opt/ros/noetic/include/xgc2_lightweight_sim_msgs/SetProvider.h
rossrv show xgc2_lightweight_sim_msgs/SetProvider >/dev/null
