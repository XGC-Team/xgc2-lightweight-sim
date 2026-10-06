#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
set -u
dpkg -s xsim >/dev/null
dpkg-query -S /opt/xgc2/xsim/bin/xsim /opt/xgc2/xsim/share/xsim/fs150_native_flight_model.json
ldd /opt/xgc2/xsim/bin/xsim | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'
/opt/xgc2/xsim/bin/xsim --help
! dpkg -L xsim | grep -E 'liblightweight_vehicle|libros_io|xgc-rt-host|simulation_records_v1|/opt/ros/'
