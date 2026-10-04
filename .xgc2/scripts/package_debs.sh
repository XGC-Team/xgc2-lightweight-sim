#!/usr/bin/env bash
set -euo pipefail
[[ $# == 4 && "$1" == --install-root && "$3" == --output-dir ]] || { echo 'usage: package_debs.sh --install-root ROOT --output-dir DIR' >&2; exit 2; }
install_root="$2"
output="$4"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
version="$(awk -F': *' '/^version:/ {print $2; exit}' "$root/.xgc2/product.yml")"
arch="$(dpkg --print-architecture)"
mode="${PACKAGE_MODE:-all}"
case "$mode" in all|native|interfaces) ;; *) echo 'invalid PACKAGE_MODE' >&2; exit 2;; esac
native_library=/usr/lib/xgc2-lightweight-sim/liblightweight_vehicle.so
model=/usr/share/xgc2-lightweight-sim/fs150_native_flight_model.json
interface=/opt/ros/noetic/share/xgc2_lightweight_sim_msgs/srv/SetProvider.srv
header=/opt/ros/noetic/include/xgc2_lightweight_sim_msgs/SetProvider.h
required=()
WIRE_PATHS=(
  "/usr/include/xgc-lightweight-sim/simulation_records_v1.h"
  "/usr/share/cmake/XgcLightweightSimInterfaces/XgcLightweightSimInterfacesConfig.cmake"
  "/usr/share/cmake/XgcLightweightSimInterfaces/XgcLightweightSimInterfacesConfigVersion.cmake"
  "/usr/share/cmake/XgcLightweightSimInterfaces/XgcLightweightSimInterfacesTargets.cmake"
)
[[ "$mode" == interfaces ]] || required+=("$native_library" "$model" "${WIRE_PATHS[@]}")
[[ "$mode" == native ]] || required+=("$interface" "$header"
  /opt/ros/noetic/share/xgc2_lightweight_sim_msgs/msg/NamedPose.msg
  /opt/ros/noetic/share/xgc2_lightweight_sim_msgs/msg/PartitionPoses.msg
  /opt/ros/noetic/include/xgc2_lightweight_sim_msgs/NamedPose.h
  /opt/ros/noetic/include/xgc2_lightweight_sim_msgs/PartitionPoses.h)
for path in "${required[@]}"; do
  [[ -f "$install_root$path" ]] || { echo "missing required installed payload: $path" >&2; exit 1; }
done
if [[ "$mode" != interfaces ]]; then
  file -b "$install_root$native_library" | grep -q '^ELF'
  nm -D --defined-only "$install_root$native_library" | awk '$3 == "xgc_rt_plugin_v1" {found=1} END {exit !found}'
  python3 - "$install_root$model" "$root/.xgc2/build-inputs/fs150.lock.json" <<'PY_ASSET'
import json, sys
asset, lock = (json.load(open(p)) for p in sys.argv[1:])
assert asset['sources']['sdf']['sha256'] == lock['files']['models/fs150/iris.sdf']['sha256']
assert asset['sources']['parameters']['sha256'] == lock['files']['config/generated/fs150-sitl.params']['sha256']
PY_ASSET
fi
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
mkdir -p "$output"
if [[ "$mode" != interfaces ]]; then
  pkg="$work/xgc2-lightweight-sim"
  mkdir -p "$pkg/usr/lib/xgc2-lightweight-sim" "$pkg/usr/share/xgc2-lightweight-sim" "$pkg/DEBIAN" "$work/debian"
  cp -a "$install_root$native_library" "$pkg$native_library"
  cp -a "$install_root$model" "$pkg$model"
  for path in "${WIRE_PATHS[@]}"; do
    mkdir -p "$pkg$(dirname "$path")"
    cp -a "$install_root$path" "$pkg$path"
  done
  cp "$root/.xgc2/build-inputs/fs150.lock.json" "$pkg/usr/share/xgc2-lightweight-sim/fs150-build-inputs.json"
  printf 'Source: xgc2-lightweight-sim\nSection: science\nPriority: optional\nMaintainer: XGC2 <867768510@qq.com>\n\nPackage: xgc2-lightweight-sim\nArchitecture: any\n' >"$work/debian/control"
  depends="$(cd "$work" && dpkg-shlibdeps -O -e"$pkg$native_library")"
  depends="${depends#shlibs:Depends=}"
  [[ -n "$depends" && "$depends" != *sss* && "$depends" != *ros-noetic* ]] || { echo 'invalid native runtime dependency closure' >&2; exit 1; }
  printf 'Package: xgc2-lightweight-sim\nVersion: %s\nArchitecture: %s\nSection: science\nPriority: optional\nMaintainer: XGC2 <867768510@qq.com>\nDepends: %s\nDescription: Independent XGC2 native rigid-body and FCU simulator\n' "$version" "$arch" "$depends" >"$pkg/DEBIAN/control"
  dpkg-deb --root-owner-group --build "$pkg" "$output/xgc2-lightweight-sim_${version}_${arch}.deb" >/dev/null
fi
if [[ "$mode" != native ]]; then
  name=ros-noetic-xgc2-lightweight-sim-msgs
  pkg="$work/$name"
  mkdir -p "$pkg/DEBIAN"
  for path in /opt/ros/noetic/share/xgc2_lightweight_sim_msgs /opt/ros/noetic/include/xgc2_lightweight_sim_msgs /opt/ros/noetic/lib/python3/dist-packages/xgc2_lightweight_sim_msgs /opt/ros/noetic/share/common-lisp/ros/xgc2_lightweight_sim_msgs /opt/ros/noetic/share/gennodejs/ros/xgc2_lightweight_sim_msgs /opt/ros/noetic/share/roseus/ros/xgc2_lightweight_sim_msgs; do
    if [[ -e "$install_root$path" ]]; then
      mkdir -p "$pkg$(dirname "$path")"
      cp -a "$install_root$path" "$pkg$path"
    fi
  done
  printf 'Package: %s\nVersion: %s\nArchitecture: all\nSection: misc\nPriority: optional\nMaintainer: XGC2 <867768510@qq.com>\nDepends: ros-noetic-message-runtime, ros-noetic-std-msgs, ros-noetic-geometry-msgs\nDescription: XGC2 lightweight simulator lifecycle and named-pose interface\n' "$name" "$version" >"$pkg/DEBIAN/control"
  dpkg-deb --root-owner-group --build "$pkg" "$output/${name}_${version}_all.deb" >/dev/null
fi
