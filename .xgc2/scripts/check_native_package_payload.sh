#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: check_native_package_payload.sh INSTALL_ROOT DEB_DIR' >&2; exit 2; }
install_root="$1"
deb_dir="$2"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/xgc2-lightweight-sim_*.deb)
[[ ${#debs[@]} == 1 ]]
dpkg-deb --extract "${debs[0]}" "$work/payload"
WIRE_PATHS=(
  "/usr/include/xgc-lightweight-sim/simulation_records_v1.h"
  "/usr/share/cmake/XgcLightweightSimInterfaces/XgcLightweightSimInterfacesConfig.cmake"
  "/usr/share/cmake/XgcLightweightSimInterfaces/XgcLightweightSimInterfacesConfigVersion.cmake"
  "/usr/share/cmake/XgcLightweightSimInterfaces/XgcLightweightSimInterfacesTargets.cmake"
)
paths=(/usr/lib/xgc2-lightweight-sim/liblightweight_vehicle.so /usr/share/xgc2-lightweight-sim/fs150_native_flight_model.json "${WIRE_PATHS[@]}")
for path in "${paths[@]}"; do cmp "$install_root$path" "$work/payload$path"; done
for missing in "${paths[@]}"; do
  root="$work/missing-$(basename "$missing")"
  for path in "${paths[@]}"; do
    [[ "$path" == "$missing" ]] && continue
    mkdir -p "$root$(dirname "$path")"
    cp -a "$install_root$path" "$root$path"
  done
  if PACKAGE_MODE=native "$script_dir/package_debs.sh" --install-root "$root" --output-dir "$work/out" >"$work/negative.log" 2>&1; then
    echo "packager accepted missing $missing" >&2; exit 1
  fi
  grep -Fq "missing required installed payload: $missing" "$work/negative.log"
  echo "PASS: missing $missing refused"
done
echo 'PASS: real independent native Deb payload and missing-library/model controls'
