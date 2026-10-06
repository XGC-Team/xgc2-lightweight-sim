#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.2@sha256:8b5059ac3ddab9355899f0e79783f2596ac9726e6bb108c055c74d0bc6c3da42}"
work="${WORK_DIR:-$root/.work/docker}";out="${OUTPUT_DIR:-$root/debs}";sensor="${XSIM_SENSOR_SOURCE_ROOT:-}"
while [[ $# -gt 0 ]];do case "$1" in --work-dir)work="$2";shift 2;;--output-dir)out="$2";shift 2;;--sensor-source)sensor="$2";shift 2;;*)echo "unknown argument: $1" >&2;exit 2;;esac;done
[[ -f "$sensor/xgc2_world_lidar/library/CMakeLists.txt" ]] || { echo 'an explicit owning convex_geometry source is required' >&2;exit 2; }
mkdir -p "$work" "$out"
container_name="xgc2-xsim-build-$(date +%s)-$$"
container_created=false
cleanup() {
  if [[ "$container_created" == true ]]; then
    docker rm -f "$container_name" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# The container installs the declared XGC dependencies internally.
# Only source inputs are mounted; docker cp leaves host output owned by the caller.
docker create --name "$container_name" -i --cpus 2 --pids-limit 256 \
  -e DEBIAN_FRONTEND=noninteractive -v "$root:/source:ro" \
  -v "$(realpath "$sensor"):/sensors:ro" "$image" bash -s >/dev/null
container_created=true
docker start -ai "$container_name" <<'XSIM_BUILD'
set -euo pipefail
# System development packages belong to the selected XGC2 build image.
# Fail closed if the image lacks them; product CI does not bootstrap toolchains.
dpkg-query -W libglfw3-dev libglm-dev libyaml-cpp-dev nlohmann-json3-dev >/dev/null
# Math/neutral public record development packages are the only XGC dependencies.
install -d -m0755 /etc/apt/keyrings
curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
gpg --batch --show-keys --with-colons /etc/apt/keyrings/xgc2-archive-keyring.gpg | awk -F: '$1=="fpr"{print $10}' | grep -Fxq 2A8E11B36F56D307ADF626D85E5FDC30979EA43F
echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" >/etc/apt/sources.list.d/xgc2.list
apt-get update
apt-get install -y --no-install-recommends libxgc2-math-dev libxgc2-robotics-interfaces-dev
python3 /source/.xgc2/scripts/check_build_inputs.py
bash /source/.xgc2/scripts/build_package.sh /sensors /source/.xgc2/build-inputs/fs150 /output
apt-get install -y /output/xsim_*.deb
bash /source/.xgc2/scripts/check_installed_packages.sh
bash /source/.xgc2/scripts/check_native_package_payload.sh /output /output
XSIM_BUILD
docker cp "$container_name:/output/." "$(realpath "$out")/"
