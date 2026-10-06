#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0}"
work="${WORK_DIR:-$root/.work/docker}";out="${OUTPUT_DIR:-$root/debs}";sensor="${XSIM_SENSOR_SOURCE_ROOT:-}"
while [[ $# -gt 0 ]];do case "$1" in --work-dir)work="$2";shift 2;;--output-dir)out="$2";shift 2;;--sensor-source)sensor="$2";shift 2;;*)echo "unknown argument: $1" >&2;exit 2;;esac;done
[[ -f "$sensor/xgc2_world_lidar/library/CMakeLists.txt" ]] || { echo 'an explicit owning convex_geometry source is required' >&2;exit 2; }
mkdir -p "$work" "$out"
docker run --rm --cpus 2 --pids-limit 256 -e DEBIAN_FRONTEND=noninteractive -v "$root:/source:ro" -v "$(realpath "$sensor"):/sensors:ro" -v "$(realpath "$out"):/output" "$image" bash -c '
set -euo pipefail
# System development packages belong to the selected XGC2 build image.
# Fail closed if the image lacks them; product CI does not bootstrap toolchains.
dpkg-query -W libglfw3-dev libglm-dev libyaml-cpp-dev nlohmann-json3-dev >/dev/null
# Math/neutral public record development packages are the only XGC dependencies.
install -d -m0755 /etc/apt/keyrings
curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
gpg --batch --show-keys --with-colons /etc/apt/keyrings/xgc2-archive-keyring.gpg | awk -F: '''$1=="fpr"{print $10}''' | grep -Fxq 2A8E11B36F56D307ADF626D85E5FDC30979EA43F
echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" >/etc/apt/sources.list.d/xgc2.list
apt-get update
apt-get install -y --no-install-recommends libxgc2-math-dev libxgc2-robotics-interfaces-dev
python3 /source/.xgc2/scripts/check_build_inputs.py
bash /source/.xgc2/scripts/build_package.sh /sensors /source/.xgc2/build-inputs/fs150 /output
apt-get install -y /output/xsim_*.deb
bash /source/.xgc2/scripts/check_installed_packages.sh
bash /source/.xgc2/scripts/check_native_package_payload.sh /output /output
'
