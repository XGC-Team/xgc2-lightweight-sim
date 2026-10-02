#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0}"
work="${WORK_DIR:-$root/.work/docker}"
output="${OUTPUT_DIR:-$root/debs}"
interface_deb=""
interfaces_only=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir) work="$2"; shift 2;;
    --output-dir) output="$2"; shift 2;;
    --interface-deb) interface_deb="$2"; shift 2;;
    --interfaces-only) interfaces_only=true; shift;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
mkdir -p "$work" "$output"
work="$(cd "$work" && pwd)"
output="$(cd "$output" && pwd)"
mounts=()
if [[ -n "$interface_deb" ]]; then
  test -f "$interface_deb"
  interface_deb="$(realpath "$interface_deb")"
  mounts=(-v "$interface_deb:/workspace/interface.deb:ro")
fi
# All build inputs are owning checkout bytes, image packages or signed APT.
# No sibling source/workspace prefix is mounted into this leaf build.
docker pull "$image"
docker run --rm \
  -e INTERFACES_ONLY="$interfaces_only" \
  -e SOURCE_DATE_EPOCH="$(git -C "$root" log -1 --format=%ct)" \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -v "$root:/workspace/source:ro" \
  -v "$work:/workspace/work" \
  -v "$output:/workspace/out" \
  "${mounts[@]}" "$image" bash -lc '
    set -euo pipefail
    export DEBIAN_FRONTEND=noninteractive
    for tool in cmake c++ dpkg-deb dpkg-shlibdeps rsync file nm curl gpg; do
      command -v "$tool" >/dev/null || { echo "build image missing $tool" >&2; exit 1; }
    done
    source=/workspace/source
    python3 "$source/.xgc2/scripts/check_build_inputs.py"
    version="$(awk -F": *" "/^version:/ {print \$2; exit}" "$source/.xgc2/product.yml")"
    if [[ -f /workspace/interface.deb ]]; then
      test "$(dpkg-deb -f /workspace/interface.deb Package)" = ros-noetic-xgc2-lightweight-sim-msgs
      test "$(dpkg-deb -f /workspace/interface.deb Version)" = "$version"
      test "$(dpkg-deb -f /workspace/interface.deb Architecture)" = all
      cp /workspace/interface.deb "/workspace/out/ros-noetic-xgc2-lightweight-sim-msgs_${version}_all.deb"
    else
      mkdir -p /workspace/work/interfaces/src
      rsync -a "$source/src/xgc2_lightweight_sim_msgs/" /workspace/work/interfaces/src/xgc2_lightweight_sim_msgs/
      set +u
      . /opt/ros/noetic/setup.bash
      set -u
      cd /workspace/work/interfaces
      DESTDIR=/workspace/work/interfaces-install catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic
      PACKAGE_MODE=interfaces "$source/.xgc2/scripts/package_debs.sh" \
        --install-root /workspace/work/interfaces-install --output-dir /workspace/out
    fi
    if [[ "$INTERFACES_ONLY" == true ]]; then exit 0; fi
    install -d -m 0755 /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    gpg --batch --show-keys --with-colons /etc/apt/keyrings/xgc2-archive-keyring.gpg | \
      awk -F: '\''$1 == "fpr" {print $10}'\'' | grep -Fxq 2A8E11B36F56D307ADF626D85E5FDC30979EA43F
    echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" >/etc/apt/sources.list.d/xgc2.list
    if [[ -n "$XGC2_APT_OVERLAY_URL" ]]; then
      echo "deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] ${XGC2_APT_OVERLAY_URL%/} focal main" >/etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt-get update
    apt-get install -y --no-install-recommends libxgc-runtime-sdk-dev libxgc2-math-dev
    test -f /usr/include/xgc-runtime/xgc_rt.h
    test -f /usr/share/cmake/XgcRuntimeSDK/XgcRuntimeSDKConfig.cmake
    dpkg-query -S /usr/include/xgc-runtime/xgc_rt.h /usr/share/cmake/XgcRuntimeSDK/XgcRuntimeSDKConfig.cmake
    dpkg --compare-versions "$(dpkg-query -W -f="\${Version}" libxgc-runtime-sdk-dev)" ge 0.1.0-1~focal
    cmake -S "$source" -B /workspace/work/native \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr \
      -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_DATADIR=share \
      -DXGC_RUNTIME_SDK_SOURCE_ROOT= -DXGC2_MATH_INCLUDE=/usr/include \
      -DFS150_ASSET_SOURCE_ROOT="$source/.xgc2/build-inputs/fs150" \
      -DLIGHTWEIGHT_TESTS=ON -DLIGHTWEIGHT_CONTROLLER_TEST=OFF
    cmake --build /workspace/work/native -j "$(nproc)"
    (cd /workspace/work/native/src/xgc2_lightweight_sim && ctest --output-on-failure)
    DESTDIR=/workspace/work/native-install cmake --install /workspace/work/native
    PACKAGE_MODE=native "$source/.xgc2/scripts/package_debs.sh" \
      --install-root /workspace/work/native-install --output-dir /workspace/out
    "$source/.xgc2/scripts/check_native_package_payload.sh" /workspace/work/native-install /workspace/out
    apt-get install -y /workspace/out/*.deb
    "$source/.xgc2/scripts/check_installed_packages.sh"
  '
