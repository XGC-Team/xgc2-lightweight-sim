#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo "usage: build.sh BUILD_DIR INSTALL_PREFIX" >&2; exit 2; }
source_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
cmake -S "$source_dir" -B "$1" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$2" \
  -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}" \
  -DFS150_ASSET_SOURCE_ROOT="${FS150_ASSET_SOURCE_ROOT:?set owning FS150 asset source}" \
  -DLIGHTWEIGHT_TESTS="${LIGHTWEIGHT_TESTS:-OFF}"
cmake --build "$1" --target xsim -j "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "$1"
