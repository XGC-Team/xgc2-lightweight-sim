#!/usr/bin/env bash
set -euo pipefail
owner="$(cd -- "$(dirname -- "$0")" && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
cmake -S "$owner" -B "$test_dir" -DCMAKE_BUILD_TYPE=Release \
  -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}" \
  -DEIGEN_INCLUDE="${EIGEN_INCLUDE:-/usr/include/eigen3}" \
  -DFS150_ASSET_SOURCE_ROOT="${FS150_ASSET_SOURCE_ROOT:?set owning FS150 asset source}" \
  -DLIGHTWEIGHT_TESTS=ON
cmake --build "$test_dir" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
(cd "$test_dir" && ctest --output-on-failure -R '^(lightweight_|xsim_)')
