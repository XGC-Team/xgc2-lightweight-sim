#!/usr/bin/env bash
# Exercise installed plant/controller/HTE artifacts; never rebuild a Runtime domain copy.
set -euo pipefail
[[ $# -ge 3 ]] || { echo "usage: $0 INSTALLED_PLANT.so CTL.so HTE.so [fixture options]" >&2; exit 2; }
plant="$1"; controller="$2"; hte="$3"; shift 3
for library in "$plant" "$controller" "$hte"; do [[ -f "$library" ]] || { echo "missing installed artifact: $library" >&2; exit 2; }; done
owner="$(cd -- "$(dirname -- "$0")" && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
cmake -S "$owner" -B "$test_dir" -DCMAKE_BUILD_TYPE=Release \
  -DXGC_RUNTIME_SDK_SOURCE_ROOT="${XGC_RUNTIME_SDK_SOURCE_ROOT:-}" \
  -DFS150_ASSET_SOURCE_ROOT="${FS150_ASSET_SOURCE_ROOT:?set owning FS150 asset source}" \
  -DLIGHTWEIGHT_CONTROLLER_TEST=ON
cmake --build "$test_dir" --target controller_test -j "${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
"$test_dir/controller_test" "$plant" "$controller" --hte "$hte" "$@"
