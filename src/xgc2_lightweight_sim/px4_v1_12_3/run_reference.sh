#!/usr/bin/env bash
# No network, clone, firmware scheduling, wrapper or live station. One CPU.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
plugin="$(dirname -- "$root")"
work="${1:-$(mktemp -d /tmp/xgc-px4-reference-XXXXXX)}"
mkdir -p "$work"
work="$(cd -- "$work" && pwd)"
python3 "$root/verify_sources.py"
docker run --rm --pull never --network none --cpus 1 --entrypoint bash \
  --mount "type=bind,src=$plugin,dst=/plugin,readonly" \
  --mount "type=bind,src=$work,dst=/evidence" \
  ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0 -c '
    set -euo pipefail
    python3 /plugin/px4_v1_12_3/upstream/src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py \
      -f /plugin/px4_v1_12_3/upstream/src/lib/mixer/MultirotorMixer/geometries/quad_x.toml \
      --normalize -o /evidence/generated-quad-x.h
    cmp /plugin/px4_v1_12_3/include/mixer_multirotor_normalized.generated.h /evidence/generated-quad-x.h
    cmake -S /plugin/px4_v1_12_3 -B /evidence/build -DCMAKE_BUILD_TYPE=Release -DPX4_REFERENCE_BUILD_TESTS=ON
    cmake --build /evidence/build -j1
    cd /evidence/build
    ctest --output-on-failure
    ./px4_reference_test --trace /evidence/reference.jsonl
  ' > "$work/reference.log" 2>&1
printf 'PX4 original reference passed; library and history trace: %s\n' "$work"
