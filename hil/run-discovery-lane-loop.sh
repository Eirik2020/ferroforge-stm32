#!/usr/bin/env bash
# PROTOTYPE: runs the timer lane loopback on the STM32F4 Discovery. Needs the
# owner's link: PC8 (P2-45) to PE11 (P1-29) through 1 kOhm
# (stm32f4-discovery-lane-loopback's src/lib.rs). The images are the
# arguments, as variant and lane count: t1, t2, t4, f1, f2, f4; with none,
# all six. Logs go to
# hil/logs/discovery-lane-<image>.log. Exits non-zero if any run failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${DISCOVERY_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$DISCOVERY_PROBE
fi
images=("$@")
[ ${#images[@]} -gt 0 ] || images=(t1 f1 t2 f2 t4 f4)
mkdir -p "$here/logs"
status=0
for v in "${images[@]}"; do
    variant=${v:0:1}
    lanes=${v:1}
    log="$here/logs/discovery-lane-$v.log"
    (cd "$here/stm32f4-discovery-lane-loopback" && timeout 300 cargo run --release \
        --no-default-features --features "lanes$lanes" --bin "lanes-$variant") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run lane-$v: exit $code, $pass passed, $fail failed"
    grep -E "margin:|turn-around|listen alone|member run|lock hold|exchange,|back to back|long frame|longest receive|cancel mid|copies:|FINDING|before any|FAIL|WARN|panicked" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
