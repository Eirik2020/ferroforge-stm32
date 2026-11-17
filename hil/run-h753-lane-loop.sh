#!/usr/bin/env bash
# PROTOTYPE: runs the timer lane loopback on the Nucleo-H753ZI. Needs the
# owner's link: PC8 to PE11 through 1 kOhm (nucleo-h753zi-lane-loopback's
# src/lib.rs). The images are the arguments, as variant and lane count: t2,
# f2; with none, both. Logs go to hil/logs/h753-lane-<image>.log. Exits
# non-zero if any run failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${H753_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$H753_PROBE
fi
images=("$@")
[ ${#images[@]} -gt 0 ] || images=(t2 f2)
mkdir -p "$here/logs"
status=0
for v in "${images[@]}"; do
    variant=${v:0:1}
    log="$here/logs/h753-lane-$v.log"
    (cd "$here/nucleo-h753zi-lane-loopback" && timeout 300 cargo run --release --bin "lanes-$variant") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run lane-$v: exit $code, $pass passed, $fail failed"
    grep -E "margin:|turn-around|listen alone|member run|lock hold|exchange,|back to back|long frame|longest receive|cancel mid|copies:|FINDING|before any|FAIL|WARN|panicked" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
