#!/usr/bin/env bash
# PROTOTYPE: runs the UART port comparison on the Nucleo-H753ZI, over
# FerroHAL's H753 links Z2 and Z3. The images to run are the arguments, by
# name without the `uart-` prefix (a, b, b-mixed, b2, b2-mixed); with none,
# the second prototype's: b b2 b2-mixed. Logs go to
# hil/logs/h753-uart-<image>.log. Exits non-zero if any run failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${H753_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$H753_PROBE
fi
images=("$@")
[ ${#images[@]} -gt 0 ] || images=(b b2 b2-mixed)
mkdir -p "$here/logs"
status=0
for v in "${images[@]}"; do
    log="$here/logs/h753-uart-$v.log"
    (cd "$here/nucleo-h753zi-uart-ab" && timeout 120 cargo run --release --bin "uart-$v") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run uart-$v: exit $code, $pass passed, $fail failed"
    grep -E "sent |slow:|refused:|long frame:|split:|unknown outcome|FAIL|WARN|panicked" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
