#!/usr/bin/env bash
# PROTOTYPE: runs the UART port comparison on the Nucleo-F401RE, variant A
# then variant B, over FerroHAL's F401 links J1 and J2. Logs go to
# hil/logs/f401-uart-{a,b}.log. Exits non-zero if either run failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${F401_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$F401_PROBE
fi
mkdir -p "$here/logs"
status=0
for v in a b b-mixed; do
    log="$here/logs/f401-uart-$v.log"
    (cd "$here/nucleo-f401re-uart-ab" && timeout 120 cargo run --release --bin "uart-$v") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run uart-$v: exit $code, $pass passed, $fail failed"
    grep -E "sent |slow:|FAIL|WARN|panicked" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
