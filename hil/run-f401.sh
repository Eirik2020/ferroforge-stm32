#!/usr/bin/env bash
# Runs the Nucleo-F401RE bench: the uart_port loopback. Needs FerroHAL's F401
# links J1 and J2 (requirements/benches.md). Exits non-zero if the run failed.
#
# The probe: F401_PROBE (probe-rs's VID:PID:Serial selector), else
# PROBE_RS_PROBE, else the only probe attached. The log goes to
# hil/logs/f401-uart-loopback.log.

set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${F401_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$F401_PROBE
fi
mkdir -p "$here/logs"
log="$here/logs/f401-uart-loopback.log"
(cd "$here/nucleo-f401re-uart-loopback" && timeout 300 cargo run --release --bin uart-loopback) > "$log" 2>&1
code=$?
pass=$(grep -c "PASS" "$log")
fail=$(grep -c "FAIL" "$log")
echo "uart-loopback: exit $code, $pass passed, $fail failed"
grep -E "FAIL|WARN|panicked" "$log"
[ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]
