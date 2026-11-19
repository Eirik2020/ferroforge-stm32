#!/usr/bin/env bash
# Runs the Nucleo-H753ZI bench: the timer_tick bench. Needs no link
# (requirements/benches.md). Exits non-zero if the run failed.
#
# The probe: H753_PROBE (probe-rs's VID:PID:Serial selector), else
# PROBE_RS_PROBE, else the only probe attached. The log goes to
# hil/logs/h753-timer-tick.log.

set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${H753_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$H753_PROBE
fi
mkdir -p "$here/logs"
log="$here/logs/h753-timer-tick.log"
(cd "$here/nucleo-h753zi-timer-tick" && timeout 300 cargo run --release --bin timer-tick) > "$log" 2>&1
code=$?
pass=$(grep -c "PASS" "$log")
fail=$(grep -c "FAIL" "$log")
# The line FerroHAL's bench scripts print, which the rig's telemetry reads.
echo "run timer-tick: exit $code, $pass passed, $fail failed"
grep -E "FAIL|WARN|panicked" "$log"
[ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]
