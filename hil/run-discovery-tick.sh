#!/usr/bin/env bash
# Runs the STM32F4 Discovery bench: the timer_tick bench. Needs no link
# (requirements/benches.md). Exits non-zero if the run failed.
#
# The probe: DISCOVERY_PROBE (probe-rs's VID:PID:Serial selector), else
# PROBE_RS_PROBE, else the only probe attached. The log goes to
# hil/logs/discovery-timer-tick.log.

set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${DISCOVERY_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$DISCOVERY_PROBE
fi
mkdir -p "$here/logs"
log="$here/logs/discovery-timer-tick.log"
(cd "$here/stm32f4-discovery-timer-tick" && timeout 300 cargo run --release --bin timer-tick) > "$log" 2>&1
code=$?
pass=$(grep -c "PASS" "$log")
fail=$(grep -c "FAIL" "$log")
# The line FerroHAL's bench scripts print, which the rig's telemetry reads.
echo "run timer-tick: exit $code, $pass passed, $fail failed"
grep -E "FAIL|WARN|panicked" "$log"
[ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]
