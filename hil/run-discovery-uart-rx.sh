#!/usr/bin/env bash
# Runs the STM32F4 Discovery bench: the receive-only uart_rx_port. Needs FerroHAL's
# Discovery links D1 and D2 (requirements/benches.md). Exits non-zero if the run
# failed.
#
# The probe: DISCOVERY_PROBE (probe-rs's VID:PID:Serial selector), else
# PROBE_RS_PROBE, else the only probe attached. The log goes to
# hil/logs/discovery-uart-rx.log.

set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${DISCOVERY_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$DISCOVERY_PROBE
fi
mkdir -p "$here/logs"
log="$here/logs/discovery-uart-rx.log"
(cd "$here/stm32f4discovery-uart-rx" && timeout 300 cargo run --release --bin uart-rx) > "$log" 2>&1
code=$?
pass=$(grep -c "PASS" "$log")
fail=$(grep -c "FAIL" "$log")
# The line FerroHAL's bench scripts print, which the rig's telemetry reads.
echo "run uart-rx: exit $code, $pass passed, $fail failed"
grep -E "FAIL|WARN|panicked" "$log"
[ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]
