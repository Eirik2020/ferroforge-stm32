#!/usr/bin/env bash
# Runs the STM32F4 Discovery bench: usb_device on CN5, cabled to the host, nothing
# to fit (requirements/benches.md). FerroHAL's hil/usb_cdc_host.py runs beside the
# firmware, from FERROHAL_DIR (a FerroHAL checkout at the revision this crate
# pins), by default the ferrohal-stm32 checkout beside this repository. Exits
# non-zero if the run failed.
#
# The probe: DISCOVERY_PROBE (probe-rs's VID:PID:Serial selector), else
# PROBE_RS_PROBE, else the only probe attached. The log goes to
# hil/logs/discovery-usb-device.log.

set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${DISCOVERY_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$DISCOVERY_PROBE
fi
host_script="${FERROHAL_DIR:-$here/../../ferrohal-stm32}/hil/usb_cdc_host.py"
if [ ! -f "$host_script" ]; then
    echo "FAIL no $host_script: set FERROHAL_DIR to a ferrohal-stm32 checkout"
    exit 1
fi
mkdir -p "$here/logs"
log="$here/logs/discovery-usb-device.log"
crate="$here/stm32f4-discovery-usb-device"
# Built first, so the host's wait for the port is not spent compiling;
# --fresh waits for a port a previous image left to go first.
(cd "$crate" && timeout 300 cargo build --release --bin usb-device) > "$log" 2>&1
code=$?
if [ "$code" -eq 0 ]; then
    env FERROHAL_SERIAL_GLOB="/dev/serial/by-id/*FerroForge*f407-usb-device*" \
        python3 -I "$host_script" --fresh 30 > "$log.host" 2>&1 &
    host=$!
    (cd "$crate" && timeout 300 cargo run --release --bin usb-device) >> "$log" 2>&1 || code=$?
    wait "$host" || code=1
    cat "$log.host" >> "$log"
    rm -f "$log.host"
fi
pass=$(grep -c "PASS" "$log")
fail=$(grep -c "FAIL" "$log")
# The line FerroHAL's bench scripts print, which the rig's telemetry reads.
echo "run usb-device: exit $code, $pass passed, $fail failed"
grep -E "FAIL|WARN|panicked|MEASURE|KiB/s" "$log"
[ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]
