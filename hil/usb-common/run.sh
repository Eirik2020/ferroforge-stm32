#!/usr/bin/env bash
# PROTOTYPE: runs both USB port variants on one board, each image with
# FerroHAL's usb_cdc_host.py alongside. Called by run-h753-usb.sh and
# run-discovery-usb.sh, which name the crate, the probe and the board.
#
#   usb-common/run.sh <crate> <chip probe variable> <serial prefix> [deferred] [polled]
#
# The host script: $FERROHAL_DIR/hil/usb_cdc_host.py, FERROHAL_DIR defaulting
# to a ferrohal-stm32 checkout beside this repository. Logs go to
# hil/logs/<prefix>-usb-<variant>.log and .log.host. Exits non-zero if any
# run failed.

set -u
here="$(cd "$(dirname "$0")/.." && pwd)"
crate=$1 probe_var=$2 prefix=$3
shift 3
probe=${!probe_var:-}
if [ -n "$probe" ]; then
    export PROBE_RS_PROBE=$probe
fi
ferrohal=${FERROHAL_DIR:-$here/../../ferrohal-stm32}
host_script="$ferrohal/hil/usb_cdc_host.py"
if [ ! -f "$host_script" ]; then
    echo "FAIL no $host_script: set FERROHAL_DIR to a ferrohal-stm32 checkout"
    exit 2
fi
mkdir -p "$here/logs"
status=0
for variant in ${*:-deferred signalled locked polled}; do
    features=()
    [ "$variant" = locked ] && features=(--features no-fifo-cs)
    log="$here/logs/$prefix-usb-$variant.log"
    : > "$log"
    (cd "$here/$crate" && timeout 300 cargo build --release --bin "usb-$variant" "${features[@]}") >> "$log" 2>&1
    code=$?
    host=""
    if [ "$code" -eq 0 ]; then
        # Built first, so the host's wait for the port is not spent compiling;
        # --fresh waits for a port the previous image left to go first.
        FERROHAL_SERIAL_GLOB="/dev/serial/by-id/*FerroForge*$prefix-usb-$variant*" \
            python3 -I "$host_script" --fresh 30 > "$log.host" 2>&1 &
        host=$!
        (cd "$here/$crate" && timeout 120 cargo run --release --bin "usb-$variant" "${features[@]}") >> "$log" 2>&1
        code=$?
    fi
    if [ -n "$host" ]; then
        wait "$host"
        host_code=$?
        cat "$log.host" >> "$log"
        [ "$host_code" -eq 0 ] || code=1
    fi
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    # The line FerroHAL's bench scripts print, which the rig's telemetry reads.
    echo "run usb-$variant: exit $code, $pass passed, $fail failed"
    grep -E "FAIL|WARN|panicked|MEASURE|KiB/s|configured after" "$log"
    if [ "$code" -ne 0 ] || [ "$fail" -ne 0 ] || [ "$pass" -eq 0 ]; then
        status=1
    fi
done
exit $status
