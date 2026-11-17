#!/usr/bin/env bash
# PROTOTYPE: runs the SPI port comparison on the Nucleo-F401RE, over
# FerroHAL's link S1: the baseline spi_port, S, M with one device, M with
# four. Logs go to hil/logs/f401-spi-<variant>.log. Exits non-zero if any
# run failed. Arguments pick variants (base s m1 m4); none runs all four.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${F401_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$F401_PROBE
fi
mkdir -p "$here/logs"
status=0
variants=("$@")
[ ${#variants[@]} -eq 0 ] && variants=(base s m1 m4)
for v in "${variants[@]}"; do
    log="$here/logs/f401-spi-$v.log"
    (cd "$here/nucleo-f401re-spi-sm" && timeout 120 cargo run --release --bin "spi-$v") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run spi-$v: exit $code, $pass passed, $fail failed"
    grep -E "latency|multi-part|chip select|full:|priority:|switch:|cancel:|concurrent:| port:|FAIL|WARN|panicked" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
