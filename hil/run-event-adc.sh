#!/usr/bin/env bash
# PROTOTYPE: runs the event port and ADC port benches. The first argument is
# the board, `discovery` (STM32F4 Discovery, links E1 and A1 of
# stm32f4-discovery-event-adc's src/lib.rs; DISCOVERY_PROBE picks the probe)
# or `h753` (Nucleo-H753ZI, FerroHAL's links Z1 and Z10; H753_PROBE). The
# rest are images; with none, every one:
#
#   e                 E, exti_line
#   k1000 k10000 ...  K, timer_tick at that many hertz (above 10 kHz: the
#                     highest-rate sweep, quiet only)
#   a                 A, adc_scan
#   p  p16            P, adc_event_port, three or sixteen channels
#   tp1000 ...        the tick kicking P at that many hertz
#
# Logs go to hil/logs/<board>-event-adc-<image>.log. Exits non-zero if any
# run failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
board=${1:-}
shift || true
case "$board" in
    discovery) crate=stm32f4-discovery-event-adc; probe=${DISCOVERY_PROBE:-} ;;
    h753) crate=nucleo-h753zi-event-adc; probe=${H753_PROBE:-} ;;
    *) echo "usage: $0 discovery|h753 [image...]" >&2; exit 2 ;;
esac
[ -z "$probe" ] || export PROBE_RS_PROBE=$probe
images=("$@")
[ ${#images[@]} -gt 0 ] || images=(e k1000 k10000 k20000 k50000 k100000 k200000 a p p16 tp1000 tp10000 tp12500 tp16000 tp20000 tp25000)
mkdir -p "$here/logs"
status=0
for image in "${images[@]}"; do
    features=()
    hz=1000
    case "$image" in
        e) bin=events-e ;;
        k*) bin=events-k; hz=${image#k} ;;
        a) bin=adc-a ;;
        p) bin=adc-p ;;
        p16) bin=adc-p; features=(--features ch16) ;;
        tp*) bin=adc-tick; hz=${image#tp} ;;
        *) echo "unknown image $image" >&2; status=1; continue ;;
    esac
    log="$here/logs/$board-event-adc-$image.log"
    (cd "$here/$crate" && TICK_HZ=$hz timeout 300 cargo run --release "${features[@]}" --bin "$bin") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run event-adc-$image: exit $code, $pass passed, $fail failed"
    grep -E "stamp:|least|spread|took|driven|sequences|lost result|late kick|moves|highest|FAIL|WARN|panicked" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
