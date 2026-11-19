#!/usr/bin/env bash
# The lane frame port and FerroHAL's lanes on the Nucleo-H753ZI, in one layout
# (nucleo-h753zi-lane-port's src/lib.rs states the wiring). The first argument is the layout:
# reader-ahead, own-streams. The rest are the images: f (the frame port) and t (FerroHAL's
# lanes under RTIC with no port); with none, both. LANE_FEATURES adds
# features (profile, profile-start). Logs go to
# hil/logs/h753-lane-port-<layout>-<image>.log. Exits non-zero if any run
# failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
if [ -n "${H753_PROBE:-}" ]; then
    export PROBE_RS_PROBE=$H753_PROBE
fi
layout=${1:-}
case "$layout" in
    reader-ahead|own-streams) ;;
    *) echo "usage: $0 <reader-ahead|own-streams> [f] [t]"; exit 2 ;;
esac
shift
images=("$@")
[ ${#images[@]} -gt 0 ] || images=(f t)
mkdir -p "$here/logs"
status=0
for v in "${images[@]}"; do
    log="$here/logs/h753-lane-port-$layout-$v.log"
    (cd "$here/nucleo-h753zi-lane-port" && timeout 300 cargo run --release --no-default-features \
        --features "$layout ${LANE_FEATURES:-}" --bin "lanes-$v") > "$log" 2>&1
    code=$?
    pass=$(grep -c "PASS" "$log")
    fail=$(grep -c "FAIL" "$log")
    echo "run $layout-$v: exit $code, $pass passed, $fail failed"
    grep -E "margin:|turn-around|listen alone|member run|submit \(F\)|lock hold|exchange,|back to back|long frame|longest receive|cancel mid|copies:|start whole|before any|FAIL|WARN|panicked|^.*profile" "$log"
    { [ "$code" -eq 0 ] && [ "$fail" -eq 0 ] && [ "$pass" -gt 0 ]; } || status=1
done
exit $status
