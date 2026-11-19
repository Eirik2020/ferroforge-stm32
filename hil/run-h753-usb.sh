#!/usr/bin/env bash
# PROTOTYPE: the Nucleo-H753ZI's USB port variants, deferred and polled, on
# CN13 with a data cable to the host. The probe: H753_PROBE, else
# PROBE_RS_PROBE, else the only probe attached. Arguments pick variants.
exec "$(dirname "$0")/usb-common/run.sh" nucleo-h753zi-usb-device H753_PROBE h753 "$@"
