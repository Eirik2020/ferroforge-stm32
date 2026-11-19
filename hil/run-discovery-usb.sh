#!/usr/bin/env bash
# PROTOTYPE: the STM32F4 Discovery's USB port variants, deferred and polled,
# on CN5 with a data cable to the host. The probe: DISCOVERY_PROBE, else
# PROBE_RS_PROBE, else the only probe attached. Arguments pick variants.
exec "$(dirname "$0")/usb-common/run.sh" stm32f4-discovery-usb-device DISCOVERY_PROBE f407 "$@"
