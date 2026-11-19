#!/usr/bin/env bash
# Every check that needs no hardware. Exits non-zero if any fails. The benches
# are hil/run-*.sh.
#
# - formatting (rustfmt), the crate and the bench;
# - clippy and rustdoc on the crate, per chip, with warnings as errors: one chip
#   per USART IP version (f401re: usart_v2, h753zi: usart_v4);
# - the host tests, per chip;
# - every bench image, built for its chip;
# - reqtool validate, when reqtool is installed (CI installs it).

set -u
root="$(cd "$(dirname "$0")/.." && pwd)"
chips=(stm32f401re stm32h753zi)
host=$(rustc -vV | sed -n 's/^host: //p')
status=0

step() {
    local name=$1
    shift
    local output
    if output=$("$@" 2>&1); then
        echo "pass  $name"
    else
        echo "FAIL  $name"
        printf '%s\n' "$output" >&2
        status=1
    fi
}

step "rustfmt" cargo fmt --manifest-path "$root/Cargo.toml" --check
for chip in "${chips[@]}"; do
    step "clippy ($chip, target)" \
        cargo clippy --manifest-path "$root/Cargo.toml" --features "$chip,usb" --target thumbv7em-none-eabihf -- -D warnings
    step "clippy ($chip, host)" \
        cargo clippy --manifest-path "$root/Cargo.toml" --features "$chip,usb" --target "$host" --all-targets -- -D warnings
    step "rustdoc ($chip)" \
        env RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --manifest-path "$root/Cargo.toml" --features "$chip,usb" --target thumbv7em-none-eabihf
    step "host tests ($chip)" \
        cargo test --manifest-path "$root/Cargo.toml" --features "$chip,usb" --target "$host"
done
for bench in "$root"/hil/*/; do
    # Shared bench sources (hil/lane-common) and the ignored logs are no crate.
    [ -f "$bench/Cargo.toml" ] || continue
    name=$(basename "$bench")
    # hil/common holds checks the bench crates include, not a crate.
    [ -f "$bench/Cargo.toml" ] || continue
    # From the bench's own directory, so its .cargo/config.toml picks the
    # target and the linker scripts.
    step "bench image ($name)" bash -c 'cd "$1" && cargo build --release --bins' _ "$bench"
done
if command -v reqtool > /dev/null; then
    step "reqtool validate" reqtool validate --repository "$root"
else
    echo "skip  reqtool validate (reqtool not installed)"
fi
exit $status
