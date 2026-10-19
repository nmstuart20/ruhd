# ruhd task runner. Run `just` for the list.

default:
    @just --list

# Compile every crate in the workspace.
build:
    cargo build --workspace --all-targets

# Compile with optimisations. Sample-rate work is unusable in a debug build.
build-release:
    cargo build --workspace --release

# Run every test, including doc tests.
test:
    cargo test --workspace

# Just the compat negotiation tests: the shims and the policy matrix.
test-compat:
    cargo test -p uhd-b200 --lib -- compat:: policy:: --nocapture

# The codec bring-up gate: reach the AD936x over SPI and read its product ID.
# Needs a board with firmware and an FPGA image loaded (`just cli init`), but
# not an initialised codec — see crates/uhd-b200/examples/codec_id.rs.
#
#   just codec-id [SERIAL]
codec-id *ARGS:
    cargo run -p uhd-b200 --example codec_id -- {{ARGS}}

# Run the hardware compat tests against an attached B205mini, for one UHD
# release. Needs a board; see crates/uhd-b200/tests/hw_compat.rs.
#
#   just hw-test v4.10.0.0
hw-test RELEASE:
    #!/usr/bin/env bash
    set -euo pipefail
    cache=$(mktemp -d)
    export UHD_IMAGES_CACHE_DIR="$cache"
    export UHD_IMAGES_LOCKFILE="$cache/uhd-images.lock"
    export RUHD_HW_RELEASE="{{RELEASE}}"
    cargo run --bin ruhd -- images sync --device b205mini --release "{{RELEASE}}"
    cargo run --bin ruhd -- reset || true
    cargo run --bin ruhd -- init --no-fpga
    cargo test -p uhd-b200 --features hw-tests -- --test-threads 1 --nocapture

# Both legs of the hardware matrix CI runs, back to back.
hw-matrix: (hw-test "v4.6.0.0") (hw-test "v4.10.0.0")

# Phase 3 acceptance gate: initialize a B205mini with stock UHD and RUHD at
# matched settings, dump all AD9364 registers after each, and reject every
# undocumented bit difference. Extra arguments are passed to the harness, e.g.
# `just register-gate --serial 3168B41 --rate 30720000`.
register-gate *ARGS:
    tools/validate_ad9364_registers.py {{ARGS}}

# Lint. Warnings are errors, same as CI.
clippy:
    cargo clippy --workspace --all-targets -- -D warnings
    cargo clippy -p uhd-b200 --features hw-tests --all-targets -- -D warnings

# Rewrite source to rustfmt's liking.
fmt:
    cargo fmt --all

# Check formatting without rewriting, same as CI.
fmt-check:
    cargo fmt --all --check

# Build the API docs, treating rustdoc warnings as errors.
doc:
    RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps

# Build and open the API docs.
doc-open:
    RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --open

# Everything CI runs, in CI's order. Run this before pushing.
ci: fmt-check clippy test doc

# Run the CLI, e.g. `just cli find` or `just cli probe usb`.
cli *ARGS:
    cargo run --bin ruhd -- {{ARGS}}

# Record IQ to a file (release build), e.g. `just rx --freq 98e6 --duration 2 --file fm.sc16`.
rx *ARGS:
    cargo run --release --bin ruhd -- rx {{ARGS}}

# Transmit a raw IQ file, e.g. `just tx --freq 915e6 --rate 1e6 --file burst.sc16`.
tx *ARGS:
    cargo run --release --bin ruhd -- tx {{ARGS}}

# Receive throughput benchmark (release build), e.g. `just bench --rx-rate 61.44e6 --duration 10`.
bench *ARGS:
    cargo run --release --bin ruhd -- benchmark {{ARGS}}

# Let B2xx devices open without sudo (Linux; asks for sudo, then replug). See README.
udev-install: (_udev-install "60-ruhd-b2xx.rules")

# Also allow blank boards in the FX3 bootloader. Opt-in: matches any FX3 device.
udev-install-bootloader: (_udev-install "61-ruhd-fx3-bootloader.rules")

# Remove the udev rules installed by udev-install and udev-install-bootloader.
udev-uninstall:
    sudo rm -f /etc/udev/rules.d/60-ruhd-b2xx.rules /etc/udev/rules.d/61-ruhd-fx3-bootloader.rules
    sudo udevadm control --reload-rules

_udev-install FILE:
    #!/usr/bin/env bash
    set -euo pipefail
    sudo groupadd --system --force usrp
    sudo install -m 0644 "udev/{{FILE}}" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=usb
    echo "installed /etc/udev/rules.d/{{FILE}}"
    echo "now replug the board, then check with: just cli probe usb"
    if ! id -nG | tr ' ' '\n' | grep -qx usrp; then
        echo "using this machine over SSH or headless? also run:"
        echo "  sudo usermod -aG usrp $USER   # then log in again"
    fi

# Fetch the pinned UHD reference checkout (see NOTES.md).
reference:
    git submodule update --init --depth 1 reference/uhd

# Grep the UHD reference sources, e.g. `just grep-ref ad9361_set_gain`.
grep-ref PATTERN:
    @git -C reference/uhd grep -n -- "{{PATTERN}}" -- 'host/**' || true

# Remove build artifacts.
clean:
    cargo clean
