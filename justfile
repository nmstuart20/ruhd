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

# Fetch the pinned UHD reference checkout (see NOTES.md).
reference:
    git submodule update --init --depth 1 reference/uhd

# Grep the UHD reference sources, e.g. `just grep-ref ad9361_set_gain`.
grep-ref PATTERN:
    @git -C reference/uhd grep -n -- "{{PATTERN}}" -- 'host/**' || true

# Remove build artifacts.
clean:
    cargo clean
