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

# Lint. Warnings are errors, same as CI.
clippy:
    cargo clippy --workspace --all-targets -- -D warnings

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
