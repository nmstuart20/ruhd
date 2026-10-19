# ruhd

A pure-Rust reimplementation of the [Ettus USRP Hardware Driver](https://github.com/EttusResearch/uhd),
targeting the B205mini first. Single-channel RX and TX are implemented; TX RF
validation is pending. Licensed GPL-3.0-or-later.

## Run

Requires Rust 1.85+ and optionally [just](https://github.com/casey/just).

```sh
cargo build --workspace
cargo run --bin ruhd -- --help
```

With a B2xx attached:

```sh
just cli probe usb          # check device state and USB access
just cli images download    # skip if UHD images are already installed
just cli init               # load firmware and FPGA after each plug-in
just cli find
just rx --freq 98e6 --rate 2e6 --gain 40 --duration 2 --file fm.sigmf-data
just bench --rx-rate 10e6 --duration 10
```

For TX, connect TX/RX to a terminated or attenuated test setup:

```sh
just tx --freq 915e6 --rate 1e6 --gain 0 --file burst.sc16
```

`just cli` runs `cargo run --bin ruhd --`. The `rx`, `tx`, and `bench` recipes
use release builds for streaming throughput. Without `just`, use
`cargo run --release --bin ruhd -- <command>`. Use `<command> --help` for options.
RX/TX files contain raw interleaved I/Q in host byte order (`sc16` by default);
RX also writes SigMF metadata. See [hardware setup](docs/hardware.md) for USB
permissions, image paths, and troubleshooting.

## Test

```sh
cargo test --workspace      # no hardware required
just ci                    # formatting, clippy, tests, and API docs
```

Hardware test recipes need a B205mini, initialize it, and run tests serially
with an isolated image cache. Set `RUHD_HW_SERIAL` to select a board if needed:

```sh
just hw-test v4.10.0.0      # hardware suite against one image release
just hw-matrix             # compatibility checks against both pinned image releases
```

TX tests are ignored by default; see [TX validation](docs/tx-validation.md).
The [AD9364 register comparison](docs/register-validation.md) checks codec
initialization against stock UHD. [Implementation notes](NOTES.md) cover the
crate layout, compatibility policy, and current limitations. Build API docs
with `just doc`; runnable Rust examples live in
[crates/uhd-b200/examples](crates/uhd-b200/examples).
