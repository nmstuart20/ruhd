# Implementation notes

Working notes for reimplementing UHD in Rust. Design rationale that belongs next
to the code lives in doc comments; this file covers the things that have no
single obvious home — where the reference lives, what to read, and which
decisions were made once and should not be relitigated per-crate.

## The UHD reference checkout

`reference/uhd` is a git submodule pinned to **UHD v4.10.0.0** (commit
`2af4ddb96219a99d2300804830e0971f79557b23`), the latest release tag at the time
of writing. It is shallow-cloned and read-only: nothing in the Rust build reads
it, CI does not check it out, and it is never modified. It is there to be read.

```sh
just reference          # fetch it
just grep-ref set_gain  # search it
```

Bumping the pin is a deliberate act: `cd reference/uhd && git fetch --tags &&
git checkout <tag>`, then commit the gitlink and note here what changed in the
files below.

## Primary reference files

These four are the ones that matter for the B205mini. Everything else in UHD is
either shared infrastructure we are replacing wholesale or support for hardware
we are not targeting yet.

| File | Lines | What it holds |
| --- | ---: | --- |
| [`host/lib/usrp/b200/b200_iface.cpp`](reference/uhd/host/lib/usrp/b200/b200_iface.cpp) | 817 | The USB layer. FX3 firmware load, FPGA bitstream load, the vendor-request protocol, and peek/poke plus SPI/I2C carried over USB control transfers. Maps to `uhd-transport` traits and the low half of `uhd-b200`. |
| [`host/lib/usrp/b200/b200_impl.cpp`](reference/uhd/host/lib/usrp/b200/b200_impl.cpp) | 1503 | Device construction and the control plane: discovery, EEPROM, clock and time sources, tick rate, the property tree wiring, and the `b200_product_t` detection that tells a B205mini from a B210. Maps to `uhd-b200`'s `Device` impl. |
| [`host/lib/usrp/b200/b200_io_impl.cpp`](reference/uhd/host/lib/usrp/b200/b200_io_impl.cpp) | 613 | The streaming plane: streamer construction, sample-rate coercion, the tick-rate interlocks between channels, and the receive/transmit packet handling. Maps to `uhd-b200`'s `RawRxStreamer`/`RawTxStreamer` and to `chdr`. |
| [`host/lib/usrp/common/ad9361_driver/ad9361_device.cpp`](reference/uhd/host/lib/usrp/common/ad9361_driver/ad9361_device.cpp) | 3011 | The whole AD9361/AD9364 transceiver driver: BBPLL and synthesiser calibration, RX/TX analog filter tuning, DC-offset and quadrature calibration, gain tables, FIR programming. Maps to `ad936x`. Much the largest and fiddliest of the four. |

Useful secondary files, in rough order of how often they come up:

- `host/lib/usrp/b200/b200_regs.hpp` — the settings-bus register map.
- `host/lib/usrp/b200/b200_impl.hpp` — USB VID/PID table, frame-size defaults,
  compatibility numbers.
- `host/lib/usrp/b200/b200_cores.cpp` — the radio control core.
- `host/lib/usrp/common/ad9361_driver/ad9361_gain_tables.h` — the gain tables
  `ad9361_device.cpp` indexes into.
- `host/include/uhd/types/` — the C++ originals of the value types in
  `uhd-core`: `tune_request.hpp`, `tune_result.hpp`, `ranges.hpp`,
  `time_spec.hpp`, `metadata.hpp`, `device_addr.hpp`.

## B205mini specifics

- USB vendor ID `0x2500`, product ID `0x0022` (`B200_VENDOR_ID` /
  `B205MINI_PRODUCT_ID` in `b200_iface.hpp`). A plain B200 is `0x0020`.
- Carries an **AD9364** — the single-channel sibling of the AD9361 — so it has
  1 RX and 1 TX channel. This is exactly why channel counts are a runtime query:
  the same `uhd-b200` code has to report 2 for a B210.
- Zero-IF direct conversion, so there is an LO leakage / DC spike at the tuned
  centre frequency unless you use an LO offset. See
  `TuneRequest::with_lo_offset`.
- The device arrives with no FPGA image loaded and must be programmed over USB
  on every plug-in; there is no flash. `b200_iface.cpp` has the sequence.
- Default USB data frame size is 8176 bytes, deliberately *not* a multiple of
  512 (`B200_USB_DATA_DEFAULT_FRAME_SIZE`).

## Crate layout and why

```
uhd-core        traits and value types; no I/O, no hardware knowledge
uhd-transport   link / register / serial-bus traits; no implementations yet
chdr            packet framing; pure parsing, testable against captures
ad936x          AD936x transceiver driver; talks SPI, knows no board
uhd-b200        B2xx backend; the only crate that knows about USB and the FX3
uhd-cli         the `ruhd` binary
```

The dependency direction is the point. `ad936x` depends on `uhd-transport`'s
`SpiIface`, **not** on `uhd-b200`, so the transceiver driver is reusable on any
board carrying the part. `chdr` depends on nothing, so packet framing can be
tested against captured bytes with no device present.

## Design decisions

These were settled once, up front, because reversing them later means breaking
every backend.

### `Device` is object-safe

Applications hold `Arc<dyn Device>`. A B205mini over USB and a future X310 over
UDP are interchangeable at the call site, and `uhd-cli` never names a backend
crate. `crates/uhd-core/src/device.rs` ends with a compile-time assertion that
this stays true, so a method that breaks it fails loudly and locally.

The cost is that `Device` cannot have generic methods. Two consequences:

- Generic conveniences live on `DeviceExt`, blanket-implemented for every
  `Device` including `dyn Device`.
- Sample types are erased at the streamer boundary (below).

### Streamers are per-direction and sample-type-erased

Receive and transmit are separate traits — `RawRxStreamer` and `RawTxStreamer` —
rather than one bidirectional streamer. They have genuinely different
operations (`issue_stream_cmd` versus `recv_async_msg`), different lifetimes in
practice, and are usually driven from different threads.

Backends implement the `Raw*` traits, which speak in `&mut [u8]`. Callers hold
`RxStream<T>`/`TxStream<T>`, which speak in `&mut [T]` and do the free,
bytemuck-checked reinterpretation. The alternative — one constructor per sample
type on `Device` — would mean adding a format is a breaking change to every
backend. This way a backend writes one streamer, gets every host format, and
adding `sc4` later touches only `uhd-core::sample`.

### Channel counts are runtime queries

`num_channels(Direction) -> usize`, never a const generic or an associated
constant. A B205mini reports 1, a B210 reports 2, an N310 reports 4, all from
code that does not know which is plugged in. Buffers are always
`&mut [&mut [T]]`, one per channel — the single-channel case goes through
`recv_one`, which is a wrapper, not a separate path.

### Direction is an argument, not a doubled API

`set_gain(Direction::Rx, ...)` rather than `set_rx_gain(...)`. This halves the
trait, and it makes "do this for both directions" a loop over `Direction::ALL`
instead of copy-paste.

### `&self`, not `&mut self`

A receive thread and a control thread need concurrent access to the same device.
Backends use interior mutability and the trait requires `Send + Sync`.

### Optional operations have defaults

Every `Device` method that some plausible device cannot do returns
`Error::NotSupported` by default, so adding methods later is not a breaking
change. The genuinely universal ones — identity, channel counts, rate, tuning,
gain, antenna, opening a streamer — have no default, so a backend cannot forget
them silently.

### Room for network devices

Nothing above the transport layer assumes USB:

- `Endpoint` already has a `Udp(SocketAddr)` variant beside `Usb(..)`.
- `RecvLink`/`SendLink` are fixed-size frames from a pool, which is what both a
  libusb transfer ring and a `recvmmsg` buffer array are.
- `RegisterIface` is peek/poke, equally an FX3 control transfer or a ZPU packet.
- `DeviceArgs` already distinguishes `serial=` from `addr=`, and `Registry`
  dispatches on `type=` without knowing what any type means.

Adding X310-over-UDP should mean writing a UDP link, a factory, and a `Device`
impl — and editing nothing in `uhd-core`.

## Deliberate divergences from UHD

Places where we do *not* copy the reference, and why. Keep this list short and
justified.

- **`Range::clip` stays on the step grid.** UHD's `meta_range_t::clip` snaps to
  the nearest step and then clamps, which can return an off-grid value above the
  maximum — and that value then gets programmed into a register that cannot
  represent it. Ours steps down instead. See the test in
  `crates/uhd-core/src/range.rs`.
- **Malformed device args are an error.** UHD silently turns a bare `foo`
  segment into `foo=""`, which hides typos. We reject it.
- **No property tree.** UHD's runtime-typed `property_tree` of string paths
  exists largely to paper over C++'s lack of a good interface story. The trait
  methods are the interface here.
- **Overflows are not exceptions.** An overflow arrives in `RxMetadata` and
  `RxMetadata::check()` deliberately treats it as non-fatal, because absorbing
  overflows is exactly what a receive loop is supposed to do.

## Licensing

UHD is GPL-3.0-or-later. This is a reimplementation written while reading UHD's
source, so the workspace is licensed GPL-3.0-or-later to match. If the intent is
ever a permissively licensed driver, that requires a clean-room process — no
reading `reference/uhd` — and this decision needs revisiting **before** any
implementation code is written, not after.

## Status

Scaffolding only. `uhd-core` and `uhd-transport` have their traits and value
types; `chdr`, `ad936x` and `uhd-b200` are empty; `uhd-cli` runs but finds
nothing because no backend registers itself yet.

Rough order of work from here:

1. `uhd-transport` — a real libusb-backed link and register interface.
2. `uhd-b200` — enumeration, FX3 firmware load, FPGA load. Enough for
   `ruhd find` to print a real B205mini.
3. `ad936x` — SPI register access, then initialisation and tuning.
4. `chdr` — packet framing.
5. `uhd-b200` streamers — first samples.

When step 2 lands, `register_backends()` in `crates/uhd-cli/src/main.rs` becomes
`uhd_b200::register()` and the CLI needs no other change.
