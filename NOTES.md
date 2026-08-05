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
uhd-transport   link / register / serial-bus traits, plus USB discovery (nusb)
chdr            packet framing; pure parsing, testable against captures
ad936x          AD936x transceiver driver; talks SPI, knows no board
uhd-b200        B2xx backend; the only crate that knows about USB and the FX3
uhd-cli         the `ruhd` binary
```

The dependency direction is the point. `ad936x` depends on **nothing but
`uhd-core`** — it defines the SPI trait it needs and `uhd-b200` implements it —
so the transceiver driver is reusable on any board carrying the part. `chdr`
depends on nothing, so packet framing can be tested against captured bytes with
no device present.

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
- **Discovery does not load firmware.** `b200_find` loads `usrp_b200_fw.hex`
  into every device it finds, waits up to 3 s for the device to re-enumerate,
  and only then reports what is attached — so merely listing devices mutates
  them, and fails outright if the images are not installed. `usb::probe` reports
  the state it found and leaves the loading to the caller, which is why
  `DeviceState` exists as a value rather than an intermediate. `ruhd init` is
  the explicit form of what `b200_find` does implicitly.
- **Rediscovery is by elimination, not by serial.** See the section below; this
  is forced by the hardware rather than chosen.
- **Control transfers have a timeout.** UHD passes `0` to libusb throughout
  `b200_iface.cpp`, which means *no* timeout: a device that stops answering
  hangs the process with no diagnostic. `CONTROL_TIMEOUT` is one second.
- **`find_image` searches every directory, not just the first.**
  `uhd::find_image_path` resolves the images directory first and then requires
  the file to be in that one directory, so a `UHD_IMAGES_DIR` containing a
  single hand-built FPGA image hides an otherwise complete system-wide package.
- **`ruhd images download` does not need root.** It installs into
  `$XDG_DATA_HOME/uhd/images`, which is ahead of the system locations on the
  search path, rather than into an install prefix.
- **Discovery reports bootloader devices.** `b200_find` only looks for
  programmed VID/PIDs, so a board with a blank EEPROM is invisible to it and you
  have to reach for `b2xx_fx3_utils` to discover that it exists. `usb::probe`
  includes `04b4:00f3` in the default search, because explaining a board that
  does *not* work is most of what discovery is for.
- **Overflows are not exceptions.** An overflow arrives in `RxMetadata` and
  `RxMetadata::check()` deliberately treats it as non-fatal, because absorbing
  overflows is exactly what a receive loop is supposed to do.
- **Compatibility numbers are reported by the transport and judged by the
  backend.** See the section below; this is the largest deliberate divergence so
  far.
- **A mismatch offers a fix rather than only naming one.** `check_fpga_compat`
  throws and tells you to run `uhd_images_downloader.py`. `uhd_b200::policy`
  looks in the image cache first, and if a verified image would put the device
  in window it says so — and loads it with `--auto-flash`. That is only
  reasonable because a B2xx's FPGA is volatile anyway; it would be a bad idea on
  a device with configuration flash.
- **The FPGA loader is version-agnostic.** `usb::fpga` pushes the bytes it is
  given and reports what the device says afterwards. It never reads a version
  out of the bitstream and never compares one. `b200_impl.cpp` interleaves the
  load with `check_fpga_compat`, which means a locally built FPGA image cannot
  be loaded at all without editing and rebuilding the host code.
- **A bad EEPROM header is described rather than thrown.** `get_mb_eeprom`
  throws on a magic or layout-compat mismatch, so a board with a slightly
  corrupt EEPROM cannot be inspected with the tool you would use to diagnose it.
  `MbEeprom::problems()` lists the faults and leaves the fields readable.

## A B2xx changes identity when firmware loads

Measured on a B205mini, across one `NeedsFirmware` → `Ready` transition:

| | Before | After |
| --- | --- | --- |
| VID:PID | `2500:0022` | `2500:0022` |
| serial | `0000000004BE` | `3168B41` |
| manufacturer | `Cypress` | `Ettus Research LLC` |
| product | `WestBridge` | `USRP B205-mini` |
| bus / address | `001` / `17` | `002` / `10` |

Every string descriptor changes, because before firmware the FX3 serves them
out of its own EEPROM and afterwards the firmware serves them, reporting the
USRP serial it read from the motherboard EEPROM over I²C. The bus changes
because the boot ROM enumerates at high speed while the firmware brings up
SuperSpeed, and Linux puts those on different buses. **The VID/PID is the only
field that survives, and it identifies the model, not the board.**

The consequence is that the obvious design for post-load rediscovery — record
the serial, poll until it comes back — cannot work, and fails in the most
annoying possible way: the firmware load succeeds, the device is sitting there
perfectly healthy, and the tool reports a timeout. `usb::wait_for_state`
therefore takes a snapshot of everything already in the target state *before*
the transfer that makes the device leave the bus, and identifies the returning
device as the one that is in that state and not in the snapshot. The serial is
still passed and still preferred when it matches, because it is the stronger
signal on transitions where it does survive; it just cannot be the only one.

UHD sidesteps this rather than solving it: `b200_find` re-runs discovery and
accepts whatever it finds, which is correct for one device and quietly wrong
for two.

Two smaller findings from the same work, both of which cost a debugging cycle:

- **udev has not finished when the device appears.** A re-enumerated device is
  visible to enumeration before its `/dev/bus/usb/…` node has its final
  permissions, so opening it the instant rediscovery finds it fails with
  `EACCES` on a machine where opening it 50 ms later works. Hence
  `OPEN_GRACE_PERIOD`, applied only after a re-enumeration — everywhere else a
  permission error is the real thing and should be reported at once, since
  "install the udev rules" is the most common first-run failure there is.
- **The firmware image is not all in one place.** `usrp_b200_fw.hex` has no
  extended-linear-address record for its first 595 data records, which
  therefore load at `0x0000_0100` — the FX3's tightly-coupled instruction
  memory — and only then do four type `04` records walk the high half from
  `0x4000` to `0x4003` for the remaining ~180 KB in system RAM. A parser that
  assumed a single leading `04` record would put four fifths of the firmware at
  the wrong address.

## Compatibility numbers are reported, not enforced

A B2xx carries three version numbers that all have to line up: the FX3
firmware's compatibility number, the FPGA build's, and the EEPROM layout's. UHD
checks each of them in a different place — `b200_impl::check_fw_compat`,
`b200_impl::check_fpga_compat`, `get_mb_eeprom` — and throws on any mismatch,
all three from inside `b200_impl`'s constructor.

`usb::CompatReport` gathers all of them, records what this driver was written
against, and stops. `problems()` returns the complaints as text;
`is_compatible()` summarises them; neither is called for you. The reasons, in
rough order of how much they matter:

- **The numbers are the diagnostic, and the throw destroys it.** When a device
  does not come up, these three numbers plus the EEPROM contents are what tells
  you which image on disk is the wrong one. Failing construction is exactly the
  moment they stop being printable.
- **A tool that reports on hardware must be able to describe hardware it would
  not open.** `ruhd probe --full` prints a full report for a device with
  mismatched images, which `uhd_usrp_probe` structurally cannot.
- **A major FPGA mismatch is fatal in UHD with no override**, so anyone with a
  locally built FPGA image has to patch and rebuild the host code. The loader
  here does not care what it is loading.
- **Acceptance is policy, and policy differs by caller.** A streaming
  application should refuse; a bring-up tool should warn; a recovery tool should
  proceed regardless. One hard-coded answer cannot serve all three.

The accept/reject decision, and what to do about a rejection, is
`uhd_b200::policy` — see the next section. The split is the point: the transport
gathers, the backend judges, and `ruhd probe --full` can still describe a device
`uhd_b200::open` would refuse.

The one place the old behaviour survives is `Fx3Iface::check_firmware_version`,
used by `initialize`: pushing an FPGA bitstream through firmware whose protocol
this driver does not know is not a diagnosis, it is a way to hang. Reading the
number is `firmware_version`, and it is what `CompatReport` uses.

## FPGA compat negotiation, and the commitment it creates

`uhd_b200::compat` defines a `FpgaCompat` enum with **one variant per supported
compatibility major**, and a `FpgaShim` behind each:

| Variant | Major | UHD constant | FPGA builds |
| --- | ---: | --- | --- |
| `V7` | 7 | `B205_FPGA_COMPAT_NUM` | the B2xxmini images |
| `V16` | 16 | `B200_FPGA_COMPAT_NUM` | `usrp_b200_fpga.bin`, `usrp_b210_fpga.bin` |

The major is read *before any other configuration* — `session::open_probed`'s
first two operations are the firmware compat read and the FPGA compat read — and
resolved once into a shim that everything below it consults. This is deliberately
not how `b200_impl.cpp` does it: there, a dozen `is_b2xx_mini(_product)` tests
are scattered from the constructor through `update_clock_source`,
`update_time_source` and `update_bandsel`, and adding a third FPGA generation
means auditing the file. Here the difference between generations is a table:
GPSDO presence, band-select GPIO, the reference-lock mechanism, the clock and
time source lists, the radio-chain ceiling, the stream IDs, and
`SR_CORE_GPSDO_ST`'s existence.

The policy layer over that is `uhd_b200::policy::negotiate`, a pure function of
a `DeviceCompat` and an `ImageCatalog`:

1. **In window** — the reported major has a shim and is legal for this board:
   proceed on that shim.
2. **Out of window, fixable** — a verified image in the `uhd-images` cache would
   put it in window: offer the reload, or perform it with `--auto-flash` through
   `usb::configure`, then read and negotiate again. The device's own answer after
   the load is what decides, not the loader's claim.
3. **Neither** — refuse, with a diagnostic naming the device's compatibility
   number, the supported window, and the manifest entry that would fix it.

A firmware major mismatch is checked first and is never case 2: an FPGA reload
cannot fix firmware, and pushing a bitstream through firmware whose protocol is
unknown is how a device hangs. An *unreadable* compat register is not case 2
either — the usual cause is another process holding the bulk interfaces, and
reflashing a working FPGA is the wrong answer to that.

`ruhd doctor` prints the four numbers the decision rests on — firmware compat,
FPGA compat major and minor, EEPROM revision — plus the verdict and the fix. It
is the only command that runs the policy without acting on it, which is exactly
what you want from a device that will not open.

### The commitment

Every variant of `FpgaCompat` is a promise, and it is worth being explicit about
the terms before the enum grows a third one:

- **Each supported major keeps its shim alive.** `V7` and `V16` both have a
  complete `FpgaShim`. A behavioural difference discovered later has to be added
  to *both*, not guarded with a version test at the call site — the whole value
  of the enum is that call sites do not branch.
- **Each supported major stays CI-tested.** `compat::tests` and `policy::tests`
  run on every push and cover both shims and the full policy matrix. The
  `hw-tests` feature adds a hardware matrix that runs once per cached image
  version — currently the two different `usrp_b205mini_fpga.bin` builds in
  `crates/uhd-images/images.toml`, which have different bytes, sizes and hashes —
  and asserts the FX3 ends up holding the hash of the image that leg cached. Two
  legs against two versions is what makes it a test of *negotiation* rather than
  a test that the driver works. Both legs were run against the bench B205mini:
  v4.6.0.0 loads 4 222 976 bytes, v4.10.0.0 loads 4 223 252, both land on major
  7, and each leg's cache holds only its own image so the choice is forced.
- **Dropping a major from the window is a breaking change for `uhd-b200`.**
  Hardware in the field does not update itself, and removing `V16` would turn
  every B200 and B210 from "supported" into "refused with a diagnostic". It is a
  semver-major change, and it needs the same deliberation as removing a public
  method — not a cleanup.
- **The manifest and the window have to agree.** An image the manifest offers
  whose compat major has no shim is a `ruhd images sync` that installs something
  we would then refuse. `policy::tests::the_shipped_manifest_covers_the_b205mini_in_window`
  is the assertion that keeps them in step.

The cost this buys is small and known: two zero-sized shims and a table each.
The cost it avoids is the one `b200_impl.cpp` is paying.

## Reaching the FPGA is not the same as reaching the FX3

Everything in `usb::fx3` is a control transfer on endpoint 0, answered by the
microcontroller. The FPGA's settings bus is not reachable that way at all — the
vendor requests stop at the FX3. `usb::ctrl` therefore claims interfaces 3 and 4
and sends CHDR context packets over bulk endpoints `0x04` and `0x88`, which the
FX3 forwards across the GPIF. This is the same path samples will take.

Three consequences that cost time to rediscover:

- **Nothing on the settings bus answers until the FPGA is configured.** An
  unconfigured FPGA does not error, it simply never replies, so the symptom is
  the full two-second ack timeout. `CompatReport` checks the FX3 state first and
  reports `FpgaCompat::Unconfigured` rather than paying for the timeout.
- **The GPIF reset after an FPGA load is not optional.** `b200_impl` calls
  `reset_gpif()` on the line after `load_fpga`, unconditionally, and skipping it
  leaves a device that answers every control request perfectly and returns
  nothing at all on its bulk endpoints.
- **The readback is 64 bits and arrives high word first.** `peek32` in
  `b200_radio_ctrl_core.cpp` issues the same 64-bit readback and then selects a
  half with `(addr / 4) & 1`; `LocalCtrl` overrides `RegisterIface::peek64` for
  the same reason, since the default's two round trips could straddle a change.

A fourth, learned the hard way while building the hardware matrix: **the local
core answers as soon as the FPGA is configured, the radio cores do not.** The
radio chains run off the AD936x's clock, so until the codec is both clocking and
out of its wait state nothing at `SR_TEST`/`RB32_TEST` replies and the symptom is
again the full two-second ack timeout. Everything compat negotiation needs — the
compatibility register at address 0, the radio-chain count in `RB32_CORE_STATUS`
— is on the local core, which is why the hardware matrix can run today and
`RadioCtrl::gate` is behind `RUHD_HW_RADIO_CLOCKED` until then.

`usb::ctrl` is deliberately the minimum: one packet, one reply, under a mutex.
It carries no timestamps, queues nothing, and does not demultiplex the GPSDO
UART out of the same endpoint. When the real radio control core lands it should
replace this rather than wrap it.

## The bootstrap order is forced by where the SPI master sits

The local/radio split above is not just a fact about latency, it is the shape of
bring-up. The radio cores run off the AD936x's sample clock; the AD936x is
started over SPI; and `SR_CORE_SPI` is therefore a **local**-core register. That
circle only opens one way:

```text
  fpga configured
    -> local core answers
      -> SR_CORE_MISC : release the codec's reset pin
      -> SR_CORE_SPI  : spi_core_3000 at 1 MHz
        -> AD936x register 0x037 reads back 0x08 | revision
          -> Ad936x::initialize_clocking : the BBPLL locks
            -> (not yet) the ENSM leaves its wait state
              -> radio cores answer: ATR, time, dsp, scratch
```

`uhd_b200::spi` and `uhd_b200::misc` are the two local-core peripherals, and
`B200Session::codec_gate` is the arrow in the middle. `uhd_b200::atr` and
`uhd_b200::time_core` are on the far side. The BBPLL is now running, but whether
the part drives its data port before the ENSM has been walked into ALERT is not
something this workspace has confirmed on a board — so `hw_compat.rs`'s scratch
gate stays behind `RUHD_HW_RADIO_CLOCKED` until the ENSM transitions land and a
B205mini says otherwise.

`RadioCtrl` therefore hands out `CtrlPort`s rather than owning a route: one
transport, a stream ID per port, and the queue of outstanding commands records
which response stream ID each one is owed a reply from. `b200_impl` gets the
same effect by constructing a separate `radio_ctrl_core` per SID over a shared
transport; the difference is only that a port here is a cheap clone rather than
an object with a task attached.

## The SPI boundary is drawn at the part, not at the bus

`ad936x` defines its own `SpiInterface` — one transaction, MSB first, no slave
select, no clock edges — and `uhd-b200` implements it. `uhd_transport::SpiIface`
still exists and is still what `spi_core_3000` implements, but `ad936x` does not
use it and does not depend on `uhd-transport` at all.

This is a divergence from both UHD and from the plan recorded earlier in this
file, and the reason is visible in one line of `ad9361_ctrl.cpp`:

```cpp
config.miso_edge = uhd::spi_config_t::EDGE_FALL; // TODO (Ashish): FPGA SPI
                                                 // workaround. This should be EDGE_RISE
```

That is a workaround for a *Xilinx SPI core*, sitting inside the transceiver
driver, applied to every board that will ever carry an AD936x. It is there
because `ad9361_io_spi` holds a general-purpose `spi_iface` and therefore has to
build a `spi_config_t`, and once a driver is building bus configuration it will
inevitably encode something about the bus. Narrowing the trait removes the
opportunity: with no `spi_config_t` to fill in, `ad936x` cannot express a
board-specific workaround even by accident, and `uhd_b200::spi::CODEC_SPI_CONFIG`
is the single place the edge is chosen.

What each side owns, concretely:

| Fact | Owner |
| --- | --- |
| 24-bit word, R/W bit at 23, address at 8, data at 0 | `ad936x::spi` |
| register 0x037 means product ID | `ad936x::regs` |
| a 40 MHz reference feeds the BBPLL | `ad936x::clocking` |
| the codec is slave 0, clocked at 1 MHz | `uhd_b200::spi` |
| MISO is sampled on the falling edge | `uhd_b200::spi` |
| the reset pin is bit 2 of `SR_CORE_MISC` | `uhd_b200::misc` |
| the data port is CMOS, delays 0/0xF | `uhd_b200::codec` |
| a mini stays on RF port A, a B210 switches | `uhd_b200::codec` |

The last two are the second half of that boundary. UHD makes them a virtual
interface — `ad9361_params`, with `b200_ad9361_client_t` and
`b2xxmini_ad9361_client_t` implementing four getters that return constants — and
here they are a `ClientParams` struct handed over at construction. Both
implementations of those getters return compile-time constants, and there is
nowhere in the driver that calls one at a moment when the answer could have
changed; making them data means the settings are printable, comparable and
testable, and that a callback from inside the codec's own lock cannot exist.

## The FPGA image hash is UHD's, on purpose

`usb::fpga::image_hash` reproduces UHD's `generate_hash` byte for byte — a Boost
`hash_combine` over the file, one byte at a time — and it is not replaced with
something better because **the FX3 is a shared mailbox**. `load_fpga` reads the
hash the FX3 is holding, computes this over the image it was about to send, and
skips the load if they match. If this driver stored a number UHD would not have
computed, every `uhd_usrp_probe` after a `ruhd init` would reconfigure the FPGA
from scratch, and vice versa.

Verified on a B205mini: both drivers compute `0xe38c4b7b` for
`usrp_b205mini_fpga.bin`, and each skips the load after the other has done it.

The one subtlety is that UHD reads the file into a `char`, so the addend is
sign-extended wherever C's `char` is signed — which is every platform this
driver targets, and is why `image_hash` has an `as i8` in the middle of it. On a
platform where `char` is unsigned (ARM Linux) a UHD built there would compute a
different value; the only consequence is one redundant FPGA load, since nothing
but UHD ever interprets the number.

## Licensing

UHD is GPL-3.0-or-later. This is a reimplementation written while reading UHD's
source, so the workspace is licensed GPL-3.0-or-later to match. If the intent is
ever a permissively licensed driver, that requires a clean-room process — no
reading `reference/uhd` — and this decision needs revisiting **before** any
implementation code is written, not after.

## Status

`uhd-core` and `uhd-transport` have their traits and value types. The working
hardware path is `uhd_transport::usb`, which now takes a device all the way from
"just plugged in" to a live settings bus:

- enumeration and classification with [`nusb`][nusb] (`ruhd probe usb`);
- Intel HEX parsing and the FX3 boot ROM's `0xA0` load protocol, with
  re-enumeration handling and the compatibility-number check (`ruhd init`,
  `ruhd reset`);
- the full FX3 vendor-request set: firmware compat, USB speed, FX3 state, FPGA
  config start/data/status, image hashes, GPIF and FX3 reset, EEPROM read and
  write;
- the FPGA bitstream load, hash shortcut and post-load GPIF reset (`usb::fpga`);
- the motherboard EEPROM map, both layouts (`usb::eeprom`);
- settings-bus peek/poke over the bulk endpoints (`usb::ctrl`), which is what
  makes the FPGA compatibility number readable;
- `usb::CompatReport`, which gathers every version number and judges none of
  them (`ruhd probe --full`);
- image path resolution and a downloader for the B2xx images package
  (`ruhd images status` / `download`).

`uhd-b200` has the compat negotiation layer over that: `compat` (the
`FpgaCompat` enum and its shims), `policy` (the accept / reload / refuse
decision), `session` (the open path and the read-only `checkup`), and
`radio_ctrl` promoted to open against a negotiated major. `ruhd doctor` is the
CLI front end. It is not yet a `Device` impl — `ruhd find` still finds nothing.

On top of that sit the settings-bus peripherals, one module each:

| Module | Ports | Core |
| --- | --- | --- |
| `spi` | `spi_core_3000.cpp`, `b200_local_spi_core` | local |
| `misc` | `update_gpio_state`, `reset_codec`, `get_ref_locked` | local |
| `atr` | `gpio_atr_3000.cpp`, `update_atrs` | radio |
| `time_core` | `time_core_3000.cpp`, `update_time_source`, `set_time` | radio |

`ad936x` has the bus, the identity check and the clock chain: the 24-bit command
framing, the register file, the soft reset and the product-ID read that
`ad9361_device_t::initialize` opens with, and then the first half of that
sequence — the reference clock path, `_setup_rates` in full (the rate table, the
BBPLL divide word, the DAC divider and both FIR filters), the data port and the
interface delays. The arithmetic is split out into `ad936x::clocking` as pure
functions, so the numbers can be checked against the C++ without a board.

Nothing above that yet: no RF tuning, gain tables, analog filter calibration,
DC-offset or quadrature calibration, and no public `set_clock_rate` — the
reference's wraps `_setup_rates` in ENSM transitions and seven re-calibrations,
and shipping the wrapper without them would give a part that clocks at the new
rate and receives badly.

Verified against the bench B205mini (serial `3168B41`, FPGA compat 7.0), on a
device whose codec no other process had touched.
`cargo run -p uhd-b200 --example codec_id` releases the reset pin through
`SR_CORE_MISC`, soft-resets the part over SPI at 1 MHz, and reads `0x0a` back
from register `0x037` — family `0x08` as `initialize` requires, silicon revision
2. The four SPI-level tests in `hw_codec.rs` pass on the same board: the gate,
ten repeated product-ID reads with no variation, a write/read loopback over
`0x03d` across `0x00`/`0x55`/`0xaa`/`0xff`, and the reset-line check.

The two clock-chain tests in that file — `the_clock_chain_comes_up_and_the_bbpll_locks`
and `the_chain_re_solves_for_every_rate_the_board_is_asked_for` — arrived with
the clocking port and **have not been run against a board yet**. They are the
next thing to try on the bench, and the lock bit is the finding to record here.

That is the first evidence in this workspace that anything past the FPGA
exists — every earlier hardware result was the FX3 or the FPGA describing
itself.

Verified against a B205mini, from a bootloader-state device:

- `ruhd init` loads firmware 8.0, reads the EEPROM (rev 0 layout, serial
  `3168B41`, product `0x773a`, revision 3), pushes all 4 222 976 bytes of
  `usrp_b205mini_fpga.bin` in 512-byte chunks in about 8.2 s, resets the GPIF,
  and reads FPGA compatibility 7.0 back off the settings bus;
- stock `uhd_usrp_probe` then opens the device and prints **no** "Loading FPGA
  image" line, passes its own register loopback test, and reports the identical
  serial, product, revision, FW 8.0 and FPGA 7.0. That is the cross-check that
  matters: UHD accepts what we loaded, at both the firmware and the FPGA layer,
  and agrees with our EEPROM decode field for field;
- the handover works in both directions — after a stock `uhd_usrp_probe` load,
  `ruhd init` reports the FPGA already current and skips it, because the image
  hashes agree.

`ruhd find` still finds nothing because no backend registers itself yet, and no
samples move.

`nusb` rather than `rusb`/`libusb`: it is pure Rust, so the build has no C
dependency, and its enumeration reads sysfs without opening anything, which is
what lets discovery describe a device the caller has no permission to open. That
distinction is load-bearing for the udev diagnostics in `ruhd probe usb`.

Rough order of work from here:

1. ~~**Compat negotiation**~~ — done; see the section above.
2. ~~**The settings-bus peripherals**~~ — done: `spi`, `misc`, `atr`,
   `time_core`, and `ad936x`'s SPI register access. The codec product-ID gate is
   the proof.
3. `ad936x` — the initialisation sequence. The clock half is done: BBPLL, the
   clock tree, the rate table, the FIRs and the digital interface. What remains
   is the ENSM transitions, RF tuning, the gain and mixer tables, the analog
   filter calibrations, the DC-offset and quadrature calibrations, and the
   public `set_clock_rate` that needs them. `ad9361_device.cpp` is 3011 lines
   and most of them are still here.
4. `uhd-b200` — a real `Device` impl over all of it: the EEPROM read for
   identity, tick rate, tuning, gain. Enough for `ruhd find` to print a real
   B205mini.
5. `chdr` — packet framing.
6. `uhd-b200` streamers — first samples.

Step 3 is the one that unblocks the rest: the scratch gate in `hw_compat.rs` and
the two radio-core modules all become runnable the moment the codec is clocking,
and `RUHD_HW_RADIO_CLOCKED` can go away with them.

When step 4 lands, `register_backends()` in `crates/uhd-cli/src/main.rs` becomes
`uhd_b200::register()` and the CLI needs no other change.

[nusb]: https://docs.rs/nusb
