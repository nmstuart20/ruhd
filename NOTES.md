# Implementation notes

## Crate layout

| Crate | Responsibility |
| --- | --- |
| `uhd-core` | Device/stream traits and value types; no I/O. |
| `uhd-transport` | Transport traits, USB discovery, image loading, and EEPROM access. |
| `uhd-images` | Image manifest, verified cache, and optional downloads. |
| `chdr` | Legacy CVITA/CHDR framing; no dependencies. |
| `ad936x` | Board-independent AD9361/AD9364 driver with its own SPI trait. |
| `uhd-b200` | B2xx compatibility, session, radio controls, and RX/TX backend. |
| `uhd-cli` | The `ruhd` binary. |

`Device` is object-safe; generic helpers live on `DeviceExt`. Backends implement
byte-oriented `RawRxStreamer`/`RawTxStreamer`; callers use typed streams.
Channel counts are runtime queries, and controls take a `Direction`. Device
configuration uses interior mutability to allow concurrent control and streaming.
Detailed behavior and hardware register maps belong in the crate's rustdoc.

## Compatibility policy

Discovery is read-only. `ruhd init` explicitly loads the volatile FX3 and FPGA
images; firmware loading can change the device's serial and USB bus address.
The transport gathers compatibility information without rejecting the device,
so `ruhd probe --full` can diagnose hardware that the backend cannot open.
`ruhd doctor` reports the backend's verdict; `--auto-flash` permits a cached
FPGA reload.

The backend negotiates an FPGA shim before configuration: major 7 for B2xxmini,
major 16 for B200/B210. Version-dependent behavior belongs in those shims.
Each supported major must retain its shim and software tests; dropping a major
is a breaking change. The image manifest must stay consistent with the supported
window. The hardware matrix uses UHD v4.6.0.0 and v4.10.0.0 B205mini images;
both exercise major 7, so it does not validate major 16 on hardware.

An FPGA reload cannot fix incompatible FX3 firmware. An unreadable compatibility
register is also not grounds for reflashing: check for another process holding
the device first.

## Current limits

- One single-channel RX or TX streamer may exist per device. Simultaneous RX/TX,
  two-channel B210 streaming, and `sc8`/`sc12` wire formats are unsupported.
- Drop the streamer before changing rates, clocks, reference/time sources, or
  device time. Streamers retain the session after the device handle is dropped.
- On B210, the LO and analog bandwidth are shared between channels within each
  direction. External reference clock configuration and instantaneous AGC gain
  readback are unsupported.
- RX has been checked against stock UHD on a B205mini over USB 3, including
  61.44 MS/s `sc16` captures. Throughput depends on the host and sample format.
- TX has software coverage; RF correctness and throughput remain unvalidated.
  A send queues USB data; wait for `BurstAck` before dropping a stream when
  delivery matters. The CLI throughput benchmark measures RX only.

## UHD reference checkout

`reference/uhd` is a read-only submodule pinned to UHD v4.10.0.0. It is optional
and is not used by the Rust build or normal CI.

```sh
just reference
just grep-ref ad9361_set_gain
```

Start with these paths under `reference/uhd/host/lib/usrp/`:

- `b200/b200_iface.cpp`: firmware and FPGA loading, USB requests.
- `b200/b200_impl.cpp`: discovery, clocks, compatibility, device construction.
- `b200/b200_io_impl.cpp`: streaming and sample-rate handling.
- `common/ad9361_driver/ad9361_device.cpp`: transceiver initialization and tuning.

When changing the reference pin, review the image manifests in
`crates/uhd-images/images.toml` and `crates/uhd-cli/src/images.rs` as well.

## Validation

[Hardware setup](docs/hardware.md), [AD9364 register validation](docs/register-validation.md),
and [TX validation](docs/tx-validation.md) contain the bench procedures. Keep
validation results with the relevant procedure rather than growing a work log here.

## Licensing

The workspace uses GPL-3.0-or-later to match the UHD source used as its reference.
