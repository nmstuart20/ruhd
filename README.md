# ruhd

A pure-Rust reimplementation of the [Ettus USRP Hardware Driver][uhd], targeting
the USRP B205mini first.

**Status: early.** The traits and value types are in place, and the USB layer
works against real hardware: `ruhd probe usb` finds and classifies a B2xx, and
`ruhd init` takes one from freshly plugged in to FX3 firmware running, FPGA
configured, and a live settings bus. Stock `uhd_usrp_probe` then opens the same
device without reloading anything. `ruhd find` discovers firmware-ready B2xx
devices; the generic device/streamer implementation is not complete, so no
samples move yet. See [NOTES.md](NOTES.md) for the design rationale, the
reference material, and the plan.

## Layout

| Crate | Contents |
| --- | --- |
| `uhd-core` | The object-safe `Device` trait, streamers, and the value types they trade in. No I/O. |
| `uhd-transport` | Link, register and serial-bus traits; USB discovery, FX3 firmware and FPGA loading, the EEPROM map and settings-bus access (`usb`); image path resolution (`images`). |
| `chdr` | Packet framing. Empty. |
| `ad936x` | AD9361/AD9364 transceiver driver: complete initialization, RF/sample-rate tuning, gain tables and AGC, FIR/analog filters, and DC/quadrature calibration. |
| `uhd-b200` | B2xx discovery, compatibility policy, open session, settings-bus peripherals, and AD936x board integration. |
| `uhd-cli` | The `ruhd` binary. |

## Getting started

```sh
just build
just test
just ci        # everything CI runs
just cli find
```

`just` with no arguments lists every target. If you do not have [just][just],
the underlying commands are plain `cargo build` / `cargo test` / `cargo clippy`.

The UHD C++ sources are vendored as a read-only submodule for reference:

```sh
just reference              # fetch reference/uhd, pinned at v4.10.0.0
just grep-ref ad9361_set_gain
```

It is not needed to build anything.

## Finding a device

```sh
just cli find                       # firmware-ready devices and their identity
just cli find --args type=b200,serial=30A1B2C
just cli probe usb                  # every B2xx and unprogrammed FX3 on the bus
just cli probe usb --serial 30A1B2C
just cli probe usb --vid 2500 --pid 0022
```

`find` is read-only and reports boards whose FX3 firmware is running, using the
motherboard EEPROM for the product, serial, name, and hardware revision. It
does not implicitly load firmware: if a newly plugged-in board is absent, use
`probe usb` to see its pre-firmware state and `init` to bring it up.

`probe usb` is lower-level discovery: it enumerates the bus, reads descriptors
and says what each device is and how far along it is. It works before any
backend exists and before firmware has been loaded, which is what makes it the
thing to run when a device is not showing up.

```
Device 0
  product: B205mini
  state:   needs firmware
  serial:  0000000004BE
  usb id:  2500:0022
  bus:     001 address 13
  vendor:  Cypress
  descr:   WestBridge
  access:  ok
  next:    load usrp_b200_fw.hex
```

`state` is the useful column. A B2xx has no flash: the FX3 firmware is volatile
and has to be pushed over USB on every plug-in, so `needs firmware` above is
what a perfectly healthy board looks like a second after it is plugged in.
`uninitialized` is the rarer case — the board enumerated as a bare Cypress FX3
(`04b4:00f3`) because its EEPROM has never been programmed, and it has no serial
or product to report until it is. `ready` means firmware is running.

The full state machine, and the tables of vendor and product IDs it is built
from, are documented in `uhd_transport::usb` (`just doc-open`).

## Bringing a device up

```sh
just cli init                       # the one attached device
just cli init --serial 3168B41      # a specific one
just cli init --force               # reload firmware too, not just the FPGA
just cli init --force-fpga          # reconfigure the FPGA even if it looks current
just cli init --no-fpga             # firmware only
just cli reset                      # drop back to the bootloader
```

A B2xx has *two* volatile images, and `init` loads both. First it parses
`usrp_b200_fw.hex` and pushes it into FX3 RAM over the Cypress boot ROM's `0xA0`
vendor request, waits for the device to re-enumerate, and checks the firmware
compatibility number it then reports. Then it reads the motherboard EEPROM to
find out which board this actually is, and streams that board's bitstream to the
FX3 in 512-byte vendor requests while the FX3 clocks it into the FPGA's
slave-serial pins — after which the settings bus is alive and the FPGA can be
asked its own version.

```
firmware:  /usr/local/share/uhd/images/usrp_b200_fw.hex
           11516 records, 182268 bytes, entry 0x40019dec
device:    B205mini [needs firmware] 2500:0022 bus 001 address 21
           loading firmware, this takes a moment

Device
  product:  B205mini
  serial:   3168B41
  usb id:   2500:0022
  state:    ready
  vendor:   Ettus Research LLC
  loaded:   yes, re-enumerated in 406 ms

fpga:      /usr/local/share/uhd/images/usrp_b205mini_fpga.bin
           4222976 bytes, hash 0xe38c4b7b
           loaded 4222976 bytes in 8211 ms, 512-byte chunks; fx3 is running
           gpif reset

Compatibility
  firmware: 8.0 (expected 8.0)
  usb:      operating over usb 3
  fx3:      running
  fpga:     7.0 (expected major 7)
  eeprom:   rev 0 (36 bytes)
    serial:   3168B41
    name:     B205i
    product:  0x773a (B205mini)
    revision: 3
  compat:   every version number this driver checks lines up
```

Both halves are idempotent. Firmware that is already running is checked and left
alone; an FPGA that already holds this image is left alone too, decided from a
hash the host stores on the FX3. That hash is UHD's, computed the same way byte
for byte, which is what lets the two drivers hand a device back and forth: after
the run above, `uhd_usrp_probe` opens the device and does *not* print "Loading
FPGA image".

The same block is available on its own, without touching anything:

```sh
just cli probe --full
```

**Version mismatches are reported, not enforced.** UHD throws on a firmware,
FPGA or EEPROM compatibility mismatch from inside device construction, which is
exactly the moment the numbers you need become unreadable. Here they are
gathered into a `CompatReport`, printed, and left for a policy layer to judge —
so a board with a locally built FPGA image, or one that needs explaining rather
than opening, still tells you everything it knows.

Two things about the firmware half are worth knowing before they surprise you.

**The device changes identity when firmware loads.** Before the load the FX3
serves its string descriptors out of its own EEPROM; afterwards, the firmware
serves them, reporting the USRP serial it read from the motherboard EEPROM over
I²C. Across the single transition above, on a B205mini:

| | Before | After |
| --- | --- | --- |
| VID:PID | `2500:0022` | `2500:0022` |
| serial | `0000000004BE` | `3168B41` |
| manufacturer | `Cypress` | `Ettus Research LLC` |
| bus / address | `001` / `17` | `002` / `10` |

Only the VID/PID holds still, and that identifies the model rather than the
board. So rediscovery cannot be keyed on the serial, however natural that
sounds: `ruhd` snapshots the bus before the load and identifies the returning
device by elimination against it.

**Firmware cannot be overwritten in place.** `--force` therefore resets the FX3
first and waits for it to come back as a bootloader, which is also all `ruhd
reset` does.

## Getting the images

A B2xx has no flash, so `usrp_b200_fw.hex` and the FPGA bitstream both have to
come from somewhere. If UHD is already installed, its images are found
automatically. Otherwise:

```sh
just cli images status       # where they are looked for, and whether they are current
just cli images download     # fetch the B2xx set
```

`download` installs into `$XDG_DATA_HOME/uhd/images` — a user-owned directory
that is already first on the search path, so unlike `uhd_images_downloader` it
never needs root. It verifies each archive's SHA-256 against a manifest pinned
to the same UHD release as the reference checkout, and writes an
`inventory.json` in the format UHD's own downloader reads, so the two can share
a directory without re-fetching each other's work.

The search path is `UHD_IMAGES_DIR` (a path *list*, colon separated) followed by
the conventional install locations. `--fw <path>` and `--fpga <path>` on `init`
bypass all of it, for locally built images.

## USB permissions (udev rules)

On Linux, listing a device works for any user, but *opening* one goes through
`/dev/bus/usb/…`, which is root-only by default. The symptom is a device that
shows up in full detail with `access: permission denied` against it.

The fix is a udev rule. UHD ships one, and it is in the reference checkout:

```sh
sudo cp reference/uhd/host/utils/uhd-usrp.rules /etc/udev/rules.d/10-usrp.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then **replug the device** — udev sets permissions when a device appears, so an
already-attached board keeps whatever it was given at plug-in time.

If you do not have the submodule, the B2xx lines of that file are:

```
#B200
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2500", ATTRS{idProduct}=="0020", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2500", ATTRS{idProduct}=="0021", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2500", ATTRS{idProduct}=="0022", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2500", ATTRS{idProduct}=="0023", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3923", ATTRS{idProduct}=="7813", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3923", ATTRS{idProduct}=="7814", MODE:="0666"
```

Two things that file does not do, and one of them will eventually bite:

- **It does not cover the Cypress bootloader**, `04b4:00f3`. A board in the
  `uninitialized` state still needs root, which is why UHD's own
  `b2xx_fx3_utils` is documented as a `sudo` tool. Add
  `ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="00f3"` (and `00f0`) if you expect
  to recover blank boards as an ordinary user — bearing in mind that is
  Cypress's generic bootloader ID, so it matches any FX3-based device on the
  machine.
- **`MODE:="0666"` is world-accessible.** Substituting
  `MODE:="0660", GROUP:="usrp"` restricts it to a group, at the cost of having
  to create the group and add yourself to it.

## Licence

GPL-3.0-or-later, matching UHD. See the licensing note in
[NOTES.md](NOTES.md#licensing) — it is a decision, not a default.

[uhd]: https://github.com/EttusResearch/uhd
[just]: https://github.com/casey/just
