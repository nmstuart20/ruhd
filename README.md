# ruhd

A pure-Rust reimplementation of the [Ettus USRP Hardware Driver][uhd], targeting
the USRP B205mini first.

**Status: early.** The traits and value types are in place, and the USB layer
works against real hardware: `ruhd probe usb` finds and classifies a B2xx, and
`ruhd init` takes one from freshly plugged in to FX3 firmware running, FPGA
configured, and a live settings bus. Stock `uhd_usrp_probe` then opens the same
device without reloading anything. `ruhd find` discovers firmware-ready B2xx
devices, and `ruhd rx` records IQ from one (single channel, receive only). The
generic `Device` implementation and transmit are not done yet. See
[NOTES.md](NOTES.md) for the design rationale, the reference material, and the
plan.

## Layout

| Crate | Contents |
| --- | --- |
| `uhd-core` | The object-safe `Device` trait, streamers, and the value types they trade in. No I/O. |
| `uhd-transport` | Link, register and serial-bus traits; USB discovery, FX3 firmware and FPGA loading, the EEPROM map and settings-bus access (`usb`); image path resolution (`images`). |
| `chdr` | Legacy CVITA/CHDR packet framing: pack, unpack, and a zero-copy header parser. |
| `ad936x` | AD9361/AD9364 transceiver driver: complete initialization, RF/sample-rate tuning, gain tables and AGC, FIR/analog filters, and DC/quadrature calibration. |
| `uhd-b200` | B2xx discovery, compatibility policy, open session, settings-bus peripherals, AD936x board integration, and the receive streamer. |
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

To talk to a board on Linux without `sudo` you need a udev rule. Run `just cli
probe usb`: if it reports `access: ok` you are already set; otherwise see
[USB permissions](#usb-permissions-udev-rules).

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

## Recording IQ

```sh
just cli init                                   # once per plug-in
just rx --freq 98e6 --rate 8e6 --gain 40 --duration 2 --file fm.sc16
just rx --freq 2.45e9 --rate 20e6 --nsamps 10000000 --type fc32 --file wifi.fc32
just rx --freq 433.92e6 --rate 2e6 --file ism.sigmf-data   # until Ctrl-C
just rx --freq 98e6 --rate 2e6 --duration 10 --file - | csdr ...   # to stdout
```

`just rx` is `ruhd rx` built with `--release`, because a debug build cannot
keep up above a few MS/s. The file is raw interleaved I/Q in host byte order
with no header, the same layout UHD's `rx_samples_to_file` writes, so anything
that reads those reads these: `numpy.fromfile(f, dtype=numpy.int16)` for
`sc16` (the default), `numpy.complex64` for `fc32`. Other options: `--bw`,
`--ant RX2|TX/RX`, `--master-clock-rate` (chosen from `--rate` the way UHD does
if omitted), `--spp`, `--num-recv-frames`, `--serial`.

With neither `--duration` nor `--nsamps`, recording runs until Ctrl-C, which
stops the stream cleanly and keeps everything recorded; a second Ctrl-C exits
at once. `--file -` writes samples to stdout (all messages go to stderr), and a
reader that closes the pipe ends the recording normally.

### Piping to other tools

Because messages go to stderr, `--file -` can feed straight into anything that
reads raw IQ on stdin. Some useful pipelines:

**Listen to broadcast FM** with [csdr][csdr]. At 2.4 MS/s, decimating by 10 and
then 5 lands on 48 kHz audio; `--type fc32` matches csdr's complex-float input,
so no conversion step is needed:

```sh
just rx --freq 98.1e6 --rate 2.4e6 --gain 40 --type fc32 --file - \
  | csdr fir_decimate_cc 10 0.05 HAMMING \
  | csdr fmdemod_quadri_cf \
  | csdr fractional_decimator_ff 5 \
  | csdr deemphasis_wfm_ff 48000 50e-6 \
  | csdr convert_f_s16 \
  | aplay -f S16_LE -r 48000 -c 1
```

**Record and process at the same time** with `tee`, so a live pipeline does
not cost you the capture:

```sh
just rx --freq 433.92e6 --rate 2e6 --file - --meta ism.sigmf-meta \
  | tee ism.sigmf-data | my-decoder
```

**Compress on the fly.** Raw IQ is large (61.44 MS/s of `sc16` is about
245 MB/s), and a fast `zstd` level usually keeps up while shrinking quiet
bands a lot:

```sh
just rx --freq 2.45e9 --rate 20e6 --duration 30 --file - --meta wifi.sigmf-meta \
  | zstd -1 -T0 -o wifi.sc16.zst
zstd -dc wifi.sc16.zst > wifi.sigmf-data   # later, to get the raw file back
```

**Capture on a remote machine** with the radio attached, and write the
samples locally:

```sh
ssh sdr-host 'ruhd rx --freq 1090e6 --rate 2e6 --duration 60 --file -' > adsb.sc16
```

The metadata flag, if any, is evaluated on the remote side, so `--meta` there
writes the file there.

**Inspect samples live in Python**, for example a running power meter. The
script reads fixed-size blocks from stdin; when it exits, the closed pipe stops
the recording cleanly:

```sh
just rx --freq 98.1e6 --rate 2e6 --type fc32 --file - | python3 -c '
import sys, numpy as np
n = 1 << 16
while (buf := sys.stdin.buffer.read(n * 8)):
    x = np.frombuffer(buf, dtype=np.complex64)
    print(f"{10 * np.log10(np.mean(np.abs(x) ** 2) + 1e-20):7.2f} dBFS")
'
```

Pipelines have no disk to buffer for them, so a consumer slower than the
sample rate will cause overflows (`O` on stderr). Lower `--rate`, or put a
buffer such as `mbuffer -m 1G` in front of the slow stage.

Each recording gets a [SigMF](https://sigmf.org) metadata file: the data
file's name with its extension replaced by `.sigmf-meta`. It carries the sample
format, rate, frequency, start time and hardware, with gain, bandwidth,
antenna, master clock and the device time of the first sample under a `ruhd:`
extension. Name the data file `*.sigmf-data` and SigMF tools pick the pair up
directly. `--meta PATH` puts the metadata elsewhere (it is the only way to get
it with `--file -`) and `--no-meta` turns it off.

Overflows print `O` and sequence gaps `D`, as in UHD, and are counted at the
end. Unlike UHD, each one is also recorded as a SigMF annotation at the sample
index where the discontinuity falls, so you know which parts of a capture to
distrust. On the bench B205mini over USB 3 a recording runs clean to disk at the
AD9364's full 61.44 MS/s.

## Benchmarking throughput

```sh
just bench --rx-rate 61.44e6 --duration 10
just bench --rx-rate 10e6 --rx-cpu sc16 --overrun-threshold 0   # fail on any overrun
```

`ruhd benchmark` is the receive half of UHD's `benchmark_rate`: it streams
into a buffer that is thrown away and prints the same summary block, plus two
cross-checks: the receive rate it actually achieved, and how many times a
packet's device timestamp did not follow on from the previous one without an
overflow to explain it (samples lost silently; this should always be 0). The options take UHD's names too
(`--rx_rate`, `--rx_spp`, `--rx_spb`, `--rx_cpu`, `--rx_delay`, `--random`,
`--overrun-threshold`, `--drop-threshold`), and the command answers to
`benchmark_rate` as well. Transmit is not implemented yet, so the TX lines are
always zero; `--priority` and multi-channel options are not supported.

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

On Linux, any user can *list* a USB device, but *opening* one goes through
`/dev/bus/usb/…`, which only root can write to unless a udev rule says
otherwise. Without one, `init`, `rx` and `reset` fail, `find` skips the board
with a warning, and the tempting workaround, `sudo cargo run`, leaves
root-owned files in `target/` and puts the images cache in root's home.

**Check first. You may already be set up.**

```sh
just cli probe usb
```

If the device shows `access:  ok`, there is nothing to do. That is common:
distro UHD packages (Fedora, EPEL, Debian/Ubuntu `libuhd`) install their own
rule, usually `/usr/lib/udev/rules.d/10-usrp-uhd.rules`. If it shows
`permission denied`, install ruhd's rules:

```sh
just udev-install        # copies udev/60-ruhd-b2xx.rules, reloads udev
```

Then **replug the board.** udev applies rules when a device appears, and the
recipe's `udevadm trigger` does not always reach a board that is already
attached. Without `just`, the recipe is:

```sh
sudo groupadd --system --force usrp
sudo install -m 0644 udev/60-ruhd-b2xx.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

### How access is granted, and when it is not

The rule (like UHD's packaged one) grants access two ways:

- **Seat access.** systemd-logind gives the user logged in at the machine's
  local seat (the desktop session) read/write access automatically. This is why
  things usually just work on a workstation, even though you are not in any
  special group.
- **The `usrp` group.** Seat access does not apply over SSH, on a headless box,
  or for a service or CI runner. There you need to join the group:

  ```sh
  sudo usermod -aG usrp $USER    # then log out and back in
  ```

So "it works on my desktop" and "it works over SSH" can have different answers
on the same machine.

### Blank boards (the FX3 bootloader)

A board whose EEPROM is blank or erased enumerates as Cypress's generic FX3
bootloader, `04b4:00f3` (or `04b4:00f0`), not as a USRP. No standard rule
covers those IDs, UHD's included, so recovering such a board needs root even
where everything else works. That is also why UHD documents `b2xx_fx3_utils`
as a `sudo` tool. To recover blank boards as an ordinary user:

```sh
just udev-install-bootloader   # udev/61-ruhd-fx3-bootloader.rules
```

This is opt-in because those IDs are not USRP-specific: the rule matches *any*
FX3-based device on the machine while it is in its bootloader.

`just udev-uninstall` removes both files.

## Licence

GPL-3.0-or-later, matching UHD. See the licensing note in
[NOTES.md](NOTES.md#licensing) — it is a decision, not a default.

[uhd]: https://github.com/EttusResearch/uhd
[just]: https://github.com/casey/just
[csdr]: https://github.com/jketterl/csdr
