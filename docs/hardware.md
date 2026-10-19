# Hardware setup

## Linux USB permissions

Run `just cli probe usb`. If it reports `access: ok`, no setup is needed.
Otherwise install the supplied udev rules, then unplug and reconnect the board:

```sh
just udev-install
```

This recipe uses `sudo` to install the rules. Local desktop users receive access
through systemd-logind. For SSH, headless use, or a service, add the account to
the `usrp` group and log in again:

```sh
sudo usermod -aG usrp "$USER"
```

Blank boards may enumerate as generic Cypress FX3 bootloaders. For those only,
use `just udev-install-bootloader`; this opt-in rule also matches non-USRP FX3
devices. `just udev-uninstall` removes both supplied rules.

## Firmware and FPGA images

B2xx firmware and FPGA images are volatile and must be loaded after each
plug-in. Installed UHD images are found automatically. Otherwise:

```sh
just cli images status
just cli images download
just cli init
```

Downloads go to `$XDG_DATA_HOME/uhd/images` (default
`~/.local/share/uhd/images`) without root. `UHD_IMAGES_DIR` is a colon-separated
list of additional search directories on Linux; `init --fw PATH --fpga PATH`
selects explicit files. `images sync --device b205mini --release v4.10.0.0`
populates the separate verified cache used by compatibility negotiation and
the hardware tests.

## Troubleshooting

- `probe usb` sees pre-firmware devices; `find` reports firmware-ready boards.
  Run `init` if a newly connected board is missing from `find`.
- Use `probe --full` for raw firmware/FPGA/EEPROM compatibility information and
  `doctor` for the backend's compatibility verdict. Stop other applications
  using the board before opening it.
- `init --force-fpga` reloads the FPGA. `init --force` resets the FX3 and reloads
  firmware too. `reset` returns the board to its bootloader.
- Use release builds for streaming. RX prints `O` for overflow and `D` for a
  sequence gap; reduce the rate if the disk or downstream consumer cannot keep
  up. TX prints `U` for underflow.

See the [README](../README.md) for run and test commands.
