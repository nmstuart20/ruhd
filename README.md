# ruhd

A pure-Rust reimplementation of the [Ettus USRP Hardware Driver][uhd], targeting
the USRP B205mini first.

**Status: scaffolding.** The traits and value types are in place and the CLI
runs, but no hardware backend exists yet, so `ruhd find` finds nothing. See
[NOTES.md](NOTES.md) for the design rationale, the reference material, and the
plan.

## Layout

| Crate | Contents |
| --- | --- |
| `uhd-core` | The object-safe `Device` trait, streamers, and the value types they trade in. No I/O. |
| `uhd-transport` | Link, register and serial-bus traits. Traits only for now. |
| `chdr` | Packet framing. Empty. |
| `ad936x` | AD9361/AD9364 transceiver driver. Empty. |
| `uhd-b200` | B2xx backend, including the B205mini. Empty. |
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

## Licence

GPL-3.0-or-later, matching UHD. See the licensing note in
[NOTES.md](NOTES.md#licensing) — it is a decision, not a default.

[uhd]: https://github.com/EttusResearch/uhd
[just]: https://github.com/casey/just
