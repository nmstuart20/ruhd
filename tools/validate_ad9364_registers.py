#!/usr/bin/env python3
"""Run the Phase 3 AD9364 register-dump acceptance gate.

Stock UHD initializes the radio and exits without resetting it. RUHD first
opens the same device read-only and snapshots that stock state, then performs
its own matched initialization and snapshots again. The gate succeeds only
when all 1,024 addresses exist in both dumps and every differing bit is covered
by the documented allow file.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import NamedTuple


LAST_REGISTER = 0x3FF
REGISTER_COUNT = LAST_REGISTER + 1
REPO = Path(__file__).resolve().parents[1]
DEFAULT_ALLOW = Path(__file__).with_name("ad9364-register-diff.allow")
STOCK_SOURCE = Path(__file__).with_name("stock_uhd_ad9364_init.cpp")
VALIDATION_DOC = REPO / "docs" / "register-validation.md"


class Allowance(NamedTuple):
    mask: int
    uhd_version: str
    reason: str


def parse_dump(path: Path) -> dict[int, int]:
    registers: dict[int, int] = {}
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 2:
            raise ValueError(f"{path}:{line_number}: expected ADDRESS VALUE")
        address, value = (int(field, 0) for field in fields)
        if not 0 <= address <= LAST_REGISTER:
            raise ValueError(f"{path}:{line_number}: address {address:#x} is outside AD9364")
        if not 0 <= value <= 0xFF:
            raise ValueError(f"{path}:{line_number}: value {value:#x} is not a byte")
        if address in registers:
            raise ValueError(f"{path}:{line_number}: duplicate address {address:#05x}")
        registers[address] = value

    expected = set(range(REGISTER_COUNT))
    missing = sorted(expected - registers.keys())
    if missing:
        preview = ", ".join(f"{address:#05x}" for address in missing[:8])
        suffix = "..." if len(missing) > 8 else ""
        raise ValueError(
            f"{path}: incomplete dump ({len(registers)}/{REGISTER_COUNT}); "
            f"missing {preview}{suffix}"
        )
    return registers


def parse_allowances(path: Path) -> dict[int, Allowance]:
    allowances: dict[int, Allowance] = {}
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(maxsplit=3)
        if len(fields) != 4:
            raise ValueError(
                f"{path}:{line_number}: expected ADDRESS MASK UHD_VERSION_GLOB EXPLANATION"
            )
        address = int(fields[0], 0)
        mask = int(fields[1], 0)
        if not 0 <= address <= LAST_REGISTER:
            raise ValueError(f"{path}:{line_number}: address {address:#x} is outside AD9364")
        if not 0 < mask <= 0xFF:
            raise ValueError(f"{path}:{line_number}: mask must select at least one byte bit")
        if address in allowances:
            raise ValueError(f"{path}:{line_number}: duplicate address {address:#05x}")
        allowances[address] = Allowance(mask, fields[2], fields[3])
    return allowances


def recorded_uhd_version(stock_path: Path, ruhd_path: Path) -> str | None:
    for directory in dict.fromkeys((stock_path.parent, ruhd_path.parent)):
        metadata_path = directory / "run.json"
        if not metadata_path.is_file():
            continue
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        version = metadata.get("stock_uhd_version")
        if isinstance(version, str):
            return version
    return None


def compare(
    stock_path: Path,
    ruhd_path: Path,
    allow_path: Path,
    stock_uhd_version: str | None = None,
) -> bool:
    stock = parse_dump(stock_path)
    ruhd = parse_dump(ruhd_path)
    allowances = parse_allowances(allow_path)
    documentation = VALIDATION_DOC.read_text(encoding="utf-8")
    stock_uhd_version = stock_uhd_version or recorded_uhd_version(stock_path, ruhd_path)
    differences = 0
    unexplained = 0

    undocumented = []
    for address, allowance in allowances.items():
        documented_row = (
            f"| {address:#05x} | {allowance.mask:#04x} | {allowance.uhd_version} | "
            f"{allowance.reason} |"
        )
        if documented_row not in documentation:
            undocumented.append(address)
    if undocumented:
        for address in undocumented:
            print(
                f"UNDOCUMENTED ALLOWANCE {address:#05x}: matching "
                f"address/mask/version/reason row is absent from {VALIDATION_DOC.name}",
                file=sys.stderr,
            )
        print(
            f"FAIL: {len(undocumented)} allow-file entries are undocumented",
            file=sys.stderr,
        )
        return False

    for address in range(REGISTER_COUNT):
        stock_value = stock[address]
        ruhd_value = ruhd[address]
        changed = stock_value ^ ruhd_value
        if not changed:
            continue
        differences += 1
        allowance = allowances.get(address)
        version_matches = allowance is not None and (
            allowance.uhd_version == "*"
            or (
                stock_uhd_version is not None
                and fnmatch.fnmatchcase(stock_uhd_version, allowance.uhd_version)
            )
        )
        allowed_mask = allowance.mask if allowance and version_matches else 0
        unallowed_mask = changed & ~allowed_mask
        disposition = "EXPLAINED" if not unallowed_mask else "UNEXPLAINED"
        if unallowed_mask:
            unexplained += 1
        if allowance is None:
            reason = "no allow-file entry"
        else:
            if version_matches:
                reason = allowance.reason
            else:
                reason = (
                    f"allowance requires stock UHD {allowance.uhd_version}, got "
                    f"{stock_uhd_version or 'unknown'}"
                )
        print(
            f"{disposition} {address:#05x}: stock={stock_value:#04x} "
            f"ruhd={ruhd_value:#04x} changed={changed:#04x} "
            f"allowed={allowed_mask:#04x} -- {reason}"
        )

    if unexplained:
        print(
            f"FAIL: {unexplained} of {differences} differing registers contain "
            "unexplained bits",
            file=sys.stderr,
        )
        return False
    print(
        f"PASS: compared {REGISTER_COUNT} AD9364 registers; "
        f"{differences} differed and all are documented"
    )
    return True


def command(args: list[str], *, stdout=None) -> None:
    print("+ " + shlex.join(args), file=sys.stderr)
    subprocess.run(args, cwd=REPO, check=True, stdout=stdout)


def build_stock_helper(output: Path, cxx: str) -> None:
    try:
        flags = subprocess.check_output(
            ["pkg-config", "--cflags", "--libs", "uhd"], text=True
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        raise RuntimeError("stock UHD development files are unavailable via pkg-config") from error
    command([cxx, "-std=c++17", str(STOCK_SOURCE), "-o", str(output), *shlex.split(flags)])


def rust_dump_command(settings: argparse.Namespace, mode: str) -> list[str]:
    out = [
        "cargo",
        "run",
        "--quiet",
        "-p",
        "uhd-b200",
        "--example",
        "ad9364_regdump",
        "--",
        "--mode",
        mode,
        "--rate",
        str(settings.rate),
        "--rx-freq",
        str(settings.rx_freq),
        "--tx-freq",
        str(settings.tx_freq),
        "--rx-gain",
        str(settings.rx_gain),
        "--tx-gain",
        str(settings.tx_gain),
        "--bandwidth",
        str(settings.bandwidth if settings.bandwidth is not None else settings.rate),
    ]
    if settings.serial:
        out.extend(("--serial", settings.serial))
    return out


def uhd_version() -> str:
    try:
        return subprocess.check_output(
            ["pkg-config", "--modversion", "uhd"], text=True
        ).strip()
    except (FileNotFoundError, subprocess.CalledProcessError):
        return "unknown"


def run_hardware_gate(args: argparse.Namespace) -> bool:
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stock_helper = output_dir / "stock_uhd_ad9364_init"
    stock_dump = output_dir / "stock-uhd.tsv"
    ruhd_dump = output_dir / "ruhd.tsv"
    bandwidth = args.bandwidth if args.bandwidth is not None else args.rate

    build_stock_helper(stock_helper, os.environ.get("CXX", "c++"))
    command(["cargo", "build", "--quiet", "-p", "uhd-b200", "--example", "ad9364_regdump"])

    device_fields = [args.device_args, f"master_clock_rate={args.rate}"]
    if args.serial:
        device_fields.append(f"serial={args.serial}")
    device_args = ",".join(field for field in device_fields if field)
    command(
        [
            str(stock_helper),
            device_args,
            str(args.rate),
            str(args.rx_freq),
            str(args.tx_freq),
            str(args.rx_gain),
            str(args.tx_gain),
            str(bandwidth),
        ]
    )

    with stock_dump.open("w", encoding="utf-8") as output:
        command(rust_dump_command(args, "read-only"), stdout=output)
    with ruhd_dump.open("w", encoding="utf-8") as output:
        command(rust_dump_command(args, "init"), stdout=output)

    metadata = {
        "stock_uhd_version": uhd_version(),
        "device_args": device_args,
        "serial": args.serial,
        "rate": args.rate,
        "rx_freq": args.rx_freq,
        "tx_freq": args.tx_freq,
        "rx_gain": args.rx_gain,
        "tx_gain": args.tx_gain,
        "bandwidth": bandwidth,
        "stock_dump": str(stock_dump),
        "ruhd_dump": str(ruhd_dump),
        "allow_file": str(args.allow.resolve()),
    }
    (output_dir / "run.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return compare(
        stock_dump,
        ruhd_dump,
        args.allow,
        stock_uhd_version=metadata["stock_uhd_version"],
    )


def parser() -> argparse.ArgumentParser:
    out = argparse.ArgumentParser(description=__doc__)
    out.add_argument("--serial", help="select one B2xx by serial")
    out.add_argument("--device-args", default="type=b200", help="extra stock UHD device args")
    out.add_argument("--rate", type=float, default=16e6, help="sample/master-clock rate in Hz")
    out.add_argument("--rx-freq", type=float, default=2.45e9, help="RX LO frequency in Hz")
    out.add_argument("--tx-freq", type=float, default=2.45e9, help="TX LO frequency in Hz")
    out.add_argument("--rx-gain", type=float, default=37.0, help="manual RX gain in dB")
    out.add_argument("--tx-gain", type=float, default=10.0, help="TX gain in dB")
    out.add_argument(
        "--bandwidth", type=float, help="RX/TX analog bandwidth in Hz (default: rate)"
    )
    out.add_argument("--allow", type=Path, default=DEFAULT_ALLOW, help="documented bit allowances")
    out.add_argument(
        "--output-dir",
        type=Path,
        default=REPO / "target" / "ad9364-register-validation",
        help="directory for helpers, dumps, and run metadata",
    )
    out.add_argument(
        "--diff-only",
        nargs=2,
        metavar=("STOCK_DUMP", "RUHD_DUMP"),
        type=Path,
        help="compare existing dumps without touching hardware",
    )
    return out


def main() -> int:
    args = parser().parse_args()
    try:
        if args.diff_only:
            passed = compare(args.diff_only[0], args.diff_only[1], args.allow)
        else:
            passed = run_hardware_gate(args)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
