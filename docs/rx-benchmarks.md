# RX loss accounting and benchmarks

Use a release build for throughput measurements. This records a report and
fails if any known samples are missing or any loss count remains unknown:

```sh
just bench --rx-rate 30.72e6 --rx-cpu sc16 --duration 60 \
  --num-recv-frames 128 --loss-threshold 0 --unknown-loss-threshold 0 \
  --json rx-30m-sc16.json
```

`--overrun-threshold` and `--drop-threshold` separately gate device overflow
and sequence-error notifications. A sequence anomaly can have zero missing
samples according to the timestamps; use those thresholds too if any such
notification should fail the run. Thresholds fail when exceeded, not when met.

## What counts as loss

The B2xx streamer checks both the packet sequence counter and integer device
timestamps. Timestamp checks happen within each burst, including when the
12-bit sequence counter wraps. Gaps between completed finite bursts are
intentional and are excluded, including with `benchmark --random`.

An overflow, bad packet, or timestamp jump produces an error notification.
The first subsequent receive that returns samples carries `RxMetadata.loss`:

- `cause`: the first indication of that discontinuity, not a diagnosis of its
  physical cause. Several notifications before recovery form one loss episode.
- `expected_ticks` and `resumed_ticks`: the expected and actual timestamps at
  the boundary, as integer device ticks.
- `tick_rate` and `ticks_per_sample`: the units and sample spacing.
- `missing_samples`: the exact forward gap divided by sample spacing, or
  `None` if either timestamp is absent, time moved backward, or the gap does
  not align to whole samples. A sequence anomaly can yield `Some(0)`.
- `duration_secs()`: the forward gap duration when both timestamps are known.

Overflow recovery preserves the expected timestamp across flushing, so the
gap includes samples discarded during recovery. No count is guessed from
sequence numbers or replaced with a nominal one-sample loss. Counts rely on
the device clock and configured sample spacing remaining valid.

The benchmark's known missing sample total excludes episodes with unknown
counts. These are reported separately. An error with no subsequent recovered
samples remains one unresolved, unknown episode, even if the run fails.
Received sample and loss totals include the final drain after stopping;
achieved throughput uses samples received before the stop command.

`ruhd rx` attaches these same details to SigMF annotations at the file index
of the first recovered sample. Global `ruhd:known_missing_samples`,
`ruhd:unknown_loss_episodes`, `ruhd:loss_episodes`, and
`ruhd:known_gap_seconds` summarize the recording. The existing
`ruhd:discontinuities` field counts notification annotations, which can exceed
the number of loss episodes. Missing samples are omitted from the data file;
they are not replaced with zeros. Tick fields are JSON integers; consumers
should preserve 64-bit integer precision when parsing them.

## Instrumentation

`--json PATH` writes a report with `schema_version: 1`, build version, host
OS/architecture, device identity, requested and applied sample rates, master
clock rate, host/wire formats, effective packet size, receive buffer size,
USB transfer count, thresholds, and results. A report is also written after a
streaming failure, with `completed: false`, the error, and partial statistics.
Setup failures before streaming do not produce reports. Threshold failures
set `passed: false` and return a failing exit status.

The additional measurements are:

| Measurement | Meaning |
| --- | --- |
| Achieved rate | Samples before stop divided by streaming wall time, excluding the configured initial start delay. |
| Receive call timing | Blocking `recv` duration, including conversion, device waits, and the initial scheduled wait. Shutdown drain calls are excluded. Failed calls are included. |
| p50/p95/p99 upper bounds | Conservative logarithmic histogram bounds in microseconds; not exact percentiles. Fixed storage avoids per-call allocations. |
| First samples after command | Time until the first nonempty receive returns, including any scheduled start delay. |
| Short/empty/failed reads | Nonempty reads smaller than requested, zero-sample reads, and calls returning an error. |
| CPU time/percent | Process user plus system CPU time over the benchmark receive function, including buffer setup and shutdown drain, excluding device initialization. Percent can exceed 100 with multiple active threads. |
| Peak RSS | Process lifetime peak resident memory, including initialization, in bytes. Linux and macOS are supported; unavailable resource values are JSON `null`. |

Receive call timing is host-side instrumentation, not antenna-to-application
latency. Each call adds a timer and a histogram update. Compare instrumented
builds with the same settings, and measure overhead before attributing small
performance differences to the driver.

## Comparing runs

Use the same board, USB port/controller, FPGA image, frequency, sample rate,
host format, samples per packet, receive size, and transfer count. Compare
both default UHD and UHD with `num_recv_frames=128`; a larger queue alone is
a configuration improvement, not evidence of a faster implementation.

Run each configuration several times with `sc16` and `fc32`, first on an idle
host and then under the same controlled CPU load. Keep the reports with the
UHD version, image version, host description, and load conditions. Sweep rates
and queue depths rather than judging the drivers by a single maximum-rate
run. Check unknown counts and timestamp jumps alongside throughput: a high
reported rate alone does not establish continuous reception.

The receive benchmark discards samples. It does not measure disk throughput,
downstream decoder speed, or RF fidelity; validate those with recording and
signal-based tests separately.
