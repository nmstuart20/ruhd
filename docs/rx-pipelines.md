# Piping RX to stdout

Use `--file -` to send raw I/Q samples to stdout; status messages go to stderr.
The default `--type sc16` writes interleaved signed 16-bit I/Q (4 bytes per
complex sample). `--type fc32` writes complex 32-bit floats (8 bytes per sample),
which NumPy can read as `complex64`. Both use host byte order with no header.
With neither `--duration` nor `--nsamps`, reception runs until Ctrl-C. Closing
the downstream reader also stops reception cleanly.

`--profile low-latency|balanced|capture` configures packet size, queued USB
transfers, application block size, and the application buffer pool together.
Stdout defaults to `low-latency`; regular files default to `capture`.
Explicit `--profile` works with either output target. `--spp` and
`--num-recv-frames` override the corresponding profile settings.

| Profile | Samples/packet | USB transfers | Block size | Blocks / pool size | Block fill at 1 MS/s, sc16 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `low-latency` | 256 | 16 | 16 KiB | 4 / 64 KiB | 4.096 ms |
| `balanced` | 1024 | 64 | 256 KiB | 8 / 2 MiB | 65.536 ms |
| `capture` | 2040 | 128 | 2 MiB | 32 / 64 MiB | 524.288 ms |

Choose `low-latency` for interactive decoding or audio, `balanced` for
moderate batching, and `capture` for recording with more room for output
stalls. For example, select capture buffering for a compression pipeline:

```sh
just rx --freq 2.45e9 --rate 20e6 --duration 30 --profile capture --file - \
  | zstd -1 -T0 -o wifi.sc16.zst
```

Startup diagnostics print the effective packet size after driver coercion,
USB transfer count, block size, and pool size. Budgets use the actual sample
rate: packet fill time is `spp / rate`, block fill time is
`block_bytes / bytes_per_sample / rate`, USB sample capacity is
`spp * transfers / rate`, and pool sample capacity is
`block_samples * blocks / rate`. Blocks use host-format bytes, so `fc32`
and `fc64` fill the same byte-sized blocks in half and a quarter of the
`sc16` time. USB still carries `sc16`.

These are sample-time batching and capacity budgets, not end-to-end latency
guarantees. Scheduling, USB service, downstream buffering, and output stalls
add delay. USB capacity describes sample payload, not allocated transfer
memory, and queued transfers are headroom rather than a required batch.
Small packets increase host work per sample; use the larger profiles when
testing throughput near the device's maximum rate.

Once a second, and after the writer finishes, stderr reports current and peak
**application backlog** in bytes and sample-time milliseconds. It includes
the partially filled receive block, queued blocks, and a block being written.
Bytes leave the backlog only after `write_all` completes. It excludes samples
still in USB/device buffers and bytes already accepted by the OS or downstream
reader. On output failure it conservatively includes an incomplete write's
whole block. The final summary also reports how often the block pool was
exhausted and total time the receive loop waited for a free block. A rising
backlog signals output pressure before the pool fills and RX has to wait.
Ctrl-C stops the receive loop during that wait; shutdown drains accepted
blocks, so a stalled output may still delay exit.

Listen to broadcast FM with [csdr](https://github.com/ha7ilm/csdr#demodulate-wfm-advanced)
and ALSA's `aplay`. Set `--freq` to a local station. `--type fc32` matches
csdr's complex-float input; decimating 2.4 MS/s by 10 and then 5 produces
48 kHz mono audio:

```sh
just rx --freq 98.1e6 --rate 2.4e6 --gain 40 --type fc32 --file - \
  | csdr fir_decimate_cc 10 0.05 HAMMING \
  | csdr fmdemod_quadri_cf \
  | csdr fractional_decimator_ff 5 \
  | csdr deemphasis_wfm_ff 48000 75e-6 \
  | csdr convert_f_s16 \
  | aplay -f S16_LE -r 48000 -c 1
```

The example uses 75 microseconds of FM de-emphasis, as used in Europe; use
`50e-6` instead of `75e-6` for Europe.

Save a capture while feeding a decoder that accepts raw `sc16` on stdin
(replace `my-decoder` with your tool):

```sh
just rx --freq 433.92e6 --rate 2e6 --file - --meta ism.sigmf-meta \
  | tee ism.sigmf-data | my-decoder
```

Compress a capture on the fly, then decompress it for use with SigMF tools:

```sh
just rx --freq 2.45e9 --rate 20e6 --duration 30 --profile capture --file - --meta wifi.sigmf-meta \
  | zstd -1 -T0 -o wifi.sc16.zst
zstd -dc wifi.sc16.zst > wifi.sigmf-data
```

Capture from a radio attached to another machine and save the samples locally
(`ruhd` must be available on the remote host):

```sh
ssh sdr-host 'ruhd rx --freq 1090e6 --rate 2e6 --duration 60 --profile capture --file -' > adsb.sc16
```

Inspect mean sample power live with Python and NumPy:

```sh
just rx --freq 98.1e6 --rate 2e6 --type fc32 --file - | python3 -c '
import sys, numpy as np
n = 1 << 16
while (buf := sys.stdin.buffer.read(n * 8)):
    x = np.frombuffer(buf, dtype=np.complex64)
    print(f"{10 * np.log10(np.mean(np.abs(x) ** 2) + 1e-20):7.2f} dBFS", flush=True)
'
```

Stdout captures only write SigMF metadata when `--meta PATH` is given. For SSH
captures, that path is on the remote host. Keep stderr separate from stdout
so diagnostics do not enter the sample stream. A consumer that cannot keep up
will eventually cause overflows (`O` on stderr); reduce `--rate` if needed.
