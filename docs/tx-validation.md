# TX bench validation plan

TX has software tests; RF and throughput validation are pending. Run these
steps once the attenuation and receiver hardware are available. No TX hardware
tests have been run for this implementation.

1. Connect the B205mini TX/RX port by coax through attenuation to the receiver.
   Choose attenuation for the transmitter output and receiver input limits;
   start at TX gain 0 dB and waveform amplitude 0.1. Use a common reference if
   available, or account for the radios' relative frequency error in analysis.
2. Initialize the board with `just cli init`. Set `RUHD_HW_SERIAL` if there is
   more than one board. Stop other applications using the B205mini.
3. Run the explicitly opt-in protocol test:

   ```sh
   cargo test -p uhd-b200 --features hw-tests --test hw_tx -- --ignored --test-threads 1 --nocapture
   ```

   It checks a timed 1001-sample burst using odd packet sizes, burst ACK,
   late-packet reporting, control readbacks during TX, exclusive stream
   ownership, reopening, and keeping the stream alive after dropping Device.
   This checks protocol behavior; an ACK alone does not prove RF correctness.
4. Capture the tone from the example with the receiver at 915 MHz and at least
   1 MS/s:

   ```sh
   cargo run --release -p uhd-b200 --example transmit -- [serial]
   ```

   The burst contains 10,000 samples at 1 MS/s, amplitude 0.1, with a complex
   tone at positive 62.5 kHz. Check its frequency sign, duration and spectrum.
   Repeat with an LO offset and verify that the intended RF center remains
   correct; the public TuneResult subtracts the logical DSP offset, while TX
   hardware adds its physical upconverter shift.
5. Generate a reusable IQ file and play identical samples with `ruhd tx` and
   stock UHD's `tx_samples_from_file`, matching clock, rate, RF frequency, gain,
   bandwidth and host format. Compare measured frequency, amplitude, images,
   LO leakage and burst envelope. Repeat for sc16, fc32 and fc64, including
   near-full-scale inputs to check conversion and clipping. `ruhd tx` appends
   one zero sample in its separate EOB packet; the finite Rust example places
   EOB on the last data packet and does not append that sample.
6. Sweep 1, 16, 30.72 and 61.44 MS/s in release mode, first sc16 and then floats.
   Record duration, underflow/sequence/time errors, receiver discontinuities,
   USB speed, send-frame count, host load and the actual coerced clock/rate.
   The current `ruhd benchmark` command measures RX only. Test a deliberately
   slow input to exercise underflow reporting and recovery, then repeated
   bursts and stream drop/reopen to look for stale data or events.
7. Record the board serial, firmware/FPGA compatibility, UHD version, receiver,
   attenuation, test commands and measurements in this document. Compare the same
   waveform against UHD before claiming RF correctness or a maximum TX rate.
