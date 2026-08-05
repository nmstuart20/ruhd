//! Pushing an FPGA bitstream through the FX3 into the FPGA.
//!
//! Mirrors `b200_iface_impl::load_fpga` in `b200_iface.cpp`, plus the GPIF
//! reset that `b200_impl`'s constructor performs immediately afterwards.
//!
//! # Why this exists at all
//!
//! A B2xx has no configuration flash. The FPGA comes up blank on every power
//! cycle, and the host clocks the bitstream in over USB — the FX3 drives the
//! FPGA's slave-serial configuration pins, and each vendor request hands it
//! another few hundred bytes to shift out. So an FPGA load is not an occasional
//! maintenance operation, it is part of opening the device.
//!
//! # The sequence
//!
//! ```text
//!   fpga_hash            -> what the FX3 says is loaded; equal and not forced
//!                           means there is nothing to do
//!   transfer_size        -> 512 over USB 3, 64 over USB 2
//!   loopback             -> proves the firmware's buffer is that large
//!   set_fpga_hash(0)     -> "nothing is loaded", in case this dies half way
//!   start_fpga_config    -> poll until FpgaReady          (up to ~5 s)
//!   start_fpga_bitstream -> poll until ConfiguringFpga    (up to ~10 s)
//!   write_fpga_data      -> the whole image, one chunk per control transfer
//!                           poll until Running            (up to ~5 s)
//!   set_fpga_hash(hash)  -> "this image is loaded"
//!   reset_gpif           -> the FX3's data path is stale; start it over
//! ```
//!
//! [`configure`] is that whole list. [`load_fpga`] is everything up to the
//! GPIF reset, kept separate only because that is where UHD's own seam is:
//! `load_fpga` lives in `b200_iface.cpp` and `reset_gpif()` is the next line of
//! `b200_impl`'s constructor.
//!
//! # Version-agnostic on purpose
//!
//! Nothing here looks at what is *in* the bitstream, and nothing here compares
//! a version number. The loader pushes the bytes it is given and reports what
//! the device says afterwards; deciding whether that FPGA build is acceptable
//! belongs to a policy layer, and the input to that decision is
//! [`CompatReport`](super::compat::CompatReport). This is a deliberate
//! departure from `b200_impl.cpp`, which throws on a compatibility-number
//! mismatch from inside device construction and leaves the caller no way to
//! say "load it anyway".

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use tracing::{debug, info, trace};
use uhd_core::{Error, Result};

use super::fx3::Fx3Iface;
use super::probe::DeviceState;
use super::{Fx3State, UsbProbe};

/// How long to leave between FX3 status polls while configuration proceeds.
///
/// 10 ms, as in `load_fpga`.
pub const FX3_POLL_INTERVAL: Duration = Duration::from_millis(10);

/// Status polls allowed while waiting for the FPGA to enter configuration mode.
///
/// The `wait_count >= 500` bound in `load_fpga`'s first and third wait loops:
/// 500 polls at [`FX3_POLL_INTERVAL`], so about five seconds.
pub const FX3_CONFIG_POLLS: u32 = 500;

/// Status polls allowed while waiting for the FX3 to start accepting bitstream.
///
/// The `wait_count >= 1000` bound in `load_fpga`'s second wait loop.
pub const FX3_BITSTREAM_POLLS: u32 = 1000;

/// An FPGA bitstream read into memory, with the hash UHD would give it.
///
/// Read once and load into as many devices as are attached: the bytes are the
/// same and so is the hash. The image is held in memory rather than streamed
/// from the file because the hash has to be computed over all of it before the
/// first byte is pushed, so it has been read once already by then.
#[derive(Debug, Clone)]
pub struct FpgaImage {
    /// The raw bitstream, exactly as it sits on disk.
    pub data: Vec<u8>,
    /// [`image_hash`] of `data`.
    pub hash: u32,
    /// Where it was read from, for messages.
    pub path: PathBuf,
}

impl FpgaImage {
    /// Reads a bitstream from disk.
    ///
    /// No validation beyond "not empty". A `.bin` bitstream is raw
    /// configuration data with no header this code could check, and refusing to
    /// load an image because it fails a guess about its format would be exactly
    /// the version-coupling this module is trying to avoid. A wrong image
    /// fails at the FX3, which reports it.
    ///
    /// # Errors
    ///
    /// [`Error::Io`] if the file cannot be read; [`Error::Firmware`] if it is
    /// empty, which UHD's `generate_hash` also treats as an error.
    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let data = std::fs::read(path)?;
        if data.is_empty() {
            return Err(Error::Firmware(format!(
                "fpga image {} is empty",
                path.display()
            )));
        }
        let hash = image_hash(&data);
        debug!(
            path = %path.display(),
            bytes = data.len(),
            hash = format_args!("{hash:#010x}"),
            "loaded fpga bitstream"
        );
        Ok(Self {
            hash,
            data,
            path: path.to_path_buf(),
        })
    }

    /// How many bytes the bitstream is.
    #[must_use]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether the bitstream is empty. Never true for an image from
    /// [`load`](Self::load), which rejects one.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

/// UHD's `generate_hash`, byte for byte.
///
/// This is not a checksum in any useful sense — it is a Boost `hash_combine`
/// applied one byte at a time — and it is reproduced exactly rather than
/// replaced with something better **because the FX3 is a shared mailbox**.
/// UHD's `load_fpga` reads the stored hash, computes this over the file it was
/// about to send, and skips the load if they match. So if this driver stores a
/// hash UHD would not have computed, a `uhd_usrp_probe` after a `ruhd` load
/// reconfigures the FPGA from scratch — which works, but takes several seconds
/// and means the two drivers cannot share a device usefully. Matching the
/// function is what makes the handover free.
///
/// ```text
/// hash = 0
/// for each byte: hash ^= byte + 0x9e3779b9 + (hash << 6) + (hash >> 2)
/// ```
///
/// All arithmetic is 32-bit and wrapping. The one subtlety is that UHD reads
/// the file into a `char`, so the addend is *sign-extended* on any platform
/// where C's `char` is signed — every platform this driver targets. On a
/// platform where it is unsigned (ARM Linux, say) a UHD built there would
/// compute a different value, and the only consequence is that it reloads the
/// FPGA once; the load itself is unaffected, since nothing but UHD ever
/// interprets this number.
#[must_use]
pub fn image_hash(data: &[u8]) -> u32 {
    let mut hash: u32 = 0;
    for &byte in data {
        // `as i8 as i32 as u32` is C's `char` -> `int` -> `unsigned int` chain
        // on a signed-char platform, which is where the sign extension enters.
        let term = (byte as i8 as i32 as u32).wrapping_add(0x9e37_79b9);
        hash ^= term.wrapping_add(hash << 6).wrapping_add(hash >> 2);
    }
    hash
}

/// What [`load_fpga`] did.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct FpgaLoad {
    /// Whether the bitstream was actually pushed, as opposed to skipped
    /// because the hash the FX3 held already matched.
    pub loaded: bool,
    /// The hash now recorded on the device.
    pub hash: u32,
    /// The hash the device held before, which is what the skip decision was
    /// made on. Zero means "nothing loaded, or a load was interrupted".
    pub previous_hash: u32,
    /// Bytes pushed. Zero on a skip.
    pub bytes: usize,
    /// Chunk size used, i.e. what [`Fx3Iface::transfer_size`] reported.
    pub chunk_size: usize,
    /// The FX3 state machine's state afterwards.
    pub state: Fx3State,
    /// How long the whole thing took.
    pub took: Duration,
}

/// Loads a bitstream into the FPGA, or reports that it was already there.
///
/// The sequence is in the [module docs](self). `progress` is called after each
/// chunk with `(bytes_sent, total_bytes)` so a caller can draw a bar; it is
/// never called at all on a skip.
///
/// # The hash shortcut
///
/// If the FX3 is holding [`image.hash`](FpgaImage::hash) already and `force` is
/// false, this returns immediately with `loaded: false`. That is UHD's
/// behaviour and it is the reason a second `uhd_usrp_probe` is fast. It is a
/// claim by the *host* that a matching image was loaded, not evidence from the
/// FPGA — so `force` exists for the case where the FPGA has been reconfigured
/// by something that did not update the note, and the hash is zeroed before the
/// first byte goes out so that a load interrupted half way cannot leave a
/// truthful-looking note over a broken image.
///
/// # Errors
///
/// [`Error::Usb`] if any transfer fails. [`Error::Firmware`] if the FX3
/// reports [`Error`](Fx3State::Error) or [`Undefined`](Fx3State::Undefined)
/// while configuration is in progress, or does not reach the expected state in
/// time — in both cases the FPGA is left unconfigured with a zero hash, which
/// is honest about what happened.
pub fn load_fpga(
    iface: &Fx3Iface,
    image: &FpgaImage,
    force: bool,
    mut progress: impl FnMut(usize, usize),
) -> Result<FpgaLoad> {
    let started = Instant::now();
    let previous_hash = iface.fpga_hash()?;

    if previous_hash == image.hash && !force {
        debug!(
            hash = format_args!("{:#010x}", image.hash),
            "fpga image already loaded"
        );
        return Ok(FpgaLoad {
            loaded: false,
            hash: image.hash,
            previous_hash,
            bytes: 0,
            chunk_size: 0,
            state: iface.fx3_state()?,
            took: started.elapsed(),
        });
    }

    let chunk_size = iface.transfer_size()?;
    // The loopback is a fixed-size read that only succeeds if the firmware's
    // control buffer is at least `chunk_size`. Doing it here means a host that
    // has overestimated the chunk size finds out before it has put the FPGA
    // into configuration mode.
    let probe_len = u16::try_from(chunk_size)
        .map_err(|_| Error::internal(format!("implausible transfer size {chunk_size}")))?;
    iface.loopback(probe_len)?;

    info!(
        path = %image.path.display(),
        bytes = image.len(),
        chunk_size,
        hash = format_args!("{:#010x}", image.hash),
        "loading fpga bitstream"
    );

    // Zero first: from here until the last line the device holds no valid
    // image, and the note has to say so, or an interrupted load leaves the next
    // caller skipping a load it badly needs.
    iface.set_fpga_hash(0)?;

    iface.start_fpga_config()?;
    wait_for_state(
        iface,
        Fx3State::FpgaReady,
        FX3_CONFIG_POLLS,
        "enter configuration mode",
    )?;

    iface.start_fpga_bitstream()?;
    wait_for_state(
        iface,
        Fx3State::ConfiguringFpga,
        FX3_BITSTREAM_POLLS,
        "start accepting bitstream",
    )?;

    let total = image.len();
    let mut sent = 0usize;
    let mut last_decile = 0usize;
    for chunk in image.data.chunks(chunk_size) {
        iface.write_fpga_data(chunk).map_err(|e| {
            Error::Usb(format!(
                "fpga load failed after {sent} of {total} bytes ({}%): {e}",
                sent * 100 / total
            ))
        })?;
        sent += chunk.len();
        progress(sent, total);

        // UHD logs on every 10% boundary; the same granularity, because a load
        // that stalls part way is worth being able to place.
        let decile = sent * 10 / total;
        if decile != last_decile {
            last_decile = decile;
            trace!(percent = decile * 10, "fpga load");
        }
    }

    wait_for_state(
        iface,
        Fx3State::Running,
        FX3_CONFIG_POLLS,
        "finish configuring",
    )?;
    iface.set_fpga_hash(image.hash)?;

    let state = iface.fx3_state()?;
    let took = started.elapsed();
    info!(
        bytes = total,
        took_ms = took.as_millis(),
        state = %state,
        "fpga image loaded"
    );

    Ok(FpgaLoad {
        loaded: true,
        hash: image.hash,
        previous_hash,
        bytes: total,
        chunk_size,
        state,
        took,
    })
}

/// [`load_fpga`], then the GPIF reset that has to follow it.
///
/// `b200_impl`'s constructor calls `_iface->reset_gpif()` on the line after
/// `load_fpga`, unconditionally — including when the load was skipped. That is
/// mirrored here, and it is not redundant: the GPIF is the parallel bus between
/// the FX3 and the FPGA, its state machine was running against whatever was in
/// the FPGA before, and the bulk endpoints are not usable until both ends agree
/// again. Skipping it leaves a device that answers control requests perfectly
/// and returns nothing on its data endpoints.
///
/// # Errors
///
/// As [`load_fpga`], plus [`Error::Usb`] if the GPIF reset request fails.
pub fn configure(
    iface: &Fx3Iface,
    image: &FpgaImage,
    force: bool,
    progress: impl FnMut(usize, usize),
) -> Result<FpgaLoad> {
    let outcome = load_fpga(iface, image, force, progress)?;
    debug!("resetting gpif after fpga load");
    iface.reset_gpif()?;
    Ok(outcome)
}

/// Which FPGA image a probed device needs.
///
/// USB answers this for every B2xx except the B200 and B210, which share
/// product ID `0x0020`; for those the EEPROM product code decides, so this
/// needs an open control endpoint. `b200_impl` resolves the same two sources in
/// the same order in `get_b200_product`.
///
/// # Errors
///
/// [`Error::Usb`] if the EEPROM read fails. [`Error::Firmware`] if neither
/// source identifies the product, which is a board whose EEPROM holds a code
/// this driver does not know — the caller should name an image explicitly.
pub fn image_for(probe: &UsbProbe, iface: &Fx3Iface) -> Result<&'static str> {
    if let Some(product) = probe.product {
        return Ok(product.fpga_image());
    }

    if probe.state != DeviceState::Ready {
        return Err(Error::Firmware(format!(
            "cannot tell which fpga image {} needs: usb reports the product id the \
             b200 and b210 share, and the eeprom cannot be read until firmware is \
             running",
            probe.endpoint
        )));
    }

    let eeprom = super::eeprom::read(iface)?;
    eeprom
        .product()
        .map(super::B2xxProduct::fpga_image)
        .ok_or_else(|| {
            Error::Firmware(format!(
                "cannot tell which fpga image {} needs: usb reports the product id the \
                 b200 and b210 share, and the eeprom product code is {}",
                probe.endpoint,
                match eeprom.product_code {
                    Some(code) => format!("{code:#06x}, which this driver does not know"),
                    None => "not set".to_owned(),
                }
            ))
        })
}

/// Polls the FX3 until it reaches `target`, or until it says something has
/// gone wrong.
///
/// `load_fpga` has three of these loops with the same shape and different
/// bounds. `Error` and `Undefined` abort immediately rather than waiting out
/// the timeout: both mean the FX3 has given up on the configuration, and the
/// only thing another five seconds of polling adds is five seconds.
fn wait_for_state(iface: &Fx3Iface, target: Fx3State, max_polls: u32, what: &str) -> Result<()> {
    // `0..=max_polls` rather than `0..max_polls`: UHD's do-while reads the
    // status once before it first tests `wait_count`, so the bound is a bound
    // on sleeps, not on reads.
    for _ in 0..=max_polls {
        let state = iface.fx3_state()?;
        if state == target {
            return Ok(());
        }
        if matches!(state, Fx3State::Error | Fx3State::Undefined) {
            return Err(Error::Firmware(format!(
                "the fx3 reported `{state}` while waiting for it to {what}; the fpga is \
                 left unconfigured"
            )));
        }
        std::thread::sleep(FX3_POLL_INTERVAL);
    }

    Err(Error::Firmware(format!(
        "the fx3 did not {what} within {} ms; it is still in `{}`",
        (u64::from(max_polls) + 1) * FX3_POLL_INTERVAL.as_millis() as u64,
        iface.fx3_state()?
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_hash_matches_the_reference_algorithm() {
        // Worked through by hand from `generate_hash` in b200_iface.cpp:
        //   hash = 0 ^ (0x00 + 0x9e3779b9 + 0 + 0) = 0x9e3779b9
        assert_eq!(image_hash(&[0x00]), 0x9e37_79b9);

        // Second byte, with the shifts of a non-zero accumulator in play:
        //   h = 0x9e3779b9
        //   0x00 + 0x9e3779b9 + (h << 6) + (h >> 2) = wrapping
        let h: u32 = 0x9e37_79b9;
        let expected = h ^ 0x9e37_79b9u32.wrapping_add(h << 6).wrapping_add(h >> 2);
        assert_eq!(image_hash(&[0x00, 0x00]), expected);
    }

    #[test]
    fn high_bytes_are_sign_extended_the_way_c_char_is() {
        // The whole point of the `as i8` in `image_hash`. A byte of 0xff is
        // C's `char` -1, so the addend is 0x9e3779b8, not 0x9e3780b8.
        assert_eq!(image_hash(&[0xff]), 0x9e37_79b9u32.wrapping_sub(1));
        assert_ne!(image_hash(&[0xff]), 0x9e37_79b9u32.wrapping_add(0xff));
    }

    #[test]
    fn the_hash_is_order_sensitive_and_length_sensitive() {
        // Not a strong property, but it is the property the skip decision
        // rests on: two different images must not collide trivially.
        assert_ne!(image_hash(b"ab"), image_hash(b"ba"));
        assert_ne!(image_hash(b"a"), image_hash(b"aa"));
        assert_eq!(image_hash(b""), 0);
    }

    #[test]
    fn the_poll_bounds_match_the_reference() {
        // wait_count limits in load_fpga, UHD v4.10.0.0.
        assert_eq!(FX3_CONFIG_POLLS, 500);
        assert_eq!(FX3_BITSTREAM_POLLS, 1000);
        assert_eq!(FX3_POLL_INTERVAL, Duration::from_millis(10));
    }
}
