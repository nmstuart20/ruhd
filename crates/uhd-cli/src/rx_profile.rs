//! Coordinated receive settings and sample-time budgets.

use clap::ValueEnum;
use uhd_core::{Error, Result, SampleFormat, StreamArgs};

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub(crate) enum RxProfile {
    /// Small packets and blocks for interactive stdout processing.
    LowLatency,
    /// Moderate batching and headroom for general processing.
    Balanced,
    /// Large blocks and a 64 MiB pool for sustained recording.
    Capture,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct RxConfig {
    pub(crate) profile: RxProfile,
    pub(crate) spp: usize,
    pub(crate) transfers: usize,
    pub(crate) block_bytes: usize,
    pub(crate) blocks: usize,
}

impl RxProfile {
    pub(crate) fn config(self) -> RxConfig {
        let (spp, transfers, block_bytes, blocks) = match self {
            Self::LowLatency => (256, 16, 16 << 10, 4),
            Self::Balanced => (1024, 64, 256 << 10, 8),
            Self::Capture => (2040, 128, 2 << 20, 32),
        };
        RxConfig {
            profile: self,
            spp,
            transfers,
            block_bytes,
            blocks,
        }
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::LowLatency => "low-latency",
            Self::Balanced => "balanced",
            Self::Capture => "capture",
        }
    }
}

impl RxConfig {
    pub(crate) fn with_overrides(
        mut self,
        spp: Option<usize>,
        transfers: Option<usize>,
    ) -> Result<Self> {
        if let Some(spp) = spp {
            if spp == 0 {
                return Err(Error::invalid_arg("--spp must be at least 1"));
            }
            self.spp = spp;
        }
        if let Some(transfers) = transfers {
            if transfers < 2 {
                return Err(Error::invalid_arg("--num-recv-frames must be at least 2"));
            }
            self.transfers = transfers;
        }
        Ok(self)
    }

    pub(crate) fn stream_args(self, format: SampleFormat) -> StreamArgs {
        let mut args = StreamArgs::new()
            .with_arg("spp", self.spp.to_string())
            .with_arg("num_recv_frames", self.transfers.to_string());
        args.host_format = format;
        args
    }

    pub(crate) fn block_samples(self, format: SampleFormat) -> usize {
        self.block_bytes / (format.bits_per_sample() / 8)
    }

    /// Use the streamer's coerced packet size and the actual sample rate.
    pub(crate) fn report(self, format: SampleFormat, rate: f64, actual_spp: usize) {
        let block_samples = self.block_samples(format);
        eprintln!(
            "  profile:      {} (effective settings below)",
            self.profile.name()
        );
        eprintln!(
            "  packet:       {actual_spp} samples, {:.3} ms fill; {} USB transfers, {:.3} ms sample capacity",
            actual_spp as f64 / rate * 1000.0,
            self.transfers,
            actual_spp as f64 * self.transfers as f64 / rate * 1000.0,
        );
        eprintln!(
            "  application:  {} bytes/block ({block_samples} samples), {:.3} ms fill; {} blocks, {:.3} ms sample capacity ({} bytes)",
            self.block_bytes,
            block_samples as f64 / rate * 1000.0,
            self.blocks,
            block_samples as f64 * self.blocks as f64 / rate * 1000.0,
            self.block_bytes * self.blocks,
        );
        eprintln!(
            "  budgets describe sample batching/capacity; scheduling and output stalls add latency"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles_meet_the_documented_batching_and_memory_budgets() {
        for (profile, block_ms, pool_bytes) in [
            (RxProfile::LowLatency, 4.096, 64 << 10),
            (RxProfile::Balanced, 65.536, 2 << 20),
            (RxProfile::Capture, 524.288, 64 << 20),
        ] {
            let config = profile.config();
            assert_eq!(config.block_bytes * config.blocks, pool_bytes);
            for (format, bytes) in [
                (SampleFormat::Sc16, 4),
                (SampleFormat::Fc32, 8),
                (SampleFormat::Fc64, 16),
            ] {
                let samples = config.block_samples(format);
                assert_eq!(samples * bytes, config.block_bytes);
                assert!(
                    (samples as f64 / 1e6 * 1000.0 - block_ms * 4.0 / bytes as f64).abs() < 1e-9
                );
            }
            // Each CHDR packet must end with a short USB packet so nusb
            // completes the transfer without waiting for another CHDR packet.
            assert_ne!((config.spp * 4 + 16) % 512, 0);
        }
    }

    #[test]
    fn stream_args_apply_the_profile_and_overrides() {
        let config = RxProfile::LowLatency.config();
        let args = config.stream_args(SampleFormat::Fc32);
        assert_eq!(args.host_format, SampleFormat::Fc32);
        assert_eq!(args.wire_format, SampleFormat::Sc16);
        assert_eq!(args.args.get_parsed::<usize>("spp").unwrap(), Some(256));
        assert_eq!(
            args.args.get_parsed::<usize>("num_recv_frames").unwrap(),
            Some(16)
        );
        let args = config
            .with_overrides(Some(100), Some(8))
            .unwrap()
            .stream_args(SampleFormat::Sc16);
        assert_eq!(args.args.get_parsed::<usize>("spp").unwrap(), Some(100));
        assert_eq!(
            args.args.get_parsed::<usize>("num_recv_frames").unwrap(),
            Some(8)
        );
    }
}
