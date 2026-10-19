//! Flags for the B2xx receive driver thread, shared by `rx` and `benchmark`.

use clap::Args;
use serde_json::json;
use uhd_b200::rx_stream::DEFAULT_RECV_BUFF_SIZE;
use uhd_core::StreamArgs;

#[derive(Clone, Debug, Default, Args)]
pub(crate) struct DriverArgs {
    /// Bytes of host buffering behind the USB transfer queue, e.g. 64e6.
    /// Filled transfers wait here if receiving falls behind, instead of the
    /// device overflowing. Defaults to 32 MiB.
    #[arg(long)]
    recv_buff_size: Option<f64>,
    /// Run the USB driver thread under SCHED_FIFO at this priority, 1 to 99.
    /// Needs CAP_SYS_NICE or `ulimit -r`; only warns if refused.
    #[arg(long, value_parser = clap::value_parser!(i32).range(1..=99))]
    driver_priority: Option<i32>,
    /// Pin the USB driver thread to this CPU.
    #[arg(long)]
    driver_cpu: Option<usize>,
    /// Lock the process's memory (mlockall) once the buffers exist. Needs
    /// CAP_IPC_LOCK or a large enough `ulimit -l`; only warns if refused.
    #[arg(long)]
    driver_mlock: bool,
}

impl DriverArgs {
    pub(crate) fn apply(&self, mut args: StreamArgs) -> StreamArgs {
        if let Some(bytes) = self.recv_buff_size {
            args = args.with_arg("recv_buff_size", bytes.to_string());
        }
        if let Some(priority) = self.driver_priority {
            args = args.with_arg("driver_rt_priority", priority.to_string());
        }
        if let Some(cpu) = self.driver_cpu {
            args = args.with_arg("driver_cpu", cpu.to_string());
        }
        if self.driver_mlock {
            args = args.with_arg("driver_mlock", "1");
        }
        args
    }

    /// The requested settings, for a benchmark report.
    pub(crate) fn report(&self) -> serde_json::Value {
        json!({
            "recv_buff_size_bytes": self
                .recv_buff_size
                .map_or(DEFAULT_RECV_BUFF_SIZE as f64, |b| b.max(0.0).trunc()),
            "rt_priority": self.driver_priority,
            "cpu": self.driver_cpu,
            "mlock": self.driver_mlock,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_given_flags_become_stream_args() {
        let args = DriverArgs::default().apply(StreamArgs::new());
        assert!(args.args.is_empty());

        let args = DriverArgs {
            recv_buff_size: Some(64e6),
            driver_priority: Some(80),
            driver_cpu: Some(2),
            driver_mlock: true,
        }
        .apply(StreamArgs::new());
        assert_eq!(args.args.get("recv_buff_size"), Some("64000000"));
        assert_eq!(args.args.get("driver_rt_priority"), Some("80"));
        assert_eq!(args.args.get("driver_cpu"), Some("2"));
        assert_eq!(args.args.get("driver_mlock"), Some("1"));
    }
}
