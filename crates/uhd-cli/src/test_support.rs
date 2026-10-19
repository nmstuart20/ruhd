//! Scripted receive stream shared by the CLI's streaming regression tests.

use std::collections::VecDeque;
use std::time::Duration;
use uhd_core::{Error, RawRxStreamer, Result, RxMetadata, SampleFormat, StreamCmd};

#[derive(Debug)]
pub(crate) struct ScriptedRx {
    responses: VecDeque<Result<(Vec<u8>, RxMetadata)>>,
    pub(crate) commands: Vec<StreamCmd>,
    pub(crate) stop_error: Option<Error>,
}

impl ScriptedRx {
    pub(crate) fn new(responses: impl IntoIterator<Item = Result<(Vec<u8>, RxMetadata)>>) -> Self {
        Self {
            responses: responses.into_iter().collect(),
            commands: Vec::new(),
            stop_error: None,
        }
    }
}

impl RawRxStreamer for ScriptedRx {
    fn num_channels(&self) -> usize {
        1
    }

    fn max_num_samps(&self) -> usize {
        1 << 19
    }

    fn host_format(&self) -> SampleFormat {
        SampleFormat::Sc16
    }

    fn wire_format(&self) -> SampleFormat {
        SampleFormat::Sc16
    }

    fn issue_stream_cmd(&mut self, cmd: &StreamCmd) -> Result<()> {
        self.commands.push(*cmd);
        if *cmd == StreamCmd::stop()
            && let Some(error) = self.stop_error.take()
        {
            return Err(error);
        }
        Ok(())
    }

    fn recv_raw(
        &mut self,
        buffs: &mut [&mut [u8]],
        _timeout: Duration,
        md: &mut RxMetadata,
    ) -> Result<usize> {
        let (bytes, metadata) = self.responses.pop_front().expect("unexpected receive")?;
        assert_eq!(buffs.len(), 1);
        assert_eq!(bytes.len() % 4, 0);
        buffs[0][..bytes.len()].copy_from_slice(&bytes);
        *md = metadata;
        Ok(bytes.len() / 4)
    }
}
