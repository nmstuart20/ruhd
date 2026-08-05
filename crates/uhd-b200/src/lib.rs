//! Backend for the USRP B200/B210/B205mini family. Initial target: B205mini.
//!
//! Reference: `reference/uhd/host/lib/usrp/b200/{b200_iface,b200_impl,b200_io_impl}.cpp`
//! — see `NOTES.md`.

pub mod radio_ctrl;

pub use radio_ctrl::{ControlGate, FpgaVersion, RadioCtrl};
