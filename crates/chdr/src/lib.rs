//! CHDR — the Compressed Header packet format USRPs use for sample and control
//! traffic.
//!
//! Empty for now. When it is filled in, it will hold packet header parsing and
//! generation only: no I/O, no device knowledge, so that it can be tested
//! against captured packets and reused by every backend.
//!
//! Reference: `reference/uhd/host/lib/rfnoc/chdr_packet_writer.cpp` and
//! `host/include/uhd/rfnoc/chdr_types.hpp`. Note that the B200 predates RFNoC
//! and speaks the older VRT/CVITA framing, described in
//! `host/lib/usrp/b200/b200_io_impl.cpp` — see `NOTES.md`.
