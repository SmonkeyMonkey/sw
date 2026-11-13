//! The operator's ground control station: its links (radio, base, QGroundControl), the
//! QGroundControl bridge, and noticing lost drones, usable without the UI (see
//! examples/headless.rs).

pub mod fleet;
pub mod link;
pub mod qgc;
