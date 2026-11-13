//! MAVLink bindings for the swarm dialect: everything in MAVLink `common` plus the swarm-specific
//! messages defined in dialect/swarm.xml.

#![cfg_attr(not(feature = "std"), no_std)]
// The generated code carries `cfg_attr(feature = "serde" | "arbitrary" | "ts-rs", ...)`
// attributes meant for the upstream `mavlink` crate's features, which this crate doesn't have.
#![allow(unexpected_cfgs)]

mod dialects {
    include!(concat!(env!("OUT_DIR"), "/mod.rs"));
}

pub use dialects::swarm::*;
pub use mavlink_core;
