//! The interface between a drone's brain and its own body: what its onboard sensors report and
//! what it commands its motors to do. This is local to one drone — inter-drone communication
//! never goes through here, only through MAVLink over the (emulated) network.

use swarm_brain::{Mode, Nav, Vec2};

/// Faults the simulator can inject into a live drone (killing one is done by the world itself,
/// by cutting off its sensor feed).
#[derive(Clone, Copy, Debug)]
pub enum Fault {
    /// Compromise the drone's radio: from now on it keeps broadcasting a fake sighting of a
    /// target at `fake_pos`, claiming to have seen it firsthand.
    Lie { fake_pos: Vec2 },
}

/// What a drone's brain tells its body after each tick.
#[derive(Clone, Copy, Debug)]
pub struct AgentReport {
    pub index: usize,
    pub time_ms: u32,
    pub velocity: Vec2,
    /// Forwarded to the viewer and telemetry only; no other drone ever sees these fields.
    pub informed: bool,
    /// The targets it knows of: where it thinks each is, and whether it counts it confirmed.
    pub known: [Option<(Vec2, bool)>; swarm_brain::MAX_TRACKS],
    /// Relaying, on the sweep, or neither.
    pub role: swarm_brain::Role,
    pub isolated: bool,
    pub mode: Mode,
    /// Swarm-wide ID of the target it's responding to, if any.
    pub target: Option<u16>,
    /// Back at base after a return-to-launch: land the airframe.
    pub land: bool,
    /// ...to charge (see `swarm_brain::TickOutput::to_charge`).
    pub to_charge: bool,
    /// Where the drone's brain thinks it is, and where that came from.
    pub position: Vec2,
    pub nav: Nav,
}
