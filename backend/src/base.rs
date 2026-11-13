//! The launch base's link to the operator: the ground station sits at the base and is wired to
//! the launcher, so this is plain UDP rather than the emulated radio. It speaks MAVLink like
//! everything else: `COMMAND_LONG(MAV_CMD_SWARM_LAUNCH, param1 = count)` addressed to
//! `BASE_SYSID`, answered with a `COMMAND_ACK`. Also `MAV_CMD_SWARM_SET_FIELD` (param1 width,
//! param2 height, in meters, param3 how many drones to launch, NaN for the default): a new
//! mission on that field. And `MAV_CMD_SWARM_REVOKE` (param1 the drone): the base revokes its
//! key for the rest of the mission, on the radio (see `beacon`). `MAV_CMD_SWARM_SET_SPARES`
//! (param1 how many) and `MAV_CMD_SWARM_SET_ROTATION` (param1 1 or 0) set up its pads (see
//! `world::Dock`), reported back in SWARM_BASE_STATUS with every SWARM_OPERATOR_KEY.
//!
//! The base is also the fleet's authority (see `swarm_brain::auth`): to the ground station's
//! heartbeat it answers with the mission's operator key and clock (SWARM_OPERATOR_KEY), over
//! this wire only. That's what the ground station signs its orders with.

use std::sync::{Arc, RwLock};
use std::time::Instant;

use mavlink_swarm::mavlink_core::{write_v2_msg, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{MavCmd, MavMessage, MavResult, COMMAND_ACK_DATA, SWARM_BASE_STATUS_DATA, SWARM_OPERATOR_KEY_DATA};
use swarm_brain::auth::Authority;
use swarm_brain::radio;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;

use crate::world::Dock;
use crate::ws::Command;

/// The mission under way, as the base knows it.
pub struct Mission {
    pub number: u32,
    /// The mission clock starts here (see `world::WorldLinks::started`).
    pub started: Instant,
    /// Drones the operator had the base revoke.
    pub revoked: Vec<u8>,
}

impl Default for Mission {
    fn default() -> Self {
        Self { number: 0, started: Instant::now(), revoked: Vec::new() }
    }
}

/// Most drones one launch request can ask for.
const MAX_LAUNCH: u32 = 160;

/// The launcher's MAVLink system ID.
pub use swarm_brain::BASE_SYSID;

/// The base's radio: a heartbeat a second on the drones' radio channel, from the base's corner
/// (netem places `BASE_SYSID` there), so the drones hear the base whether or not the operator's
/// ground station is running. Everything it hears back is dropped.
/// Revocations go out with it, every second for the rest of the mission: drones launched since,
/// or out of reach when it was relayed, hear it too.
pub async fn beacon(socket: UdpSocket, netem: std::net::SocketAddr, authority: Arc<Authority>, mission: Arc<RwLock<Mission>>) {
    let mut sequence: u8 = 0;
    let mut signer: Option<([u8; 32], radio::Signer)> = None;
    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
    let mut buf = [0u8; 512];
    loop {
        tokio::select! {
            _ = interval.tick() => {
                let hb = MavMessage::HEARTBEAT(mavlink_swarm::HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: mavlink_swarm::MavType::MAV_TYPE_GCS,
                    autopilot: mavlink_swarm::MavAutopilot::MAV_AUTOPILOT_INVALID,
                    base_mode: mavlink_swarm::MavModeFlag::empty(),
                    system_status: mavlink_swarm::MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                });
                let revocations: Vec<MavMessage> = {
                    let m = mission.read().unwrap();
                    m.revoked.iter().map(|&sysid| MavMessage::SWARM_REVOKE(authority.revoke(m.number, sysid))).collect()
                };
                // Signed with the mission's link key, like every frame on the air (see
                // `swarm_brain::radio`), as a radio of the swarm's.
                let (link_key, now_ms) = {
                    let m = mission.read().unwrap();
                    (authority.link_key(m.number), m.started.elapsed().as_millis() as u32)
                };
                // A new mission's key: a new signer.
                if signer.as_ref().is_none_or(|(key, _)| *key != link_key) {
                    signer = Some((link_key, radio::Signer::new(link_key, BASE_SYSID)));
                }
                let (_, signer) = signer.as_mut().expect("just set");
                for msg in std::iter::once(hb).chain(revocations) {
                    let header = MavHeader { system_id: BASE_SYSID, component_id: 1, sequence };
                    sequence = sequence.wrapping_add(1);
                    let _ = socket.send_to(signer.frame(header, &msg, now_ms).raw_bytes(), netem).await;
                }
            }
            _ = socket.recv_from(&mut buf) => {}
        }
    }
}

/// Most spares the operator can have the base keep on its pads.
const MAX_SPARES: f32 = 64.0;

pub async fn run(socket: UdpSocket, commands: mpsc::Sender<Command>, authority: Arc<Authority>, mission: Arc<RwLock<Mission>>, dock: Arc<RwLock<Dock>>) {
    let mut buf = [0u8; 512];
    let mut sequence: u8 = 0;
    // The last field asked for, and when: the ground station repeats a command until it hears
    // the acknowledgement, and every new field restarts the mission.
    let mut last_field: Option<((f32, f32), std::time::Instant)> = None;
    loop {
        let Ok((len, from)) = socket.recv_from(&mut buf).await else { continue };
        let mut reader = MavlinkReader::new(&buf[..len]);
        while let Ok((_, msg)) = reader.read_message::<MavMessage>(MavlinkVersion::V2) {
            let cmd = match msg {
                MavMessage::COMMAND_LONG(cmd) if cmd.target_system == BASE_SYSID => cmd,
                MavMessage::HEARTBEAT(_) => {
                    let key = {
                        let m = mission.read().unwrap();
                        SWARM_OPERATOR_KEY_DATA {
                            mission: m.number,
                            mission_ms: m.started.elapsed().as_millis() as u32,
                            secret_key: authority.operator_secret(m.number),
                            link_key: authority.link_key(m.number),
                        }
                    };
                    let header = MavHeader { system_id: BASE_SYSID, component_id: 1, sequence };
                    sequence = sequence.wrapping_add(1);
                    let mut out = Vec::new();
                    if write_v2_msg(&mut out, header, &MavMessage::SWARM_OPERATOR_KEY(key)).is_ok() {
                        let _ = socket.send_to(&out, from).await;
                    }
                    let d = *dock.read().unwrap();
                    let status = SWARM_BASE_STATUS_DATA {
                        parked: d.parked.min(255) as u8,
                        ready: d.ready.min(255) as u8,
                        waiting: d.waiting.min(255) as u8,
                        spares: d.spares.min(255) as u8,
                        rotation: d.rotation as u8,
                        batteries: d.batteries as u8,
                        lowest_charge: d.lowest_charge.map_or(255, |c| (100.0 * c).round() as u8),
                    };
                    let header = MavHeader { system_id: BASE_SYSID, component_id: 1, sequence };
                    sequence = sequence.wrapping_add(1);
                    let mut out = Vec::new();
                    if write_v2_msg(&mut out, header, &MavMessage::SWARM_BASE_STATUS(status)).is_ok() {
                        let _ = socket.send_to(&out, from).await;
                    }
                    continue;
                }
                _ => continue,
            };
            let result = match cmd.command {
                MavCmd::MAV_CMD_SWARM_LAUNCH if cmd.param1 >= 1.0 && cmd.param1.is_finite() => {
                    // Bounded again by the world, at what the field can take.
                    let count = (cmd.param1 as u32).min(MAX_LAUNCH);
                    if commands.send(Command::Launch(count)).await.is_err() {
                        return;
                    }
                    println!("base: launch {count} requested by the ground station");
                    MavResult::MAV_RESULT_ACCEPTED
                }
                MavCmd::MAV_CMD_SWARM_LAUNCH => MavResult::MAV_RESULT_DENIED,
                MavCmd::MAV_CMD_SWARM_SET_FIELD if cmd.param1 >= 1.0 && cmd.param2 >= 1.0 => {
                    let size = (cmd.param1, cmd.param2);
                    let repeat = last_field.is_some_and(|(last, at)| last == size && at.elapsed() < std::time::Duration::from_secs(5));
                    if !repeat {
                        last_field = Some((size, std::time::Instant::now()));
                        let launch = (cmd.param3.is_finite() && cmd.param3 >= 0.0).then_some(cmd.param3 as u32);
                        if commands.send(Command::SetField(swarm_brain::Vec2::new(size.0, size.1), launch)).await.is_err() {
                            return;
                        }
                        println!("base: field {:.0} x {:.0} m requested by the ground station", size.0, size.1);
                    }
                    MavResult::MAV_RESULT_ACCEPTED
                }
                MavCmd::MAV_CMD_SWARM_SET_FIELD => MavResult::MAV_RESULT_DENIED,
                MavCmd::MAV_CMD_SWARM_REVOKE if (1.0..=254.0).contains(&cmd.param1) => {
                    let sysid = cmd.param1 as u8;
                    let mut m = mission.write().unwrap();
                    if !m.revoked.contains(&sysid) {
                        m.revoked.push(sysid);
                        println!("base: drone {sysid} revoked by the ground station");
                    }
                    MavResult::MAV_RESULT_ACCEPTED
                }
                MavCmd::MAV_CMD_SWARM_REVOKE => MavResult::MAV_RESULT_DENIED,
                // Both say how things should be, not what to do: the ground station repeats a
                // command until it hears the acknowledgement, and doing one twice changes nothing.
                MavCmd::MAV_CMD_SWARM_SET_SPARES if (0.0..=MAX_SPARES).contains(&cmd.param1) => {
                    let n = cmd.param1 as u32;
                    if dock.read().unwrap().spares != n {
                        if commands.send(Command::SetSpares(n)).await.is_err() {
                            return;
                        }
                        println!("base: {n} spares on the pads, asked for by the ground station");
                    }
                    MavResult::MAV_RESULT_ACCEPTED
                }
                MavCmd::MAV_CMD_SWARM_SET_SPARES => MavResult::MAV_RESULT_DENIED,
                MavCmd::MAV_CMD_SWARM_SET_ROTATION if cmd.param1 == 0.0 || cmd.param1 == 1.0 => {
                    let on = cmd.param1 == 1.0;
                    if dock.read().unwrap().rotation != on && commands.send(Command::SetRotation(on)).await.is_err() {
                        return;
                    }
                    MavResult::MAV_RESULT_ACCEPTED
                }
                MavCmd::MAV_CMD_SWARM_SET_ROTATION => MavResult::MAV_RESULT_DENIED,
                _ => MavResult::MAV_RESULT_UNSUPPORTED,
            };
            let ack = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: cmd.command, result });
            let header = MavHeader { system_id: BASE_SYSID, component_id: 1, sequence };
            sequence = sequence.wrapping_add(1);
            let mut out = Vec::new();
            if write_v2_msg(&mut out, header, &ack).is_ok() {
                let _ = socket.send_to(&out, from).await;
            }
        }
    }
}
