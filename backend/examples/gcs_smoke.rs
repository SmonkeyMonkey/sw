//! An end-to-end check of the operator's path, without the GUI: plays the ground station
//! (system ID 255, at the base, through the emulated radio) and drives a whole operator session.
//!
//!   cargo run -- --drones 0                 # backend: nothing launched until asked
//!   cargo run --example gcs_smoke           # this
//!
//! Launches drones from the base, waits for their statuses (relayed by the swarm), orders the
//! farthest one somewhere and then home, and cancels a target, checking each step's
//! acknowledgement.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use mavlink_swarm::mavlink_core::{write_v2_msg, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{
    MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult, MavState, MavType, COMMAND_LONG_DATA, HEARTBEAT_DATA,
};
use tokio::net::UdpSocket;

#[path = "common/signer.rs"]
mod signer;
use signer::{Signer, BASE};

const RADIO: &str = "127.0.0.1:14600";
const BASE_SYSID: u8 = 250;
/// Where a drone is ordered to: within the ground station's own radio reach of the base, so
/// the test doesn't depend on the swarm happening to keep a relay chain back to the base.
const GOAL: (f32, f32) = (-8.0, -7.0);
const VIEWER: &str = "ws://127.0.0.1:9002";

/// What the ground station last heard from a drone: x, y, mode, order, ack_command,
/// ack_result, and when.
type DroneSeen = (f32, f32, u8, u8, u16, u8, Instant);

struct Gcs {
    socket: UdpSocket,
    sequence: u8,
    started: Instant,
    /// sysid -> what was last heard from it
    drones: BTreeMap<u8, DroneSeen>,
    /// target id -> (x, y, cancelled)
    targets: BTreeMap<u16, (f32, f32, bool)>,
    base_ack: Option<(MavCmd, MavResult)>,
    /// What the base last said is on its pads.
    base_status: Option<mavlink_swarm::SWARM_BASE_STATUS_DATA>,
    next_heartbeat: Instant,
    signer: Signer,
}

impl Gcs {
    /// Orders over the radio go signed (see `Signer`).
    async fn send(&mut self, to: &str, msg: &MavMessage) {
        let header = MavHeader { system_id: 255, component_id: 190, sequence: self.sequence };
        self.sequence = self.sequence.wrapping_add(1);
        let bytes = if to == RADIO {
            self.signer.on_air(header, msg)
        } else {
            let mut bytes = Vec::new();
            write_v2_msg(&mut bytes, header, msg).unwrap();
            bytes
        };
        self.socket.send_to(&bytes, to).await.unwrap();
    }

    fn t(&self) -> f32 {
        self.started.elapsed().as_secs_f32()
    }

    /// Listens (and keeps sending heartbeats) for `for_` or until `done` holds.
    async fn listen_until(&mut self, for_: Duration, mut done: impl FnMut(&Gcs) -> bool) -> bool {
        let deadline = Instant::now() + for_;
        let mut buf = [0u8; 512];
        while Instant::now() < deadline {
            if Instant::now() >= self.next_heartbeat {
                self.next_heartbeat = Instant::now() + Duration::from_secs(1);
                let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: MavType::MAV_TYPE_GCS,
                    autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                    base_mode: MavModeFlag::empty(),
                    system_status: MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                });
                self.send(RADIO, &hb).await;
                // The base answers with the operator key.
                self.send(BASE, &hb).await;
            }
            if done(self) {
                return true;
            }
            let Ok(Ok((len, _))) = tokio::time::timeout(Duration::from_millis(50), self.socket.recv_from(&mut buf)).await else {
                continue;
            };
            let mut reader = MavlinkReader::new(&buf[..len]);
            while let Ok((_, msg)) = reader.read_message::<MavMessage>(MavlinkVersion::V2) {
                self.signer.heard(&msg);
                match msg {
                    MavMessage::SWARM_DRONE_STATUS(st) => {
                        self.drones.insert(st.sysid, (st.x, st.y, st.mode, st.order, st.ack_command, st.ack_result, Instant::now()));
                    }
                    MavMessage::SWARM_TARGET_SIGHTING(s) => {
                        let was = self.targets.get(&s.target_id).is_some_and(|t| t.2);
                        self.targets.insert(s.target_id, (s.x, s.y, was || s.cancelled == 1));
                    }
                    MavMessage::COMMAND_ACK(a) => self.base_ack = Some((a.command, a.result)),
                    MavMessage::SWARM_BASE_STATUS(st) => self.base_status = Some(st),
                    _ => {}
                }
            }
        }
        done(self)
    }
}

/// `attempt` goes into `confirmation`, as MAVLink asks for repeats: it also makes each repeat a
/// new frame for the drones relaying it, which otherwise drop a frame they've just relayed.
fn command(target_system: u8, command: MavCmd, params: [f32; 7], attempt: u8) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
        command,
        target_system,
        target_component: 0,
        confirmation: attempt,
    })
}

fn check(ok: bool, what: &str) {
    println!("{} {what}", if ok { "PASS" } else { "FAIL" });
    if !ok {
        std::process::exit(1);
    }
}

#[tokio::main]
async fn main() {
    let mut gcs = Gcs {
        socket: UdpSocket::bind("127.0.0.1:0").await.unwrap(),
        sequence: 0,
        started: Instant::now(),
        drones: BTreeMap::new(),
        targets: BTreeMap::new(),
        base_ack: None,
        base_status: None,
        next_heartbeat: Instant::now(),
        signer: Signer::default(),
    };
    let keyed = gcs.listen_until(Duration::from_secs(3), |g| g.signer.ready()).await;
    check(keyed, "the base handed over the operator key");
    let status = gcs.listen_until(Duration::from_secs(3), |g| g.base_status.is_some()).await;
    check(status, "the base reports its pads");

    // 0. Scene setup through the simulator (not something an operator can do): a mouse near the
    //    base, so there's a target to cancel within radio reach.
    let (mut ws, _) = tokio_tungstenite::connect_async(VIEWER).await.expect("backend running?");
    use futures_util::SinkExt;
    ws.send(tokio_tungstenite::tungstenite::Message::Text(r#"{"add_target":{"x":-7.0,"y":-11.0}}"#.into())).await.unwrap();
    drop(ws);

    // 1. Launch from the base, over the wired base link.
    gcs.send(BASE, &command(BASE_SYSID, MavCmd::MAV_CMD_SWARM_LAUNCH, [6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0)).await;
    let acked = gcs.listen_until(Duration::from_secs(2), |g| g.base_ack.is_some()).await;
    check(acked && gcs.base_ack.unwrap().1 == MavResult::MAV_RESULT_ACCEPTED, "base accepted launch 6");

    // 2. All six report in — most of them only through other drones' relays once they spread out.
    let all = gcs.listen_until(Duration::from_secs(40), |g| g.drones.len() >= 6).await;
    check(all, &format!("statuses from all 6 drones by t={:.0}s", gcs.t()));
    // The one heard from most recently, farthest from the base.
    let (id, distance) = gcs
        .drones
        .iter()
        .filter(|(_, d)| d.6.elapsed() < Duration::from_secs(2))
        .map(|(&id, d)| (id, (d.0 + 13.0).hypot(d.1 + 13.0)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("someone in contact");
    println!("     ordering drone {id}, {distance:.0} m from the base");

    // 3. Order it to a spot, repeating until its relayed status acknowledges. A spot still in
    //    radio reach of the swarm around the base: a drone out of everyone's range can't be
    //    heard by the ground station at all.
    let goto = MavCmd::MAV_CMD_DO_REPOSITION;
    let mut acked = false;
    for attempt in 0..6 {
        gcs.send(RADIO, &command(id, goto, [-1.0, 0.0, 0.0, f32::NAN, GOAL.0, GOAL.1, 5.0], attempt)).await;
        if gcs.listen_until(Duration::from_millis(1_500), |g| g.drones.get(&id).is_some_and(|d| d.4 == goto as u16 && d.3 == 1)).await {
            println!("     acknowledged after {} send(s)", attempt + 1);
            acked = true;
            break;
        }
    }
    if !acked {
        for (sysid, d) in &gcs.drones {
            println!("     drone {sysid}: at ({:.1},{:.1}) mode {} order {} ack {} last status {:.1}s ago", d.0, d.1, d.2, d.3, d.4, d.6.elapsed().as_secs_f32());
        }
    }
    check(acked, &format!("drone {id} acknowledged go-to {GOAL:?}"));
    let arrived = gcs.listen_until(Duration::from_secs(25), |g| g.drones.get(&id).is_some_and(|d| (d.0 - GOAL.0).hypot(d.1 - GOAL.1) < 1.5)).await;
    check(arrived, &format!("drone {id} reports being at {GOAL:?}"));

    // 4. Home, and it should land and drop out of the mission.
    let rtl = MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH;
    let mut acked = false;
    for attempt in 0..6 {
        gcs.send(RADIO, &command(id, rtl, [0.0; 7], attempt)).await;
        if gcs.listen_until(Duration::from_millis(1_500), |g| g.drones.get(&id).is_some_and(|d| d.4 == rtl as u16)).await {
            acked = true;
            break;
        }
    }
    check(acked, &format!("drone {id} acknowledged return to base"));
    // Back by the base: within 5 m it asks to land, and the base guides it onto a pad beside it.
    let home = gcs.listen_until(Duration::from_secs(30), |g| g.drones.get(&id).is_some_and(|d| (d.0 + 13.0).hypot(d.1 + 13.0) < 6.0)).await;
    check(home, &format!("drone {id} reports being back at the base"));
    // Once landed it goes silent: no fresh status for a few seconds.
    let landed = gcs.listen_until(Duration::from_secs(15), |g| g.drones.get(&id).is_some_and(|d| d.6.elapsed() > Duration::from_secs(4))).await;
    check(landed, &format!("drone {id} landed and left the mission"));
    // On a pad now, and not waiting to be replaced: it was sent home, not down to charge.
    let parked = gcs.listen_until(Duration::from_secs(3), |g| g.base_status.as_ref().is_some_and(|b| b.parked >= 1 && b.waiting == 0)).await;
    check(parked, &format!("drone {id} is on a pad, and nobody takes off in its place"));

    // 5. Cancel a target the swarm has found.
    let found = gcs.listen_until(Duration::from_secs(60), |g| g.targets.values().any(|t| !t.2)).await;
    check(found, "the swarm reported a target");
    let target = *gcs.targets.iter().find(|(_, t)| !t.2).unwrap().0;
    let mut cancelled = false;
    for attempt in 0..6 {
        gcs.send(RADIO, &command(0, MavCmd::MAV_CMD_SWARM_CANCEL_TARGET, [target as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], attempt)).await;
        if gcs.listen_until(Duration::from_millis(1_500), |g| g.targets.get(&target).is_some_and(|t| t.2)).await {
            cancelled = true;
            break;
        }
    }
    check(cancelled, &format!("the swarm reports target {target:04x} cancelled"));

    // 6. The base's pads: spares, and swapping charged drones in.
    for (cmd, param, what, done) in [
        (MavCmd::MAV_CMD_SWARM_SET_SPARES, 3.0, "3 spares on the pads", (|b: &mavlink_swarm::SWARM_BASE_STATUS_DATA| b.spares == 3 && b.parked == 3) as fn(&_) -> bool),
        (MavCmd::MAV_CMD_SWARM_SET_ROTATION, 0.0, "charged drones not swapped in", |b| b.rotation == 0),
        (MavCmd::MAV_CMD_SWARM_SET_ROTATION, 1.0, "charged drones swapped in again", |b| b.rotation == 1),
    ] {
        gcs.base_ack = None;
        gcs.send(BASE, &command(BASE_SYSID, cmd, [param, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0)).await;
        let acked = gcs.listen_until(Duration::from_secs(2), |g| g.base_ack == Some((cmd, MavResult::MAV_RESULT_ACCEPTED))).await;
        let reported = gcs.listen_until(Duration::from_secs(3), |g| g.base_status.as_ref().is_some_and(done)).await;
        check(acked && reported, &format!("the base: {what}"));
    }
    println!("all checks passed at t={:.0}s", gcs.t());
}
