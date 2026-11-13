//! End-to-end check of the remote controller's protocol, without the GUI: plays the ground
//! station at the base and does what the remote does.
//!
//!   cargo run -- --mode connected            # backend (autonomous, the drones are mostly out
//!                                            # of the ground station's reach)
//!   cargo run --example remote_smoke         # this
//!
//! Takes a drone by hand and flies it toward the base, lets go of the sticks (it should
//! report the lost stick signal and hover, then rejoin the swarm on its own), takes it again
//! and hands it back by hand, then sets two drones per target and checks the swarm thins out
//! around the targets.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use mavlink_swarm::mavlink_core::{write_v2_msg, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{
    MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavState, MavType, COMMAND_LONG_DATA, HEARTBEAT_DATA, MANUAL_CONTROL_DATA,
};
use tokio::net::UdpSocket;

#[path = "common/signer.rs"]
mod signer;
use signer::{Signer, BASE};

const RADIO: &str = "127.0.0.1:14600";

struct Gcs {
    socket: UdpSocket,
    sequence: u8,
    /// sysid -> (x, y, order, seen at)
    drones: BTreeMap<u8, (f32, f32, u8, Instant)>,
    next_heartbeat: Instant,
    /// Stick frame number (MANUAL_CONTROL.buttons).
    stick_seq: u16,
    signer: Signer,
}

impl Gcs {
    /// Over the radio, signed (see `Signer`).
    async fn send(&mut self, msg: &MavMessage) {
        let header = MavHeader { system_id: 255, component_id: 190, sequence: self.sequence };
        self.sequence = self.sequence.wrapping_add(1);
        let bytes = self.signer.on_air(header, msg);
        self.socket.send_to(&bytes, RADIO).await.unwrap();
    }

    async fn send_to(&mut self, to: &str, msg: &MavMessage) {
        let mut bytes = Vec::new();
        write_v2_msg(&mut bytes, MavHeader { system_id: 255, component_id: 190, sequence: self.sequence }, msg).unwrap();
        self.sequence = self.sequence.wrapping_add(1);
        self.socket.send_to(&bytes, to).await.unwrap();
    }

    /// Listens for `for_`, keeping up the heartbeat, and sending `sticks` to `target` at 10 Hz.
    async fn listen(&mut self, for_: Duration, sticks: Option<(u8, i16, i16)>) {
        let until = Instant::now() + for_;
        let mut next_stick = Instant::now();
        let mut buf = [0u8; 512];
        while Instant::now() < until {
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
                self.send(&hb).await;
                // The base answers with the operator key.
                self.send_to(BASE, &hb).await;
            }
            if let Some((target, x, y)) = sticks {
                if Instant::now() >= next_stick {
                    next_stick = Instant::now() + Duration::from_millis(100);
                    self.stick_seq = self.stick_seq.wrapping_add(1);
                    let buttons = self.stick_seq;
                    self.send(&MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA { x, y, z: 500, r: 0, buttons, target })).await;
                }
            }
            let Ok(Ok((len, _))) = tokio::time::timeout(Duration::from_millis(20), self.socket.recv_from(&mut buf)).await else { continue };
            let mut reader = MavlinkReader::new(&buf[..len]);
            while let Ok((_, msg)) = reader.read_message::<MavMessage>(MavlinkVersion::V2) {
                self.signer.heard(&msg);
                if let MavMessage::SWARM_DRONE_STATUS(st) = msg {
                    self.drones.insert(st.sysid, (st.x, st.y, st.order, Instant::now()));
                }
            }
        }
    }
}

fn command(target_system: u8, command: MavCmd, param1: f32, confirmation: u8) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command,
        target_system,
        target_component: 0,
        confirmation,
    })
}

fn check(ok: bool, what: &str) {
    println!("{} {what}", if ok { "PASS" } else { "FAIL" });
    if !ok {
        std::process::exit(1);
    }
}

/// Where drone `id` really is, from the simulator's ground truth (the viewer feed): its status
/// reports reach the ground station up to a second or so late.
async fn truth(id: u8) -> Option<(f32, f32)> {
    let (mut ws, _) = tokio_tungstenite::connect_async("ws://127.0.0.1:9002").await.ok()?;
    let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await else { return None };
    let s: serde_json::Value = serde_json::from_str(&text).ok()?;
    let a = s["agents"].as_array()?.iter().find(|a| a["id"].as_u64() == Some(id as u64))?;
    Some((a["pos"]["x"].as_f64()? as f32, a["pos"]["y"].as_f64()? as f32))
}

/// How many drones are on each target, by the swarm's own target IDs, from the simulator's
/// ground truth (the viewer feed).
async fn drones_per_target() -> Vec<usize> {
    let (mut ws, _) = tokio_tungstenite::connect_async("ws://127.0.0.1:9002").await.unwrap();
    let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await else { return vec![] };
    let s: serde_json::Value = serde_json::from_str(&text).unwrap();
    let mut per_target: BTreeMap<u64, usize> = BTreeMap::new();
    for a in s["agents"].as_array().unwrap().iter().filter(|a| a["alive"].as_bool() == Some(true)) {
        if let Some(id) = a["target_id"].as_u64() {
            *per_target.entry(id).or_default() += 1;
        }
    }
    per_target.into_values().collect()
}

#[tokio::main]
async fn main() {
    let mut gcs = Gcs { socket: UdpSocket::bind("127.0.0.1:0").await.unwrap(), sequence: 0, drones: BTreeMap::new(), next_heartbeat: Instant::now(), stick_seq: 0, signer: Signer::default() };
    gcs.listen(Duration::from_millis(500), None).await;
    check(gcs.signer.ready(), "the base handed over the operator key");
    // Drones take off one a second from the base; wait until the first few are up and away.
    // Take one well out from the base and fly it straight back toward the base: flown away
    // from it, it can leave contact, and then its reports stop reaching us.
    gcs.listen(Duration::from_secs(20), None).await;
    // By where it really is: a drone's status carries its GPS position, a few meters off.
    let mut candidates: Vec<(u8, (f32, f32))> = Vec::new();
    for (&id, d) in gcs.drones.iter().filter(|(_, d)| d.3.elapsed() < Duration::from_secs(2)) {
        if let Some(at) = truth(id).await {
            let _ = d;
            candidates.push((id, at));
        }
    }
    let out = |p: &(f32, f32)| (p.0 + 13.0).hypot(p.1 + 13.0);
    let (id, start) = candidates
        .into_iter()
        .filter(|(_, p)| out(p) > 7.0)
        .min_by(|a, b| out(&a.1).total_cmp(&out(&b.1)))
        .expect("a drone in contact, away from the base");
    println!("     taking drone {id} at ({:.1}, {:.1})", start.0, start.1);

    // 1. Full stick toward the base for 3 s. Stick x is forward (north), y right (east).
    let (dx, dy) = (-13.0 - start.0, -13.0 - start.1);
    let len = dx.hypot(dy);
    let (forward, right) = ((dy / len * 1000.0) as i16, (dx / len * 1000.0) as i16);
    gcs.listen(Duration::from_secs(3), Some((id, forward, right))).await;
    let order = gcs.drones[&id].2;
    let now = truth(id).await.expect("it's in the simulation");
    check(order == 4, &format!("drone {id} reports manual control (order {order})"));
    let closer = (start.0 + 13.0).hypot(start.1 + 13.0) - (now.0 + 13.0).hypot(now.1 + 13.0);
    check(closer > 3.0, &format!("it flew toward the base: ({:.1}, {:.1}) -> ({:.1}, {:.1})", start.0, start.1, now.0, now.1));

    // 2. Let go of the sticks entirely: their signal stops reaching it. It notices after a
    // second and says so at once.
    gcs.listen(Duration::from_millis(2500), None).await;
    let d = gcs.drones[&id];
    check(d.2 == 5, &format!("sticks lost: it reports hovering (order {}, status {:.1}s old)", d.2, d.3.elapsed().as_secs_f32()));

    // 3. Left like that, it gives up on the sticks and rejoins the swarm on its own.
    gcs.listen(Duration::from_secs(4), None).await;
    check(gcs.drones[&id].2 == 0, &format!("sticks lost for good: it rejoined the swarm (order {})", gcs.drones[&id].2));

    // 4. Take it again, briefly, and release it by hand.
    gcs.listen(Duration::from_secs(1), Some((id, 0, 0))).await;
    check(gcs.drones[&id].2 == 4, "taken again");
    for attempt in 0..4 {
        gcs.send(&command(id, MavCmd::MAV_CMD_MISSION_START, 0.0, attempt)).await;
        gcs.listen(Duration::from_millis(800), None).await;
        if gcs.drones[&id].2 == 0 {
            break;
        }
    }
    check(gcs.drones[&id].2 == 0, "released back to the swarm");

    // 5. Two drones per target, repeated every few seconds like the remote does.
    for repeat in 0..8u8 {
        gcs.send(&command(0, MavCmd::MAV_CMD_SWARM_SET_RESPONDERS, 2.0, repeat)).await;
        gcs.listen(Duration::from_secs(5), None).await;
    }
    let mut counts = Vec::new();
    for _ in 0..5 {
        counts.extend(drones_per_target().await);
        gcs.listen(Duration::from_secs(1), None).await;
    }
    let crowded = counts.iter().filter(|&&n| n > 3).count();
    println!("     drones on each target, over 5 samples: {counts:?}");
    check(crowded * 5 <= counts.len(), "with 2 per target, targets mostly have at most 3 drones on them");
    println!("all checks passed");
}
