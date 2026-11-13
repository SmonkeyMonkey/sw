//! The ground station's two links, run on a background thread with its own tokio runtime:
//!  - radio: through the emulated radio (netem) as system ID 255, from the base, so it only
//!    reaches the drones near the base and everything else arrives relayed by the swarm;
//!  - base: wired to the launcher (plain UDP), for launching drones;
//!  - QGroundControl, if one is listening: the swarm as standard vehicles (see qgc.rs).
//!
//! Bevy talks to it through two plain channels, like the viewer's WebSocket client.
//!
//! Orders and stick input go on the air signed with the mission's operator key (see
//! `Signer`), which the base hands over the wire in answer to our heartbeat: the drones take
//! nothing else, so nobody without the key can give them orders, a captured drone included.
//! The rest of the ground station (and QGroundControl) deals in plain COMMAND_LONG and
//! MANUAL_CONTROL: they're signed here, on the way out. And every radio frame, ours and the
//! swarm's, is signed with the mission's link key, also from the base (see
//! `swarm_brain::radio`): until it comes, the radio is silent both ways.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};

use mavlink_swarm::mavlink_core::{write_v2_msg, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{
    MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, COMMAND_LONG_DATA, HEARTBEAT_DATA, SWARM_OPERATOR_KEY_DATA,
    SWARM_SIGNED_COMMAND_DATA,
};
use swarm_brain::auth::{self, Operator};
use swarm_brain::radio;
use tokio::net::UdpSocket;

use crate::qgc::{self, Bridge};

/// MAVLink's conventional system ID for a ground station.
pub const GCS_SYSID: u8 = 255;
/// MAV_COMP_ID_MISSIONPLANNER: the usual component ID for ground station software.
const GCS_COMPONENT: u8 = 190;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Link {
    Radio,
    Base,
}

/// Something heard on one of the links.
pub struct Heard {
    pub link: Link,
    pub header: MavHeader,
    pub msg: MavMessage,
}

/// Something to send.
pub struct Send {
    pub link: Link,
    pub msg: MavMessage,
}

pub fn spawn(radio: SocketAddr, base: SocketAddr, qgc: SocketAddr) -> (Receiver<Heard>, Sender<Send>) {
    let (heard_tx, heard_rx) = std::sync::mpsc::channel();
    let (send_tx, send_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to build tokio runtime for the ground station links");
        rt.block_on(run(radio, base, qgc, heard_tx, send_rx));
    });
    (heard_rx, send_tx)
}

async fn run(radio: SocketAddr, base: SocketAddr, qgc: SocketAddr, heard: Sender<Heard>, to_send: Receiver<Send>) {
    let Ok(socket) = UdpSocket::bind("127.0.0.1:0").await else {
        eprintln!("ground station: can't bind a UDP socket");
        return;
    };
    let mut sequence: u8 = 0;
    let mut buf = [0u8; 512];
    let mut heartbeat = tokio::time::interval(Duration::from_secs(1));
    let mut poll = tokio::time::interval(Duration::from_millis(20));
    let mut bridge = Bridge::default();
    let mut qgc_sequences = [0u8; 256];
    let mut signer = Signer::default();

    loop {
        tokio::select! {
            // A HEARTBEAT a second, like any MAVLink node: it's also how the radio emulator
            // learns where to deliver frames for us.
            _ = heartbeat.tick() => {
                let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: MavType::MAV_TYPE_GCS,
                    autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                    base_mode: MavModeFlag::empty(),
                    system_status: MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                });
                send_signed(&socket, radio, &mut sequence, &mut signer, &hb).await;
                // And to the base, which answers with the operator key.
                send(&socket, base, &mut sequence, &hb).await;
            }
            // Commands from the UI are rare, so polling the channel is plenty.
            _ = poll.tick() => {
                while let Ok(out) = to_send.try_recv() {
                    match out.link {
                        Link::Radio => send_signed(&socket, radio, &mut sequence, &mut signer, &out.msg).await,
                        Link::Base => send(&socket, base, &mut sequence, &out.msg).await,
                    }
                }
                let mut out = qgc::Out::default();
                bridge.tick(Instant::now(), &mut out);
                flush(&socket, radio, qgc, &mut sequence, &mut qgc_sequences, &mut signer, out).await;
            }
            received = socket.recv_from(&mut buf) => {
                let Ok((len, from)) = received else { continue };
                let mut reader = MavlinkReader::new(&buf[..len]);
                if from == qgc {
                    // QGroundControl: handled here, on the swarm's behalf.
                    let mut out = qgc::Out::default();
                    while let Ok((_, msg)) = reader.read_message::<MavMessage>(MavlinkVersion::V2) {
                        bridge.on_qgc(&msg, Instant::now(), &mut out);
                    }
                    flush(&socket, radio, qgc, &mut sequence, &mut qgc_sequences, &mut signer, out).await;
                    continue;
                }
                let link = if from == base { Link::Base } else { Link::Radio };
                while let Ok(MAVLinkMessageRaw::V2(raw)) = reader.read_raw_message::<MavMessage>(MavlinkVersion::V2) {
                    // The radio's signed with the link key, the base's wire isn't.
                    let frame = match link {
                        Link::Radio => signer.open(&raw),
                        Link::Base => radio::parse(&raw),
                    };
                    let Some((header, msg)) = frame else { continue };
                    if let MavMessage::SWARM_OPERATOR_KEY(key) = &msg {
                        // From the base's wire only: anyone on the air could send one.
                        if link == Link::Base {
                            signer.on_key(key);
                        }
                        continue;
                    }
                    if matches!(link, Link::Radio) {
                        let mut out = qgc::Out::default();
                        bridge.on_radio(&msg, Instant::now(), &mut out);
                        flush(&socket, radio, qgc, &mut sequence, &mut qgc_sequences, &mut signer, out).await;
                    }
                    if heard.send(Heard { link, header, msg }).is_err() {
                        return;
                    }
                }
            }
        }
    }
}

/// Sends what the QGC bridge produced: frames for QGC under each drone's own system ID (as if
/// they came from the drone), and commands for the swarm over the radio.
async fn flush(socket: &UdpSocket, radio: SocketAddr, qgc: SocketAddr, sequence: &mut u8, sequences: &mut [u8; 256], signer: &mut Signer, out: qgc::Out) {
    for frame in out.to_qgc {
        let seq = &mut sequences[frame.from as usize];
        let header = MavHeader { system_id: frame.from, component_id: 1, sequence: *seq };
        *seq = seq.wrapping_add(1);
        let mut bytes = Vec::with_capacity(300);
        if write_v2_msg(&mut bytes, header, &frame.msg).is_ok() {
            // Nobody listening is fine: QGC may simply not be running.
            let _ = socket.send_to(&bytes, qgc).await;
        }
    }
    for msg in out.to_radio {
        send_signed(socket, radio, sequence, signer, &msg).await;
    }
}

/// Onto the radio: orders and sticks signed with the operator key, every frame with the link
/// key, or not at all without them.
async fn send_signed(socket: &UdpSocket, radio: SocketAddr, sequence: &mut u8, signer: &mut Signer, msg: &MavMessage) {
    let header = MavHeader { system_id: GCS_SYSID, component_id: GCS_COMPONENT, sequence: *sequence };
    let Some(msg) = signer.sign(msg) else { return };
    let Some(frame) = signer.on_air(header, &msg) else { return };
    *sequence = sequence.wrapping_add(1);
    let _ = socket.send_to(frame.raw_bytes(), radio).await;
}

/// How long the repeats of an unacknowledged order go out as the first one was signed; past
/// that it's signed anew, well before the drones would find it stale
/// (`swarm_brain`'s `ORDER_MAX_AGE_MS`, 10 s).
const REPEAT_AS_SIGNED: Duration = Duration::from_secs(4);

/// Signs what the ground station sends the swarm (see `swarm_brain::auth`): with the mission's
/// operator key, dated by the mission clock, both from the base.
#[derive(Default)]
pub struct Signer {
    /// The operator key, and when the mission clock read 0 here.
    key: Option<(Operator, Instant)>,
    /// The radio link key, and our signer with it.
    link: Option<([u8; 32], radio::Signer)>,
    /// Per drone (0: everyone): the last order signed for it, and when. A repeat goes out as the
    /// same frame: signed anew, it would be a new order to the drones, and one repeated after a
    /// later order would undo it.
    last: HashMap<u8, (SWARM_SIGNED_COMMAND_DATA, Instant)>,
    /// Said that there's no key yet.
    warned: bool,
}

impl Signer {
    pub fn on_key(&mut self, k: &SWARM_OPERATOR_KEY_DATA) {
        let zero = Instant::now() - Duration::from_millis(k.mission_ms as u64);
        match &mut self.key {
            // The same mission: only the clock, kept in step.
            Some((operator, at)) if operator.mission() == k.mission => *at = zero,
            _ => {
                self.key = Some((Operator::new(k.mission, k.secret_key), zero));
                self.link = Some((k.link_key, radio::Signer::new(k.link_key, GCS_SYSID)));
                self.last.clear();
                self.warned = false;
                println!("ground station: operator key for mission {}", k.mission);
            }
        }
    }

    /// The mission clock as we have it.
    fn now_ms(&self) -> Option<u32> {
        self.key.as_ref().map(|(_, zero)| zero.elapsed().as_millis() as u32)
    }

    /// `msg` as a frame on the air: signed with the link key. `None` without it.
    pub fn on_air(&mut self, header: MavHeader, msg: &MavMessage) -> Option<MAVLinkV2MessageRaw> {
        let now_ms = self.now_ms()?;
        let (_, signer) = self.link.as_mut()?;
        Some(signer.frame(header, msg, now_ms))
    }

    /// A frame off the air, if it's signed with the link key (see `swarm_brain::radio::open`).
    pub fn open(&self, raw: &MAVLinkV2MessageRaw) -> Option<(MavHeader, MavMessage)> {
        let now_ms = self.now_ms()?;
        let (key, _) = self.link.as_ref()?;
        radio::open(raw, key, now_ms)
    }

    /// What goes on the air for `msg`: orders and stick input signed, anything else as it is.
    /// `None`: an order, but no key to sign it with yet.
    pub fn sign(&mut self, msg: &MavMessage) -> Option<MavMessage> {
        if !matches!(msg, MavMessage::COMMAND_LONG(_) | MavMessage::MANUAL_CONTROL(_)) {
            return Some(msg.clone());
        }
        let Some((operator, zero)) = &mut self.key else {
            if !self.warned {
                eprintln!("ground station: no operator key from the base yet, so no orders go out");
                self.warned = true;
            }
            return None;
        };
        let now_ms = zero.elapsed().as_millis() as u32;
        match msg {
            MavMessage::COMMAND_LONG(cmd) => {
                let plain = COMMAND_LONG_DATA { confirmation: 0, target_component: 0, ..cmd.clone() };
                if let Some((signed, at)) = self.last.get(&cmd.target_system) {
                    if auth::command_of(signed) == plain && at.elapsed() < REPEAT_AS_SIGNED {
                        return Some(MavMessage::SWARM_SIGNED_COMMAND(signed.clone()));
                    }
                }
                let signed = operator.sign_command(&plain, now_ms);
                self.last.insert(cmd.target_system, (signed.clone(), Instant::now()));
                Some(MavMessage::SWARM_SIGNED_COMMAND(signed))
            }
            MavMessage::MANUAL_CONTROL(m) => Some(MavMessage::SWARM_SIGNED_STICKS(operator.sign_sticks(m, now_ms))),
            _ => unreachable!("only orders get here"),
        }
    }
}

async fn send(socket: &UdpSocket, to: SocketAddr, sequence: &mut u8, msg: &MavMessage) {
    let header = MavHeader { system_id: GCS_SYSID, component_id: GCS_COMPONENT, sequence: *sequence };
    *sequence = sequence.wrapping_add(1);
    let mut bytes = Vec::with_capacity(300);
    if write_v2_msg(&mut bytes, header, msg).is_ok() {
        let _ = socket.send_to(&bytes, to).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_swarm::{MavCmd, MANUAL_CONTROL_DATA};

    fn rtl(target_system: u8, confirmation: u8) -> MavMessage {
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
            target_system,
            target_component: 0,
            confirmation,
        })
    }

    #[test]
    fn orders_go_out_signed_and_only_with_the_key() {
        let mut signer = Signer::default();
        assert!(signer.sign(&rtl(3, 0)).is_none(), "no key yet");
        let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        assert_eq!(signer.sign(&hb), Some(hb.clone()), "anything else as it is");

        signer.on_key(&SWARM_OPERATOR_KEY_DATA { mission: 4, mission_ms: 60_000, secret_key: [9; 32], link_key: [3; 32] });
        let Some(MavMessage::SWARM_SIGNED_COMMAND(first)) = signer.sign(&rtl(3, 0)) else { panic!("not signed") };
        assert_eq!((first.mission, first.target_system), (4, 3));
        assert!((60_000..61_000).contains(&first.issued_ms), "dated by the mission clock: {}", first.issued_ms);
        // A repeat (MAVLink counts `confirmation` up) goes out as the same frame.
        assert_eq!(signer.sign(&rtl(3, 1)), Some(MavMessage::SWARM_SIGNED_COMMAND(first.clone())));
        // Another drone's order is its own.
        let Some(MavMessage::SWARM_SIGNED_COMMAND(other)) = signer.sign(&rtl(5, 0)) else { panic!("not signed") };
        assert!(other.issued_ms > first.issued_ms);

        let sticks = MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA { x: 1000, y: 0, z: 500, r: 0, buttons: 1, target: 3 });
        assert!(matches!(signer.sign(&sticks), Some(MavMessage::SWARM_SIGNED_STICKS(s)) if s.target == 3 && s.x == 1000));
    }

    #[test]
    fn the_radio_is_silent_both_ways_without_the_link_key() {
        let header = MavHeader { system_id: GCS_SYSID, component_id: GCS_COMPONENT, sequence: 0 };
        let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        let mut signer = Signer::default();
        assert!(signer.on_air(header, &hb).is_none());
        let theirs = radio::Signer::new([3; 32], 7).frame(header, &hb, 60_000);
        assert!(signer.open(&theirs).is_none());

        signer.on_key(&SWARM_OPERATOR_KEY_DATA { mission: 4, mission_ms: 60_000, secret_key: [9; 32], link_key: [3; 32] });
        let ours = signer.on_air(header, &hb).expect("signed");
        assert_eq!(radio::open(&ours, &[3; 32], 60_000), Some((header, hb.clone())));
        assert_eq!(signer.open(&theirs), Some((header, hb)));
    }
}
