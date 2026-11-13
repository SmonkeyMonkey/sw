//! Network emulator: the stand-in for the physical radio channel between drones.
//!
//! Every drone sends its MAVLink frames to this one UDP socket, the way a real drone hands
//! frames to its radio modem. This task then decides who would actually have heard each frame
//! — only drones within radio range of the sender's *true* position — and forwards the
//! original bytes to them after a random delay, dropping some at random. Same idea as real
//! network emulators like ns-3 or Linux netem: the endpoints speak a real protocol over a real
//! socket, and only the medium between them is simulated. Drones can't tell whether a silent
//! neighbor is out of range, lost a packet, or is gone — exactly the uncertainty a real swarm
//! has to deal with.
//!
//! Who sent a frame is told by where it came from (its UDP address, like the radio it was
//! transmitted on), not by its header: a relayed frame keeps its original sender's header (see
//! `swarm_brain::Frame`), and goes out from the relaying drone's position. Each address is
//! bound to the system ID of its first frame, and no other address can then transmit as that
//! system ID while it's still on the air.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use mavlink_swarm::mavlink_core::{Message, MavlinkReader, MavlinkVersion};
use mavlink_swarm::MavMessage;
use opentelemetry::KeyValue;
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use tokio::net::UdpSocket;

use crate::observability::Telemetry;
use swarm_brain::Vec2;

pub struct NetemConfig {
    pub range: f32,
    pub base_latency: Duration,
    /// Extra uniformly random delay on top of `base_latency`, per delivery.
    pub jitter: Duration,
    /// Probability that a given delivery to a given receiver is lost.
    pub drop_chance: f64,
}

/// A system ID silent this long from its address can be taken by another: the ground station
/// restarted, on a new port.
const REBIND_AFTER: Duration = Duration::from_secs(5);

/// Which radio transmits as which system ID.
#[derive(Default)]
struct Radios {
    /// Each system ID's address, and when it was last heard from there.
    addr_of: HashMap<u8, (SocketAddr, Instant)>,
    sysid_at: HashMap<SocketAddr, u8>,
}

impl Radios {
    /// Who sent a frame from `from` with this header's system ID: the address's own system ID,
    /// or for an address not heard before, the one it claims, if nobody else is transmitting
    /// as that. `None`: an impostor (or a new drone's relay sent before any frame of its own).
    fn sender(&mut self, from: SocketAddr, claimed: u8, now: Instant) -> Option<u8> {
        let sysid = match self.sysid_at.get(&from) {
            Some(&sysid) => sysid,
            None => {
                if self.addr_of.get(&claimed).is_some_and(|&(addr, at)| addr != from && now - at < REBIND_AFTER) {
                    return None;
                }
                if let Some((old, _)) = self.addr_of.get(&claimed) {
                    self.sysid_at.remove(old);
                }
                self.sysid_at.insert(from, claimed);
                claimed
            }
        };
        self.addr_of.insert(sysid, (from, now));
        Some(sysid)
    }
}

/// `positions[i]` is the true position of the drone with MAVLink system ID `i + 1`; `stations`
/// are fixed radios that aren't drones (the operator's ground station at the base). `mission`
/// counts missions: every new one has new drones on new sockets, under the same system IDs.
pub async fn run(
    socket: UdpSocket,
    cfg: NetemConfig,
    positions: Arc<RwLock<Vec<Vec2>>>,
    stations: Arc<RwLock<Vec<(u8, Vec2)>>>,
    mission: Arc<AtomicU64>,
    telemetry: Arc<Telemetry>,
) {
    let socket = Arc::new(socket);
    let mut rng = StdRng::from_rng(&mut rand::rng());
    let mut radios = Radios::default();
    let mut radios_mission = mission.load(Ordering::Relaxed);
    let mut buf = [0u8; 512];

    loop {
        let Ok((len, from_addr)) = socket.recv_from(&mut buf).await else { continue };
        let frame = buf[..len].to_vec();

        // Parse to validate the frame and learn who sent it. The emulator only forwards the
        // original bytes — it never alters drones' messages.
        let Ok((header, msg)) = MavlinkReader::new(&frame[..]).read_message::<MavMessage>(MavlinkVersion::V2) else {
            telemetry.netem_malformed.add(1, &[]);
            continue;
        };
        let msg_label = [KeyValue::new("msg", msg.message_name())];
        telemetry.netem_frames_in.add(1, &msg_label);
        // A board's link to its body is a wire, not the radio (see `hil::is_body_link`), and
        // so is the base's to the ground station: the operator key never goes on the air.
        if swarm_brain::hil::is_body_link(&msg) || matches!(msg, MavMessage::SWARM_OPERATOR_KEY(_)) {
            telemetry.netem_impostor.add(1, &msg_label);
            continue;
        }
        let current = mission.load(Ordering::Relaxed);
        if current != radios_mission {
            radios = Radios::default();
            radios_mission = current;
        }
        let Some(sender) = radios.sender(from_addr, header.system_id, Instant::now()) else {
            telemetry.netem_impostor.add(1, &msg_label);
            continue;
        };

        // Snapshot recipients and release the lock before awaiting anything.
        let (in_range, out_of_range): (Vec<SocketAddr>, u64) = {
            let positions = positions.read().unwrap();
            let stations = stations.read().unwrap();
            let position_of = |sysid: u8| {
                stations
                    .iter()
                    .find(|(id, _)| *id == sysid)
                    .map(|&(_, p)| p)
                    .or_else(|| positions.get(sysid.wrapping_sub(1) as usize).copied())
            };
            let Some(origin) = position_of(sender) else { continue };
            let mut in_range = Vec::new();
            let mut out_of_range = 0;
            for (&sysid, &(addr, _)) in &radios.addr_of {
                if sysid == sender {
                    continue;
                }
                let close = position_of(sysid).is_some_and(|p| (p - origin).length() <= cfg.range);
                if close {
                    in_range.push(addr);
                } else {
                    out_of_range += 1;
                }
            }
            (in_range, out_of_range)
        };
        telemetry.netem_out_of_range.add(out_of_range, &msg_label);

        for addr in in_range {
            if rng.random_bool(cfg.drop_chance) {
                telemetry.netem_lost.add(1, &msg_label);
                continue;
            }
            let delay = cfg.base_latency + cfg.jitter.mul_f64(rng.random::<f64>());
            telemetry.netem_delivery_delay.record(delay.as_secs_f64(), &[]);
            let socket = socket.clone();
            let telemetry = telemetry.clone();
            let frame = frame.clone();
            let msg_label = msg_label.clone();
            // One task per delivery, so latency is per-receiver and nothing blocks the loop.
            tokio::spawn(async move {
                tokio::time::sleep(delay).await;
                if socket.send_to(&frame, addr).await.is_ok() {
                    telemetry.netem_delivered.add(1, &msg_label);
                }
            });
        }
    }
}
