//! Runs one simulated drone: connects a `swarm_brain::Brain` to its body (sensor readings in,
//! velocity commands out), to the network (MAVLink v2 over UDP), and to telemetry. The brain
//! itself lives in the `swarm-brain` crate so the STM32 firmware can run the very same code.

use std::net::SocketAddr;
use std::sync::Arc;

use mavlink_swarm::mavlink_core::{MAVLinkMessageRaw, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{MavCmd, MavMessage, COMMAND_LONG_DATA, SWARM_TARGET_SIGHTING_DATA};
use swarm_brain::auth::Operator;
use swarm_brain::{radio, Brain, SensorReading, TraceEvent, Vec2, COMPONENT_ID, GCS_SYSID, MAX_INBOX};
use tokio::net::UdpSocket;
use tokio::sync::mpsc;

use crate::body::{AgentReport, Fault};
use crate::observability::Telemetry;

pub struct AgentLinks {
    pub index: usize,
    pub sensor_rx: mpsc::Receiver<SensorReading>,
    pub report_tx: mpsc::Sender<AgentReport>,
    /// Where this drone's radio sends everything; the network emulator decides who hears it.
    pub netem_addr: SocketAddr,
    pub fault_rx: mpsc::Receiver<Fault>,
    pub telemetry: Arc<Telemetry>,
}

/// Runs one drone. Network I/O is real UDP carrying real MAVLink v2 frames; only the radio's
/// range/loss/latency is emulated, by the netem task this socket talks to.
pub async fn run(mut brain: Brain, mut links: AgentLinks) -> std::io::Result<()> {
    let socket = UdpSocket::bind("127.0.0.1:0").await?;
    let mut inbox: Vec<(MavHeader, MavMessage)> = Vec::new();
    let mut recv_buf = [0u8; 512];
    // Every frame on the air signed with the mission's link key, and only signed ones heard (see
    // `swarm_brain::radio`), by the mission clock: the last reading's.
    let link_key = brain.keys().link;
    let mut signer = radio::Signer::new(link_key, brain.sysid());
    let mut clock_ms = 0;
    let mut sequence: u8 = 0;
    let telemetry = links.telemetry.clone();
    // Set once the drone is compromised: (fake target position, next time to broadcast it).
    let mut lie: Option<(Vec2, u32)> = None;
    let beacon_period_ms = brain.config().beacon_period_ms;
    // What the brain has rejected so far, to report only what's new.
    let mut rejected = (0, 0);
    let mut orders_rejected = 0;

    loop {
        tokio::select! {
            reading = links.sensor_rx.recv() => {
                let Some(reading) = reading else { return Ok(()) };
                clock_ms = reading.time_ms;
                while let Ok(Fault::Lie { fake_pos }) = links.fault_rx.try_recv() {
                    lie = Some((fake_pos, reading.time_ms));
                }
                let was_informed = brain.informed();

                let out = brain.on_tick(&reading, &inbox);
                inbox.clear();

                if let Some(event) = out.event {
                    let (trace_id, span_id) = telemetry.record_event(brain.sysid(), event);
                    brain.attach_span(trace_id, span_id);
                }
                for lost in out.lost.into_iter().flatten() {
                    telemetry.neighbor_lost(brain.sysid(), lost.sysid, lost.silent_ms, reading.time_ms);
                }
                if let Some(observer) = out.refuted {
                    telemetry.sighting_refuted(brain.sysid(), observer);
                }
                let now_rejected = brain.sightings_rejected();
                telemetry.sightings_rejected(brain.sysid(), now_rejected.0 - rejected.0, now_rejected.1 - rejected.1);
                rejected = now_rejected;
                telemetry.orders_rejected(brain.sysid(), brain.orders_rejected() - orders_rejected);
                orders_rejected = brain.orders_rejected();
                if !was_informed && brain.informed() {
                    let source = if matches!(out.event, Some(TraceEvent::Spotted)) { "firsthand" } else { "relayed" };
                    telemetry.became_informed(brain.sysid(), source);
                }

                let mut outgoing = brain.outgoing_frames(reading.time_ms);
                let mut injected = [None, None];
                // A compromised radio drops the (still honest) brain's sightings and injects a
                // fake firsthand one every beacon period instead, signed with the drone's own
                // key: it can lie in its own name, not in anyone else's (see `swarm_brain::auth`).
                if let Some((fake_pos, next_ms)) = lie {
                    for slot in outgoing.iter_mut().filter(|f| matches!(f, Some((None, MavMessage::SWARM_TARGET_SIGHTING(_))))) {
                        *slot = None;
                    }
                    if reading.time_ms >= next_ms {
                        lie = Some((fake_pos, reading.time_ms + beacon_period_ms));
                        let mut fake = SWARM_TARGET_SIGHTING_DATA {
                            // Looks like a track of its own, as a real first sighting would.
                            target_id: ((brain.sysid() as u16) << 8) | 0xFF,
                            age_ms: 0,
                            x: fake_pos.x,
                            y: fake_pos.y,
                            vx: 0.0,
                            vy: 0.0,
                            observer_sysid: brain.sysid(),
                            cancelled: 0,
                            responders: 0,
                            hops: 0,
                            trace_id: [0; 16],
                            parent_span_id: [0; 8],
                            observed_ms: 0,
                            signature: [0; 64],
                        };
                        brain.keys().sign_sighting(&mut fake, reading.time_ms);
                        outgoing[2] = Some((None, MavMessage::SWARM_TARGET_SIGHTING(fake)));
                        // It tries to take the swarm over too: an order for the next drone to
                        // come home, in the ground station's name, both unsigned and signed
                        // with the only key it has, its own. Drones refuse both (see
                        // `swarm_brain::auth`): telemetry counts them as `order_rejected`.
                        let header = MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 };
                        let rtl = takeover_order(brain.sysid() % 250 + 1);
                        let forged = Operator::new(brain.keys().mission, brain.keys().secret).sign_command(&rtl, reading.time_ms);
                        injected = [Some((Some(header), MavMessage::COMMAND_LONG(rtl))), Some((Some(header), MavMessage::SWARM_SIGNED_COMMAND(forged)))];
                    }
                }

                for (relayed, msg) in outgoing.into_iter().chain(injected).flatten() {
                    // A relay goes out under the header it came with (see `swarm_brain::Frame`).
                    let header = relayed.unwrap_or_else(|| {
                        let own = MavHeader { system_id: brain.sysid(), component_id: COMPONENT_ID, sequence };
                        sequence = sequence.wrapping_add(1);
                        own
                    });
                    let frame = signer.frame(header, &msg, reading.time_ms);
                    socket.send_to(frame.raw_bytes(), links.netem_addr).await?;
                }

                let report = AgentReport {
                    index: links.index,
                    time_ms: reading.time_ms,
                    velocity: out.velocity,
                    informed: brain.informed(),
                    known: brain.known_targets(reading.time_ms),
                    role: brain.role(),
                    isolated: brain.isolated(reading.time_ms),
                    mode: brain.mode(),
                    target: brain.assigned_target(),
                    land: out.land,
                    to_charge: out.to_charge,
                    position: out.position,
                    nav: out.nav,
                };
                if links.report_tx.send(report).await.is_err() {
                    return Ok(());
                }
            }
            received = socket.recv_from(&mut recv_buf) => {
                let (len, _) = received?;
                // CRC-invalid or unknown frames are skipped by the reader, and unsigned ones
                // here; anything else goes in the inbox for the next tick.
                let mut reader = MavlinkReader::new(&recv_buf[..len]);
                while let Ok(raw) = reader.read_raw_message::<MavMessage>(MavlinkVersion::V2) {
                    let MAVLinkMessageRaw::V2(raw) = raw else { continue };
                    let Some(frame) = radio::open(&raw, &link_key, clock_ms) else {
                        telemetry.frame_unsigned(brain.sysid());
                        continue;
                    };
                    telemetry.frame_received(brain.sysid());
                    // As much as the board keeps between two ticks, no more.
                    if inbox.len() < MAX_INBOX {
                        inbox.push(frame);
                    } else {
                        telemetry.inbox_overflow(brain.sysid());
                    }
                }
            }
        }
    }
}

/// What a compromised drone would want the swarm to do: drone `target` to come home.
fn takeover_order(target: u8) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
        target_system: target,
        target_component: 0,
        confirmation: 0,
    }
}
