//! The firmware's main loop (firmware/src/main.rs) running on the host, for testing the HIL
//! bridge without the board plugged in. Same brain, same SWARM_HIL_* protocol, same tick
//! structure; only the transport differs (TCP instead of the ST-Link's UART).
//!
//!   cargo run -- --hil tcpin:127.0.0.1:5760      # backend: drone 1 waits for a brain here
//!   cargo run --example fake_board               # this: connects and flies drone 1
//!
//! It takes as long over its signatures as the board would (measured on it, see the firmware's
//! Cargo.toml), so it falls behind when the board would; `--fast` for the host's own speed.

use std::time::Duration;

use mavlink_swarm::mavlink_core::{connect_async, AsyncMavConnection, MAVLinkMessageRaw, MavHeader};
use mavlink_swarm::MavMessage;
use swarm_brain::hil::{self, command_msg, sensor_reading, trace_event_msg};
use swarm_brain::hil::Mission;
use swarm_brain::{radio, Brain, COMPONENT_ID, MAX_INBOX};

const SPAN_TIMEOUT: Duration = Duration::from_millis(10);

/// On the board (Cortex-M7, 216 MHz): checking a signature, unpacking the key included, and
/// making one.
const CHECK_ON_BOARD: Duration = Duration::from_micros(6_600);
const SIGN_ON_BOARD: Duration = Duration::from_micros(2_400);

#[tokio::main]
async fn main() {
    let fast = std::env::args().any(|a| a == "--fast");
    let address = std::env::args().skip(1).find(|a| !a.starts_with("--")).unwrap_or_else(|| "tcpout:127.0.0.1:5760".into());
    // The backend's bridge drops the link on every mission reset; just plug back in, like a
    // board that stays powered while the simulator restarts.
    loop {
        if let Ok(link) = connect_async::<MavMessage>(&address).await {
            println!("fake board: connected to {address}");
            if let Err(e) = fly(link.as_ref(), fast).await {
                println!("fake board: link lost ({e}), reconnecting");
            }
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

async fn fly(link: &(dyn AsyncMavConnection<MavMessage> + Sync + Send), fast: bool) -> Result<(), Box<dyn std::error::Error>> {
    let mut brain: Option<Brain> = None;
    let mut mission: Option<Mission> = None;
    let mut inbox: Vec<(MavHeader, MavMessage)> = Vec::with_capacity(MAX_INBOX);
    let mut ticks: u32 = 0;
    let mut dropped: u32 = 0;
    let mut last_time_ms: u32 = 0;
    let mut spans: u32 = 0;
    let mut span_timeouts: u32 = 0;
    // As on the board: our radio frames signed with the mission's link key, and only signed
    // ones heard (see `swarm_brain::radio`).
    let mut signer: Option<radio::Signer> = None;
    let mut unsigned: u32 = 0;

    loop {
        let (raw, header, msg) = recv(link).await?;
        let sensor = match msg {
            MavMessage::SWARM_HIL_SENSOR(sensor) => sensor,
            MavMessage::SWARM_HIL_TRACE_SPAN(_) => continue,
            // A different one is a new mission.
            MavMessage::SWARM_HIL_MISSION(m) => {
                let m = hil::mission(&m);
                if m.is_some() && mission != m {
                    mission = m;
                    brain = None;
                    inbox.clear();
                }
                continue;
            }
            radio => {
                if !mission.is_some_and(|m| radio::signed(&raw, &m.keys.link, last_time_ms)) {
                    unsigned += 1;
                } else if inbox.len() < MAX_INBOX {
                    inbox.push((header, radio));
                } else {
                    dropped += 1;
                }
                continue;
            }
        };

        // Same as the firmware: the clock going backwards means the mission was reset.
        if sensor.time_ms < last_time_ms {
            println!("fake board: new mission");
            brain = None;
            inbox.clear();
        }
        last_time_ms = sensor.time_ms;
        // No mission for this drone yet: sit the tick out (the bridge sends it every second).
        let Some(mission) = mission.filter(|m| m.keys.sysid == sensor.sysid) else { continue };
        if brain.is_none() {
            signer = Some(radio::Signer::new(mission.keys.link, sensor.sysid));
        }
        let brain = brain.get_or_insert_with(|| {
            println!(
                "fake board: flying as drone {} (mission {}, field {:?}, base {:?}, {:?})",
                sensor.sysid, mission.keys.mission, mission.field_half * 2.0, mission.home, mission.mode
            );
            Brain::new(sensor.sysid, mission.config(), 0x5EED, mission.keys)
        });
        let reading = sensor_reading(&sensor);
        let (checks, _) = brain.crypto_work();
        let out = brain.on_tick(&reading, &inbox);
        inbox.clear();
        if !fast {
            tokio::time::sleep(CHECK_ON_BOARD * (brain.crypto_work().0 - checks)).await;
        }

        let header = MavHeader { system_id: brain.sysid(), component_id: COMPONENT_ID, sequence: 0 };
        if let Some(event) = out.event {
            link.send(&header, &trace_event_msg(event)).await?;
            let wait = async {
                loop {
                    match recv(link).await? {
                        (_, _, MavMessage::SWARM_HIL_TRACE_SPAN(span)) => return Ok(Some((span.trace_id, span.span_id))),
                        (_, _, MavMessage::SWARM_HIL_SENSOR(_)) => return Ok(None),
                        (raw, _, _) if !radio::signed(&raw, &mission.keys.link, reading.time_ms) => unsigned += 1,
                        (_, header, msg) if inbox.len() < MAX_INBOX => inbox.push((header, msg)),
                        _ => dropped += 1,
                    }
                }
            };
            match tokio::time::timeout(SPAN_TIMEOUT, wait).await {
                Ok(Ok(Some((trace_id, span_id)))) => {
                    brain.attach_span(trace_id, span_id);
                    spans += 1;
                }
                Ok(Err::<_, mavlink_swarm::mavlink_core::error::MessageReadError>(e)) => return Err(e.into()),
                _ => span_timeouts += 1,
            }
        }

        let (_, signatures) = brain.crypto_work();
        let frames = brain.outgoing_frames(reading.time_ms);
        if !fast {
            tokio::time::sleep(SIGN_ON_BOARD * (brain.crypto_work().1 - signatures)).await;
        }
        let signer = signer.as_mut().expect("made with the brain");
        for (relayed, msg) in frames.into_iter().flatten() {
            let frame = signer.frame(relayed.unwrap_or(header), &msg, reading.time_ms);
            link.send_raw(&MAVLinkMessageRaw::V2(frame)).await?;
        }
        link.send(&header, &command_msg(brain, &reading, &out)).await?;

        ticks += 1;
        if ticks.is_multiple_of(100) {
            println!(
                "fake board: t={}ms pos=({:.1},{:.1}) mode={:?} informed={} spans={spans} span_timeouts={span_timeouts} dropped={dropped} unsigned={unsigned} sightings (forged, unchecked)={:?} orders forged={}",
                reading.time_ms,
                reading.own_pos.x,
                reading.own_pos.y,
                brain.mode(),
                brain.informed(),
                brain.sightings_rejected(),
                brain.orders_rejected()
            );
        }
    }
}

/// The next frame, as it came (to check its signature) and read.
async fn recv(
    link: &(dyn AsyncMavConnection<MavMessage> + Sync + Send),
) -> Result<(mavlink_swarm::mavlink_core::MAVLinkV2MessageRaw, MavHeader, MavMessage), mavlink_swarm::mavlink_core::error::MessageReadError> {
    loop {
        if let MAVLinkMessageRaw::V2(raw) = link.recv_raw().await? {
            if let Some((header, msg)) = radio::parse(&raw) {
                return Ok((raw, header, msg));
            }
        }
    }
}
