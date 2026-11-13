//! Plays a silent ground station at the base for a while: sends its heartbeat (which is what
//! lets the swarm keep its link to it) and reports, every 10 s, how many drones it has a fresh
//! status from — i.e. how much of the swarm the operator can actually see.
//!
//!   cargo run --example gcs_watch -- 120

use std::collections::HashMap;
use std::time::{Duration, Instant};

use mavlink_swarm::mavlink_core::{write_v2_msg, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, HEARTBEAT_DATA};
use tokio::net::UdpSocket;

#[tokio::main]
async fn main() {
    let seconds: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(60);
    let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let start = Instant::now();
    let mut heard: HashMap<u8, Instant> = HashMap::new();
    let (mut next_heartbeat, mut next_report) = (Instant::now(), Instant::now() + Duration::from_secs(10));
    let mut sequence = 0u8;
    let mut buf = [0u8; 512];
    let mut seen_share = Vec::new();
    while start.elapsed() < Duration::from_secs(seconds) {
        if Instant::now() >= next_heartbeat {
            next_heartbeat += Duration::from_secs(1);
            let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_GCS,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            });
            let mut bytes = Vec::new();
            write_v2_msg(&mut bytes, MavHeader { system_id: 255, component_id: 190, sequence }, &hb).unwrap();
            sequence = sequence.wrapping_add(1);
            socket.send_to(&bytes, "127.0.0.1:14600").await.unwrap();
        }
        if Instant::now() >= next_report {
            next_report += Duration::from_secs(10);
            let fresh = heard.values().filter(|t| t.elapsed() < Duration::from_secs(3)).count();
            println!("t={:3.0}s: fresh status from {fresh}/{} drones", start.elapsed().as_secs_f32(), heard.len());
            if start.elapsed() > Duration::from_secs(25) && !heard.is_empty() {
                seen_share.push(fresh as f32 / heard.len() as f32);
            }
        }
        let Ok(Ok((len, _))) = tokio::time::timeout(Duration::from_millis(50), socket.recv_from(&mut buf)).await else { continue };
        let mut reader = MavlinkReader::new(&buf[..len]);
        while let Ok((_, msg)) = reader.read_message::<MavMessage>(MavlinkVersion::V2) {
            if let MavMessage::SWARM_DRONE_STATUS(st) = msg {
                heard.insert(st.sysid, Instant::now() - Duration::from_millis(st.age_ms as u64));
            }
        }
    }
    if !seen_share.is_empty() {
        println!("average share of the swarm visible after launch: {:.0}%", 100.0 * seen_share.iter().sum::<f32>() / seen_share.len() as f32);
    }
}
