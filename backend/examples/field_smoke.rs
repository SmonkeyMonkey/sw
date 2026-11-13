//! End-to-end check of setting the field from the ground station: plays the ground station on
//! the base link and asks for a 50 x 30 m field, then watches the simulator (the viewer feed).
//!
//!   cargo run                                # backend
//!   cargo run --example field_smoke          # this
//!
//! The base must acknowledge it, the mission must restart on the new field with the base in its
//! new lower left corner, the 8 drones asked for must take off from there, and stay inside it.

use std::time::{Duration, Instant};

use futures_util::StreamExt;
use mavlink_swarm::mavlink_core::{write_v2_msg, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{MavCmd, MavMessage, MavResult, COMMAND_LONG_DATA};
use tokio::net::UdpSocket;

const BASE: &str = "127.0.0.1:14650";
const BASE_SYSID: u8 = 250;

fn check(ok: bool, what: &str) {
    println!("{} {what}", if ok { "PASS" } else { "FAIL" });
    if !ok {
        std::process::exit(1);
    }
}

#[tokio::main]
async fn main() {
    let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let cmd = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 50.0,
        param2: 30.0,
        // Drones to launch on it.
        param3: 8.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_SWARM_SET_FIELD,
        target_system: BASE_SYSID,
        target_component: 0,
        confirmation: 0,
    });
    let mut bytes = Vec::new();
    write_v2_msg(&mut bytes, MavHeader { system_id: 255, component_id: 190, sequence: 0 }, &cmd).unwrap();
    socket.send_to(&bytes, BASE).await.unwrap();
    let mut buf = [0u8; 512];
    let acked = match tokio::time::timeout(Duration::from_secs(2), socket.recv_from(&mut buf)).await {
        Ok(Ok((len, _))) => {
            let mut reader = MavlinkReader::new(&buf[..len]);
            matches!(reader.read_message::<MavMessage>(MavlinkVersion::V2), Ok((_, MavMessage::COMMAND_ACK(a))) if a.command == MavCmd::MAV_CMD_SWARM_SET_FIELD && a.result == MavResult::MAV_RESULT_ACCEPTED)
        }
        _ => false,
    };
    check(acked, "the base accepted a 50 x 30 m field, with 8 drones");

    let (mut ws, _) = tokio_tungstenite::connect_async("ws://127.0.0.1:9002").await.unwrap();
    let start = Instant::now();
    let (mut on_new_field, mut base_ok, mut outside, mut samples, mut flying) = (false, false, 0u32, 0u32, 0usize);
    while start.elapsed() < Duration::from_secs(30) {
        let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await else { break };
        let s: serde_json::Value = serde_json::from_str(&text).unwrap();
        let f = (s["field_half"]["x"].as_f64().unwrap(), s["field_half"]["y"].as_f64().unwrap());
        if (f.0 - 25.0).abs() > 0.01 || (f.1 - 15.0).abs() > 0.01 {
            continue;
        }
        on_new_field = true;
        base_ok = (s["base"]["x"].as_f64().unwrap() + 23.0).abs() < 0.01 && (s["base"]["y"].as_f64().unwrap() + 13.0).abs() < 0.01;
        if start.elapsed() < Duration::from_secs(20) {
            continue;
        }
        let agents = s["agents"].as_array().unwrap();
        flying = agents.iter().filter(|a| a["alt"].as_f64().unwrap_or(0.0) > 1.0).count();
        for a in agents {
            samples += 1;
            let (x, y) = (a["pos"]["x"].as_f64().unwrap(), a["pos"]["y"].as_f64().unwrap());
            outside += (x.abs() > 25.5 || y.abs() > 15.5) as u32;
        }
    }
    check(on_new_field, "the simulator is on the 50 x 30 m field");
    check(base_ok, "the base is in its lower left corner, at (-23, -13)");
    check(flying == 8, &format!("8 drones took off from it ({flying} flying)"));
    check(outside == 0, &format!("and stay inside the field ({outside} of {samples} samples outside)"));
    println!("all checks passed");
}
