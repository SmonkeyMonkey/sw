//! Stands in for QGroundControl, to check the bridge without installing QGC: listens where QGC
//! does, collects the vehicles it hears, asks one of them what QGC asks every new vehicle, then
//! sends it to a spot given in latitude/longitude and waits for it to get there.
//!
//!   cargo run --release --example mock_qgc

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use mavlink_swarm::mavlink_core::{write_v2_msg, MavHeader, MavlinkReader, MavlinkVersion};
use mavlink_swarm::{
    MavCmd, MavFrame, MavMessage, MavResult, COMMAND_INT_DATA, COMMAND_LONG_DATA, MISSION_REQUEST_LIST_DATA, PARAM_REQUEST_LIST_DATA,
};
use std::net::UdpSocket;
use swarm_gcs::qgc::{to_global, to_local};

fn check(ok: bool, what: &str) {
    println!("{} {what}", if ok { "PASS" } else { "FAIL" });
    if !ok {
        std::process::exit(1);
    }
}

fn main() {
    let socket = UdpSocket::bind("127.0.0.1:14550").expect("is QGroundControl already running?");
    socket.set_read_timeout(Some(Duration::from_millis(100))).unwrap();
    let mut vehicle_link = None;
    let mut positions: BTreeMap<u8, (f64, f64)> = BTreeMap::new();
    let mut replies: Vec<(u8, MavMessage)> = Vec::new();
    let mut seq = 0u8;
    let mut send = |sock: &UdpSocket, to: std::net::SocketAddr, msg: MavMessage| {
        let mut b = Vec::new();
        write_v2_msg(&mut b, MavHeader { system_id: 254, component_id: 190, sequence: seq }, &msg).unwrap();
        seq = seq.wrapping_add(1);
        sock.send_to(&b, to).unwrap();
    };
    let listen = |for_: Duration, positions: &mut BTreeMap<u8, (f64, f64)>, replies: &mut Vec<(u8, MavMessage)>, link: &mut Option<std::net::SocketAddr>| {
        let until = Instant::now() + for_;
        let mut buf = [0u8; 2048];
        while Instant::now() < until {
            let Ok((len, from)) = socket.recv_from(&mut buf) else { continue };
            *link = Some(from);
            let mut r = MavlinkReader::new(&buf[..len]);
            while let Ok((h, msg)) = r.read_message::<MavMessage>(MavlinkVersion::V2) {
                match &msg {
                    MavMessage::GLOBAL_POSITION_INT(p) => {
                        positions.insert(h.system_id, (p.lat as f64 / 1e7, p.lon as f64 / 1e7));
                    }
                    MavMessage::HEARTBEAT(_) | MavMessage::GPS_RAW_INT(_) | MavMessage::HOME_POSITION(_) => {}
                    _ => replies.push((h.system_id, msg)),
                }
            }
        }
    };

    listen(Duration::from_secs(6), &mut positions, &mut replies, &mut vehicle_link);
    check(positions.len() >= 3, &format!("vehicles on the map: {:?}", positions.keys().collect::<Vec<_>>()));
    let link = vehicle_link.unwrap();
    let id = *positions.keys().next().unwrap();
    let (x, y) = to_local(positions[&id].0, positions[&id].1);
    println!("     vehicle {id} at lat {:.6} lon {:.6} = field ({x:.1}, {y:.1})", positions[&id].0, positions[&id].1);

    // What QGC asks a new vehicle.
    send(&socket, link, MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA { target_system: id, target_component: 1 }));
    send(&socket, link, MavMessage::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA { target_system: id, target_component: 1 }));
    send(&socket, link, MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: 148.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0,
        command: MavCmd::MAV_CMD_REQUEST_MESSAGE, target_system: id, target_component: 1, confirmation: 0,
    }));
    listen(Duration::from_secs(1), &mut positions, &mut replies, &mut vehicle_link);
    check(replies.iter().any(|(s, m)| *s == id && matches!(m, MavMessage::PARAM_VALUE(_))), "parameters answered");
    check(replies.iter().any(|(s, m)| *s == id && matches!(m, MavMessage::MISSION_COUNT(_))), "mission answered");
    check(replies.iter().any(|(s, m)| *s == id && matches!(m, MavMessage::AUTOPILOT_VERSION(_))), "autopilot version answered");

    // "Go to location": a spot near the base, given the way QGC gives it.
    let goal = (-8.0f32, -7.0f32);
    let (lat, lon) = to_global(goal.0, goal.1);
    replies.clear();
    send(&socket, link, MavMessage::COMMAND_INT(COMMAND_INT_DATA {
        param1: -1.0, param2: 0.0, param3: 0.0, param4: f32::NAN,
        x: (lat * 1e7).round() as i32, y: (lon * 1e7).round() as i32, z: 5.0,
        command: MavCmd::MAV_CMD_DO_REPOSITION, target_system: id, target_component: 1,
        frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT, current: 0, autocontinue: 0,
    }));
    let started = Instant::now();
    let mut accepted = false;
    let mut arrived = false;
    while started.elapsed() < Duration::from_secs(30) && !(accepted && arrived) {
        listen(Duration::from_millis(500), &mut positions, &mut replies, &mut vehicle_link);
        accepted |= replies.iter().any(|(s, m)| *s == id && matches!(m, MavMessage::COMMAND_ACK(a) if a.command == MavCmd::MAV_CMD_DO_REPOSITION && a.result == MavResult::MAV_RESULT_ACCEPTED));
        let (x, y) = to_local(positions[&id].0, positions[&id].1);
        arrived |= (x - goal.0).hypot(y - goal.1) < 1.5;
    }
    check(accepted, &format!("go-to acknowledged as ACCEPTED (after {:.1}s)", started.elapsed().as_secs_f32()));
    check(arrived, &format!("vehicle {id} shows up at the goal on the map"));
    println!("all checks passed");
}
