//! Bridge to QGroundControl: shows the swarm in a standard ground station, the way a ground
//! radio plus a MAVLink router feeds QGC in the field.
//!
//! Our drones don't talk standard MAVLink telemetry to the ground (the swarm relays a compact
//! `SWARM_DRONE_STATUS` instead), and they navigate in local field meters. So the bridge
//! speaks for each drone the ground station knows about, under that drone's system ID:
//! `HEARTBEAT`, `GLOBAL_POSITION_INT` (field meters turned into latitude/longitude around
//! `FIELD_ORIGIN`), `GPS_RAW_INT` and `HOME_POSITION`. It also answers what QGC asks every new
//! vehicle for (parameters, mission, autopilot version), so QGC doesn't wait on them, and
//! passes QGC's commands for a drone on to the swarm, acknowledging back once the drone has.
//!
//! QGC only sees what the ground station sees: a drone out of contact stops updating there too.
//!
//! No I/O here: frames in, frames out. The link thread does the sending.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use mavlink_swarm::{
    GpsFixType, MavAutopilot, MavCmd, MavFrame, MavMessage, MavModeFlag, MavParamType, MavProtocolCapability, MavResult,
    MavState, MavType, AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA, GLOBAL_POSITION_INT_DATA,
    GPS_RAW_INT_DATA, HEARTBEAT_DATA, HOME_POSITION_DATA, MISSION_COUNT_DATA, PARAM_VALUE_DATA,
};

/// Where QGroundControl listens for vehicles by default.
pub const QGC_ADDR: &str = "127.0.0.1:14550";

/// Where the field's (0, 0) is on Earth: open farmland, picked only so QGC has somewhere to put
/// the map. Field x runs east, y north.
pub const FIELD_ORIGIN: (f64, f64) = (49.0, 32.0);
/// Ground level there (m above sea level) and the swarm's flying height above it.
const GROUND_AMSL_M: f64 = 100.0;
const CRUISE_ALTITUDE_M: f64 = 5.0;
/// The launch base, in field meters (the swarm's `BrainConfig::home`).
pub const BASE: (f32, f32) = (-13.0, -13.0);

/// How often each drone's telemetry is sent on to QGC.
const SEND_EVERY: Duration = Duration::from_millis(500);
/// Drones not heard from for this long aren't spoken for any more: QGC shows them as lost.
const STALE: Duration = Duration::from_secs(5);
/// Commands from QGC are repeated to the swarm this often until acknowledged, this many times.
const RETRY_EVERY: Duration = Duration::from_millis(1_200);
const MAX_ATTEMPTS: u8 = 6;

const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Field meters <-> latitude/longitude, on a flat patch of Earth around `FIELD_ORIGIN` (good to
/// centimeters over a 30 m field).
pub fn to_global(x: f32, y: f32) -> (f64, f64) {
    let (lat0, lon0) = FIELD_ORIGIN;
    let lat = lat0 + (y as f64 / EARTH_RADIUS_M).to_degrees();
    let lon = lon0 + (x as f64 / (EARTH_RADIUS_M * lat0.to_radians().cos())).to_degrees();
    (lat, lon)
}

pub fn to_local(lat: f64, lon: f64) -> (f32, f32) {
    let (lat0, lon0) = FIELD_ORIGIN;
    let y = (lat - lat0).to_radians() * EARTH_RADIUS_M;
    let x = (lon - lon0).to_radians() * EARTH_RADIUS_M * lat0.to_radians().cos();
    (x as f32, y as f32)
}

/// A latitude/longitude frame. Older QGC versions still send the `*_INT` names, deprecated
/// since 2024 in favor of the plain ones but meaning the same.
#[allow(deprecated)]
fn is_global(frame: MavFrame) -> bool {
    matches!(
        frame,
        MavFrame::MAV_FRAME_GLOBAL | MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT | MavFrame::MAV_FRAME_GLOBAL_INT | MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    )
}

fn degrees_e7(d: f64) -> i32 {
    (d * 1e7).round() as i32
}

struct Drone {
    x: f32,
    y: f32,
    mode: u8,
    /// Where its position comes from (0 = GPS) and how sure it is, m.
    nav: u8,
    accuracy: f32,
    seen: Instant,
}

/// A QGC command passed on to the swarm, waiting for the drone's acknowledgement.
struct Forwarded {
    cmd: COMMAND_LONG_DATA,
    sent: Instant,
    attempts: u8,
}

/// A frame for QGC, sent as if from drone `from`.
pub struct ToQgc {
    pub from: u8,
    pub msg: MavMessage,
}

#[derive(Default)]
pub struct Out {
    pub to_qgc: Vec<ToQgc>,
    pub to_radio: Vec<MavMessage>,
}

pub struct Bridge {
    drones: HashMap<u8, Drone>,
    forwarded: Vec<Forwarded>,
    next_send: Instant,
    started: Instant,
}

impl Default for Bridge {
    fn default() -> Self {
        Self { drones: HashMap::new(), forwarded: Vec::new(), next_send: Instant::now(), started: Instant::now() }
    }
}

impl Bridge {
    /// Something the ground station heard over the radio.
    pub fn on_radio(&mut self, msg: &MavMessage, now: Instant, out: &mut Out) {
        let MavMessage::SWARM_DRONE_STATUS(st) = msg else { return };
        let seen = now - Duration::from_millis(st.age_ms as u64);
        // Relays can deliver the same status several times; keep the freshest.
        if self.drones.get(&st.sysid).is_some_and(|d| d.seen >= seen) {
            return;
        }
        self.drones.insert(st.sysid, Drone { x: st.x, y: st.y, mode: st.mode, nav: st.nav, accuracy: st.pos_accuracy_dm as f32 / 10.0, seen });
        // A command we passed on for QGC, now acknowledged by the drone: tell QGC.
        if let Some(k) = self
            .forwarded
            .iter()
            .position(|f| f.cmd.target_system == st.sysid && f.cmd.command as u16 == st.ack_command)
        {
            let f = self.forwarded.remove(k);
            let result = if st.ack_result == MavResult::MAV_RESULT_ACCEPTED as u8 { MavResult::MAV_RESULT_ACCEPTED } else { MavResult::MAV_RESULT_FAILED };
            out.to_qgc.push(ToQgc { from: st.sysid, msg: MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: f.cmd.command, result }) });
        }
    }

    /// Something QGC sent. Requests addressed to one of our drones are answered on its behalf.
    pub fn on_qgc(&mut self, msg: &MavMessage, now: Instant, out: &mut Out) {
        match msg {
            MavMessage::PARAM_REQUEST_LIST(r) if self.knows(r.target_system) => out.to_qgc.push(only_param(r.target_system)),
            MavMessage::PARAM_REQUEST_READ(r) if self.knows(r.target_system) => out.to_qgc.push(only_param(r.target_system)),
            // No missions: the swarm decides for itself where to go.
            MavMessage::MISSION_REQUEST_LIST(r) if self.knows(r.target_system) => out.to_qgc.push(ToQgc {
                from: r.target_system,
                msg: MavMessage::MISSION_COUNT(MISSION_COUNT_DATA { count: 0, target_system: 255, target_component: 0 }),
            }),
            MavMessage::COMMAND_LONG(c) if self.knows(c.target_system) => self.command(c.clone(), now, out),
            MavMessage::COMMAND_INT(c) if self.knows(c.target_system) => {
                // Same commands, with the position as latitude/longitude * 1e7.
                let (x, y) = to_local(c.x as f64 / 1e7, c.y as f64 / 1e7);
                let positioned = is_global(c.frame);
                let as_long = COMMAND_LONG_DATA {
                    param1: c.param1,
                    param2: c.param2,
                    param3: c.param3,
                    param4: c.param4,
                    param5: if positioned { x } else { 0.0 },
                    param6: if positioned { y } else { 0.0 },
                    param7: c.z,
                    command: c.command,
                    target_system: c.target_system,
                    target_component: c.target_component,
                    confirmation: 0,
                };
                self.command_local(as_long, now, out);
            }
            _ => {}
        }
    }

    fn knows(&self, sysid: u8) -> bool {
        self.drones.contains_key(&sysid)
    }

    /// A COMMAND_LONG from QGC, positions in degrees.
    fn command(&mut self, mut c: COMMAND_LONG_DATA, now: Instant, out: &mut Out) {
        if c.command == MavCmd::MAV_CMD_DO_REPOSITION {
            let (x, y) = to_local(c.param5 as f64, c.param6 as f64);
            (c.param5, c.param6) = (x, y);
        }
        self.command_local(c, now, out);
    }

    /// A command from QGC, positions already in field meters.
    fn command_local(&mut self, c: COMMAND_LONG_DATA, now: Instant, out: &mut Out) {
        match c.command {
            MavCmd::MAV_CMD_REQUEST_MESSAGE => {
                // QGC asks every new vehicle what it is; nothing else is on offer.
                let result = if c.param1 as u32 == 148 {
                    out.to_qgc.push(ToQgc { from: c.target_system, msg: autopilot_version(c.target_system) });
                    MavResult::MAV_RESULT_ACCEPTED
                } else {
                    MavResult::MAV_RESULT_UNSUPPORTED
                };
                out.to_qgc.push(ToQgc { from: c.target_system, msg: MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: c.command, result }) });
            }
            // What the swarm's drones understand: passed on, acknowledged once the drone has.
            MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH | MavCmd::MAV_CMD_NAV_LOITER_UNLIM | MavCmd::MAV_CMD_MISSION_START | MavCmd::MAV_CMD_DO_REPOSITION => {
                self.forwarded.retain(|f| f.cmd.target_system != c.target_system);
                let cmd = COMMAND_LONG_DATA { confirmation: 0, ..c };
                out.to_radio.push(MavMessage::COMMAND_LONG(cmd.clone()));
                self.forwarded.push(Forwarded { cmd, sent: now, attempts: 1 });
                out.to_qgc.push(ToQgc { from: c.target_system, msg: MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: c.command, result: MavResult::MAV_RESULT_IN_PROGRESS }) });
            }
            _ => out.to_qgc.push(ToQgc {
                from: c.target_system,
                msg: MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: c.command, result: MavResult::MAV_RESULT_UNSUPPORTED }),
            }),
        }
    }

    /// Called often: telemetry for QGC at `SEND_EVERY`, and repeats of unacknowledged commands.
    pub fn tick(&mut self, now: Instant, out: &mut Out) {
        for f in self.forwarded.iter_mut() {
            if now.duration_since(f.sent) >= RETRY_EVERY && f.attempts < MAX_ATTEMPTS {
                f.attempts += 1;
                f.sent = now;
                f.cmd.confirmation = f.cmd.confirmation.wrapping_add(1);
                out.to_radio.push(MavMessage::COMMAND_LONG(f.cmd.clone()));
            }
        }
        let (failed, waiting): (Vec<_>, Vec<_>) = std::mem::take(&mut self.forwarded)
            .into_iter()
            .partition(|f| f.attempts >= MAX_ATTEMPTS && now.duration_since(f.sent) >= RETRY_EVERY);
        self.forwarded = waiting;
        for f in failed {
            out.to_qgc.push(ToQgc { from: f.cmd.target_system, msg: MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command: f.cmd.command, result: MavResult::MAV_RESULT_FAILED }) });
        }

        if now < self.next_send {
            return;
        }
        self.next_send = now + SEND_EVERY;
        let time_boot_ms = now.duration_since(self.started).as_millis() as u32;
        let (home_lat, home_lon) = to_global(BASE.0, BASE.1);
        for (&sysid, d) in &self.drones {
            if now.duration_since(d.seen) > STALE {
                continue;
            }
            let (lat, lon) = to_global(d.x, d.y);
            let msgs = [
                MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                    custom_mode: d.mode as u32,
                    mavtype: MavType::MAV_TYPE_QUADROTOR,
                    autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
                    base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    system_status: MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                }),
                MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
                    time_boot_ms,
                    lat: degrees_e7(lat),
                    lon: degrees_e7(lon),
                    alt: ((GROUND_AMSL_M + CRUISE_ALTITUDE_M) * 1000.0) as i32,
                    relative_alt: (CRUISE_ALTITUDE_M * 1000.0) as i32,
                    vx: 0,
                    vy: 0,
                    vz: 0,
                    // Unknown: the status doesn't carry a heading.
                    hdg: u16::MAX,
                }),
                MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                    time_usec: time_boot_ms as u64 * 1000,
                    lat: degrees_e7(lat),
                    lon: degrees_e7(lon),
                    alt: ((GROUND_AMSL_M + CRUISE_ALTITUDE_M) * 1000.0) as i32,
                    // The drone's own accuracy estimate, in cm; no fix when it isn't on GPS
                    // (jammed, or spoofed and ignoring it).
                    eph: (d.accuracy * 100.0).min(u16::MAX as f32 - 1.0) as u16,
                    epv: u16::MAX,
                    vel: u16::MAX,
                    cog: u16::MAX,
                    fix_type: if d.nav == 0 { GpsFixType::GPS_FIX_TYPE_3D_FIX } else { GpsFixType::GPS_FIX_TYPE_NO_FIX },
                    satellites_visible: if d.nav == 0 { 12 } else { 0 },
                }),
                MavMessage::HOME_POSITION(HOME_POSITION_DATA {
                    latitude: degrees_e7(home_lat),
                    longitude: degrees_e7(home_lon),
                    altitude: (GROUND_AMSL_M * 1000.0) as i32,
                    x: BASE.1,
                    y: BASE.0,
                    z: 0.0,
                    q: [1.0, 0.0, 0.0, 0.0],
                    approach_x: 0.0,
                    approach_y: 0.0,
                    approach_z: 0.0,
                }),
            ];
            for msg in msgs {
                out.to_qgc.push(ToQgc { from: sysid, msg });
            }
        }
    }
}

/// QGC insists on downloading a vehicle's parameters; ours have just one, its system ID.
fn only_param(sysid: u8) -> ToQgc {
    ToQgc {
        from: sysid,
        msg: MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value: sysid as f32,
            param_count: 1,
            param_index: 0,
            param_id: "SYSID_THISMAV".into(),
            param_type: MavParamType::MAV_PARAM_TYPE_INT32,
        }),
    }
}

fn autopilot_version(sysid: u8) -> MavMessage {
    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
        uid: sysid as u64,
        flight_sw_version: 0,
        middleware_sw_version: 0,
        os_sw_version: 0,
        board_version: 0,
        vendor_id: 0,
        product_id: 0,
        flight_custom_version: [0; 8],
        middleware_custom_version: [0; 8],
        os_custom_version: [0; 8],
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink_swarm::{COMMAND_INT_DATA, PARAM_REQUEST_LIST_DATA, SWARM_DRONE_STATUS_DATA};

    fn status(sysid: u8, x: f32, y: f32, ack: (u16, u8)) -> MavMessage {
        MavMessage::SWARM_DRONE_STATUS(SWARM_DRONE_STATUS_DATA {
            age_ms: 0,
            x,
            y,
            target_id: 0,
            ack_command: ack.0,
            sysid,
            seq: 1,
            hops: 2,
            mode: 0,
            order: 0,
            ack_result: ack.1,
            nav: 0,
            pos_accuracy_dm: 20,
            fleet: 0,
        })
    }

    #[test]
    fn field_meters_round_trip_through_latitude_and_longitude() {
        for (x, y) in [(0.0, 0.0), (-13.0, -13.0), (15.0, 15.0), (7.5, -3.25)] {
            let (lat, lon) = to_global(x, y);
            let (bx, by) = to_local(lat, lon);
            assert!((bx - x).abs() < 0.01 && (by - y).abs() < 0.01, "({x}, {y}) came back as ({bx}, {by})");
        }
        // North is +y: a point 15 m north is north of the origin.
        assert!(to_global(0.0, 15.0).0 > FIELD_ORIGIN.0);
    }

    #[test]
    fn speaks_for_each_drone_the_ground_station_hears() {
        let mut bridge = Bridge::default();
        let mut out = Out::default();
        let now = Instant::now();
        bridge.on_radio(&status(3, 10.0, 0.0, (0, 0)), now, &mut out);
        bridge.on_radio(&status(7, -5.0, 5.0, (0, 0)), now, &mut out);
        bridge.tick(now, &mut out);
        let from: std::collections::BTreeSet<u8> = out.to_qgc.iter().map(|f| f.from).collect();
        assert_eq!(from, [3, 7].into());
        let pos = out.to_qgc.iter().find_map(|f| match &f.msg {
            MavMessage::GLOBAL_POSITION_INT(p) if f.from == 3 => Some(p.clone()),
            _ => None,
        });
        let pos = pos.expect("position for drone 3");
        let (x, y) = to_local(pos.lat as f64 / 1e7, pos.lon as f64 / 1e7);
        assert!((x - 10.0).abs() < 0.05 && y.abs() < 0.05);
        // A drone not heard from for a while is no longer spoken for.
        let mut later = Out::default();
        bridge.tick(now + Duration::from_secs(10), &mut later);
        assert!(later.to_qgc.is_empty());
    }

    #[test]
    fn answers_what_qgc_asks_a_new_vehicle() {
        let mut bridge = Bridge::default();
        let mut out = Out::default();
        let now = Instant::now();
        bridge.on_radio(&status(3, 0.0, 0.0, (0, 0)), now, &mut out);
        bridge.on_qgc(&MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA { target_system: 3, target_component: 1 }), now, &mut out);
        assert!(out.to_qgc.iter().any(|f| f.from == 3 && matches!(&f.msg, MavMessage::PARAM_VALUE(p) if p.param_count == 1)));
        // And ignores requests for drones it doesn't know.
        let mut other = Out::default();
        bridge.on_qgc(&MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA { target_system: 9, target_component: 1 }), now, &mut other);
        assert!(other.to_qgc.is_empty());
    }

    #[test]
    fn a_go_to_from_qgc_reaches_the_swarm_in_field_meters_and_is_acknowledged() {
        let mut bridge = Bridge::default();
        let mut out = Out::default();
        let now = Instant::now();
        bridge.on_radio(&status(3, 0.0, 0.0, (0, 0)), now, &mut out);
        let (lat, lon) = to_global(-5.0, 8.0);
        let go = MavMessage::COMMAND_INT(COMMAND_INT_DATA {
            param1: -1.0,
            param2: 0.0,
            param3: 0.0,
            param4: f32::NAN,
            x: degrees_e7(lat),
            y: degrees_e7(lon),
            z: 5.0,
            command: MavCmd::MAV_CMD_DO_REPOSITION,
            target_system: 3,
            target_component: 1,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current: 0,
            autocontinue: 0,
        });
        bridge.on_qgc(&go, now, &mut out);
        let sent = out.to_radio.iter().find_map(|m| match m { MavMessage::COMMAND_LONG(c) => Some(c.clone()), _ => None }).expect("passed on");
        assert_eq!((sent.target_system, sent.command), (3, MavCmd::MAV_CMD_DO_REPOSITION));
        assert!((sent.param5 + 5.0).abs() < 0.05 && (sent.param6 - 8.0).abs() < 0.05, "({}, {})", sent.param5, sent.param6);
        // The drone's status acknowledges it: QGC gets an ACCEPTED.
        let mut acked = Out::default();
        bridge.on_radio(&status(3, 0.0, 0.0, (MavCmd::MAV_CMD_DO_REPOSITION as u16, 0)), now + Duration::from_millis(300), &mut acked);
        assert!(acked.to_qgc.iter().any(|f| f.from == 3 && matches!(&f.msg, MavMessage::COMMAND_ACK(a) if a.result == MavResult::MAV_RESULT_ACCEPTED)));
    }
}
