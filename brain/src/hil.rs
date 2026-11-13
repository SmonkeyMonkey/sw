//! Encoding of the hardware-in-the-loop link (SWARM_HIL_* messages, see mavlink-swarm's
//! dialect/swarm.xml) to and from brain types. Shared by the firmware, the host-side fake board,
//! and the backend's bridge, so every side agrees on the wire format.

use mavlink_swarm::{
    MavMessage, SWARM_HIL_COMMAND_DATA, SWARM_HIL_MISSION_DATA, SWARM_HIL_SENSOR_DATA, SWARM_HIL_TRACE_EVENT_DATA,
};

use crate::{
    Brain, BrainConfig, Keys, SearchStrategy, SensorReading, SwarmMode, TickOutput, TraceEvent, Vec2, MAX_NEARBY, MAX_SEEN,
};

/// A message of the board's link to its body (the simulator): sensor readings, commands,
/// trace context, its keys. These never travel by radio, and one that comes in from the radio
/// is dropped, or anyone on the air could feed a board made-up sensor readings (a position, a
/// system ID, a clock gone backwards that starts it over with a fresh brain) or keys of their
/// own making. And the board's own keys never go out over the air.
pub fn is_body_link(msg: &MavMessage) -> bool {
    matches!(
        msg,
        MavMessage::SWARM_HIL_SENSOR(_)
            | MavMessage::SWARM_HIL_COMMAND(_)
            | MavMessage::SWARM_HIL_TRACE_EVENT(_)
            | MavMessage::SWARM_HIL_TRACE_SPAN(_)
            | MavMessage::SWARM_HIL_MISSION(_)
    )
}

const KIND_SPOTTED: u8 = 1;
const KIND_ADOPTED: u8 = 2;

/// What the launch base tells a drone before it takes off (SWARM_HIL_MISSION): its keys, and
/// what in its configuration is the mission's rather than the swarm's. The rest is
/// `BrainConfig::SWARM`, the same as the simulator's drones.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Mission {
    pub keys: Keys,
    pub field_half: Vec2,
    pub home: Vec2,
    pub mode: SwarmMode,
    pub search: SearchStrategy,
    pub fleet: u16,
}

impl Mission {
    pub fn new(keys: Keys, cfg: &BrainConfig) -> Self {
        Self { keys, field_half: cfg.field_half, home: cfg.home, mode: cfg.mode(), search: cfg.search, fleet: cfg.fleet }
    }

    pub fn config(&self) -> BrainConfig {
        BrainConfig { field_half: self.field_half, home: self.home, search: self.search, fleet: self.fleet, ..BrainConfig::SWARM }.with_mode(self.mode)
    }
}

pub fn mission_msg(m: &Mission) -> MavMessage {
    MavMessage::SWARM_HIL_MISSION(SWARM_HIL_MISSION_DATA {
        mission: m.keys.mission,
        sysid: m.keys.sysid,
        secret_key: m.keys.secret,
        certificate: m.keys.certificate,
        authority_key: m.keys.authority,
        operator_key: m.keys.operator,
        link_key: m.keys.link,
        field_half_x: m.field_half.x,
        field_half_y: m.field_half.y,
        home_x: m.home.x,
        home_y: m.home.y,
        swarm_mode: m.mode.to_u8(),
        search: match m.search {
            SearchStrategy::Coverage => 0,
            SearchStrategy::RandomWalk => 1,
        },
        fleet: m.fleet,
    })
}

/// None if the numbers in it aren't (a field or base at NaN would fly the drone off anywhere).
pub fn mission(m: &SWARM_HIL_MISSION_DATA) -> Option<Mission> {
    let (field_half, home) = (Vec2::new(m.field_half_x, m.field_half_y), Vec2::new(m.home_x, m.home_y));
    (field_half.is_finite() && home.is_finite() && field_half.x > 0.0 && field_half.y > 0.0).then_some(Mission {
        keys: Keys { mission: m.mission, sysid: m.sysid, secret: m.secret_key, certificate: m.certificate, authority: m.authority_key, operator: m.operator_key, link: m.link_key },
        field_half,
        home,
        mode: SwarmMode::from_u8(m.swarm_mode),
        search: if m.search == 1 { SearchStrategy::RandomWalk } else { SearchStrategy::Coverage },
        fleet: m.fleet,
    })
}

pub fn sensor_msg(reading: &SensorReading, sysid: u8) -> MavMessage {
    let (mut nearby_x, mut nearby_y, mut nearby_vx, mut nearby_vy) = ([0.0; MAX_NEARBY], [0.0; MAX_NEARBY], [0.0; MAX_NEARBY], [0.0; MAX_NEARBY]);
    let mut nearby_sysid = [0u8; MAX_NEARBY];
    let mut nearby_count = 0;
    for (i, (id, p, v)) in reading.nearby.iter().flatten().flatten().enumerate() {
        (nearby_sysid[i], nearby_x[i], nearby_y[i], nearby_vx[i], nearby_vy[i]) = (*id, p.x, p.y, v.x, v.y);
        nearby_count += 1;
    }
    let mut target_x = [0.0; MAX_SEEN];
    let mut target_y = [0.0; MAX_SEEN];
    let mut target_count = 0;
    for (i, pos) in reading.targets().enumerate() {
        target_x[i] = pos.x;
        target_y[i] = pos.y;
        target_count += 1;
    }
    MavMessage::SWARM_HIL_SENSOR(SWARM_HIL_SENSOR_DATA {
        time_ms: reading.time_ms,
        x: reading.own_pos.x,
        y: reading.own_pos.y,
        vx: reading.own_velocity.x,
        vy: reading.own_velocity.y,
        target_x,
        target_y,
        target_count,
        nearby_sysid,
        gps_fix: reading.gps_fix as u8,
        nearby_x,
        nearby_y,
        nearby_vx,
        nearby_vy,
        nearby_count,
        has_proximity: reading.nearby.is_some() as u8,
        battery: reading.battery,
        sysid,
    })
}

pub fn sensor_reading(sensor: &SWARM_HIL_SENSOR_DATA) -> SensorReading {
    SensorReading {
        time_ms: sensor.time_ms,
        own_pos: Vec2::new(sensor.x, sensor.y),
        gps_fix: sensor.gps_fix != 0,
        own_velocity: Vec2::new(sensor.vx, sensor.vy),
        targets_seen: core::array::from_fn(|i| {
            (i < sensor.target_count as usize).then(|| Vec2::new(sensor.target_x[i], sensor.target_y[i]))
        }),
        nearby: (sensor.has_proximity != 0).then(|| {
            core::array::from_fn(|i| {
                (i < sensor.nearby_count as usize)
                    .then(|| (sensor.nearby_sysid[i], Vec2::new(sensor.nearby_x[i], sensor.nearby_y[i]), Vec2::new(sensor.nearby_vx[i], sensor.nearby_vy[i])))
            })
        }),
        battery: sensor.battery,
    }
}

pub fn trace_event_msg(event: TraceEvent) -> MavMessage {
    let data = match event {
        TraceEvent::Spotted => SWARM_HIL_TRACE_EVENT_DATA {
            age_ms: 0,
            kind: KIND_SPOTTED,
            observer_sysid: 0,
            hops: 0,
            trace_id: [0; 16],
            parent_span_id: [0; 8],
        },
        TraceEvent::Adopted { trace_id, parent_span_id, observer_sysid, hops, age_ms } => SWARM_HIL_TRACE_EVENT_DATA {
            age_ms,
            kind: KIND_ADOPTED,
            observer_sysid,
            hops,
            trace_id,
            parent_span_id,
        },
    };
    MavMessage::SWARM_HIL_TRACE_EVENT(data)
}

pub fn trace_event(e: &SWARM_HIL_TRACE_EVENT_DATA) -> Option<TraceEvent> {
    match e.kind {
        KIND_SPOTTED => Some(TraceEvent::Spotted),
        KIND_ADOPTED => Some(TraceEvent::Adopted {
            trace_id: e.trace_id,
            parent_span_id: e.parent_span_id,
            observer_sysid: e.observer_sysid,
            hops: e.hops,
            age_ms: e.age_ms,
        }),
        _ => None,
    }
}

/// The command that ends the tick started by `reading`.
pub fn command_msg(brain: &Brain, reading: &SensorReading, out: &TickOutput) -> MavMessage {
    let mut lost_sysid = [0u8; 4];
    let mut lost_silent_ms = [0u32; 4];
    for (i, lost) in out.lost.iter().enumerate() {
        if let Some(lost) = lost {
            lost_sysid[i] = lost.sysid;
            lost_silent_ms[i] = lost.silent_ms;
        }
    }
    MavMessage::SWARM_HIL_COMMAND(SWARM_HIL_COMMAND_DATA {
        time_ms: reading.time_ms,
        vx: out.velocity.x,
        vy: out.velocity.y,
        mode: brain.mode().to_u8(),
        target_id: brain.assigned_target().unwrap_or(0),
        informed: brain.informed() as u8,
        isolated: brain.isolated(reading.time_ms) as u8,
        refuted_sysid: out.refuted.unwrap_or(0),
        land: if out.to_charge { 2 } else { out.land as u8 },
        est_x: out.position.x,
        est_y: out.position.y,
        nav: out.nav.to_u8(),
        lost_sysid,
        lost_silent_ms,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::auth::Authority;

    #[test]
    fn the_board_gets_the_same_brain_as_the_simulated_drones() {
        // As the backend sets its drones up for a 50 x 30 m field, connected, random walk.
        let field_half = Vec2::new(25.0, 15.0);
        let cfg = BrainConfig { search: SearchStrategy::RandomWalk, field_half, home: Vec2::new(-23.0, -13.0), fleet: 8, ..BrainConfig::SWARM }
            .with_mode(SwarmMode::Connected);
        let keys = Authority::from_secret([1; 32]).issue(3, 1, [2; 32]);
        let MavMessage::SWARM_HIL_MISSION(msg) = mission_msg(&Mission::new(keys, &cfg)) else { panic!() };
        let got = mission(&msg).expect("a good mission");
        assert_eq!(got, Mission::new(keys, &cfg));
        let board = got.config();
        assert_eq!((board.field_half, board.home, board.mode(), board.search), (cfg.field_half, cfg.home, cfg.mode(), cfg.search));
        assert_eq!((board.relay_chains, board.keep_link, board.responders, board.fleet), (cfg.relay_chains, cfg.keep_link, cfg.responders, cfg.fleet));

        let bad = SWARM_HIL_MISSION_DATA { field_half_x: f32::NAN, ..msg };
        assert!(mission(&bad).is_none());
    }
}
