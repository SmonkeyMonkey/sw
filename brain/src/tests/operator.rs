//! The operator's orders, going home, and the battery.

use super::*;

pub(super) fn plain_command(target_system: u8, command: MavCmd, params: [f32; 7]) -> COMMAND_LONG_DATA {
    COMMAND_LONG_DATA {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
        command,
        target_system,
        target_component: 0,
        confirmation: 0,
    }
}

/// The test mission's operator key (see `fleet_keys`).
pub(super) fn operator_secret() -> [u8; 32] {
    auth::Authority::from_secret([0xA5; 32]).operator_secret(TEST_MISSION)
}

std::thread_local! {
    /// When the next `gcs_command` is given, on the mission clock: each a millisecond after
    /// the last, all within `ORDER_MAX_AGE_MS` of the tests' clocks.
    static ISSUED: core::cell::Cell<u32> = const { core::cell::Cell::new(1_000) };
}

/// An order from the ground station, as it sends them: signed with the operator key.
pub(super) fn gcs_command(target_system: u8, command: MavCmd, params: [f32; 7]) -> (MavHeader, MavMessage) {
    let issued = ISSUED.with(|i| i.replace(i.get() + 1));
    gcs_command_at(issued, target_system, command, params)
}

pub(super) fn gcs_command_at(issued_ms: u32, target_system: u8, command: MavCmd, params: [f32; 7]) -> (MavHeader, MavMessage) {
    let header = MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 };
    let signed = auth::Operator::new(TEST_MISSION, operator_secret()).sign_command(&plain_command(target_system, command, params), issued_ms);
    (header, MavMessage::SWARM_SIGNED_COMMAND(signed))
}

#[test]
fn an_operator_order_overrides_the_target_and_is_acknowledged() {
    let mut brain = test_brain(1, cfg(), 1);
    let target = Vec2::new(5.0, 0.0);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[sighting(2, target, 0), sighting(3, target, 0)]);
    assert_eq!(brain.mode(), Mode::Converging);

    let go = gcs_command(1, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, -6.0, 0.0, 5.0]);
    let out = brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[go, sighting(2, target, 0), sighting(3, target, 0)]);
    assert_eq!(brain.mode(), Mode::Operator);
    assert_eq!(brain.order(), Some(Order::GoTo(Vec2::new(-6.0, 0.0))));
    assert!(out.velocity.x < -1.9, "flies the order, away from the target: {:?}", out.velocity);
    assert_eq!(brain.assigned_target(), None);

    let sent: std::vec::Vec<MavMessage> = brain.outgoing(1_050).into_iter().flatten().collect();
    assert!(sent.iter().any(|m| matches!(m, MavMessage::HEARTBEAT(hb) if hb.custom_mode & 0xFFFF == ANNOUNCE_OPERATOR)), "announces it's busy");
    assert!(sent.iter().any(|m| matches!(m, MavMessage::COMMAND_ACK(a) if a.command == MavCmd::MAV_CMD_DO_REPOSITION && a.result == MavResult::MAV_RESULT_ACCEPTED)));
    let st = sent.iter().find_map(|m| match m { MavMessage::SWARM_DRONE_STATUS(st) => Some(st.clone()), _ => None }).expect("status sent right away");
    assert_eq!((st.ack_command, st.order, st.mode), (MavCmd::MAV_CMD_DO_REPOSITION as u16, 1, 3));

    // Back to autonomy: it picks the target up again.
    let resume = gcs_command(1, MavCmd::MAV_CMD_MISSION_START, [0.0; 7]);
    brain.on_tick(&reading(1_100, Vec2::ZERO, None), &[resume, sighting(2, target, 0), sighting(3, target, 0)]);
    assert_eq!(brain.order(), None);
    assert_eq!(brain.mode(), Mode::Converging);
}

#[test]
fn return_to_launch_flies_home_and_lands() {
    let mut brain = test_brain(1, cfg(), 1);
    let rtl = gcs_command(1, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]);
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[rtl]);
    assert!(out.velocity.x < 0.0 && out.velocity.y < 0.0, "heads for home (-13,-13)");
    assert!(!out.land);
    let out = brain.on_tick(&reading(20_000, Vec2::new(-12.8, -13.1), None), &[]);
    assert!(out.land);
    assert_eq!(out.velocity, Vec2::ZERO);
}

/// A drone at work with its battery going down a tenth every minute: turns for home, by its
/// own reckoning, and lands with charge to spare, not much more than it needed. Returns (the
/// charge it turned home with, the charge it landed with), or None if it never landed.
pub(super) fn flies_home_to_charge(start: Vec2, drain_per_s: f32) -> Option<(f32, f32)> {
    let mut brain = test_brain(1, cfg(), 1);
    let (mut pos, mut battery, mut turned) = (start, 1.0f32, None);
    for tick in 0..20 * 1200 {
        let r = SensorReading { battery, ..reading(1_000 + tick * 50, pos, None) };
        let out = brain.on_tick(&r, &[]);
        if brain.recharging() && turned.is_none() {
            turned = Some(battery);
        }
        if out.land {
            return Some((turned?, battery));
        }
        pos = pos + out.velocity * 0.05;
        battery -= drain_per_s * 0.05;
    }
    None
}

#[test]
fn a_drone_low_on_charge_flies_home_in_time() {
    for (start, drain) in [(Vec2::new(13.0, 13.0), 1.0 / 600.0), (Vec2::new(13.0, -13.0), 1.0 / 300.0), (Vec2::ZERO, 1.0 / 1200.0)] {
        let (turned, landed) = flies_home_to_charge(start, drain).expect("never landed");
        std::println!("from {start:?}, a full charge a {:.0} s: turned home at {turned:.2}, landed at {landed:.2}", 1.0 / drain);
        assert!(landed >= 0.5 * BATTERY_RESERVE, "landed at {landed:.2}");
        // Drone 1 keeps a twentieth more than it needs (see `Brain::stagger`).
        assert!(turned <= 0.4, "turned home at {turned:.2}: too early");
    }
}

/// The board flies as a drone whatever airframe takes off as it: landed to charge, its brain
/// carries on, and is back to the swarm once it's on a charged one.
#[test]
fn landed_to_charge_and_charged_again_it_flies_with_the_swarm() {
    let mut brain = test_brain(1, cfg(), 1);
    let home = cfg().home;
    let low = |t: u32, at: Vec2| SensorReading { battery: 0.12, ..reading(t, at, None) };
    brain.on_tick(&low(1_000, Vec2::new(5.0, 5.0)), &[]);
    assert!(brain.recharging(), "home at 12% with no rate yet");
    let out = brain.on_tick(&low(1_050, home), &[]);
    assert!(out.land);
    let out = brain.on_tick(&SensorReading { battery: 0.97, ..reading(1_100, home, None) }, &[]);
    assert!(!out.land && !brain.recharging() && brain.order().is_none(), "{:?}", brain.order());
}

#[test]
fn a_drone_that_doesnt_know_its_charge_never_goes_home_by_itself() {
    let mut brain = test_brain(1, cfg(), 1);
    for tick in 0..2_000 {
        brain.on_tick(&reading(1_000 + tick * 50, Vec2::new(10.0, 10.0), None), &[]);
    }
    assert!(!brain.recharging() && brain.order().is_none());
}

#[test]
fn a_cancelled_target_stays_cancelled_when_seen_again() {
    let mut brain = test_brain(1, cfg(), 1);
    let target = Vec2::new(1.5, 0.0);
    brain.on_tick(&reading(1_000, Vec2::ZERO, Some(target)), &[]);
    assert_eq!(brain.mode(), Mode::Converging);
    let id = brain.assigned_target().unwrap();

    let cancel = gcs_command(0, MavCmd::MAV_CMD_SWARM_CANCEL_TARGET, [id as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    brain.on_tick(&reading(1_050, Vec2::ZERO, Some(target)), &[cancel]);
    assert_eq!(brain.mode(), Mode::Searching, "still sees it, but leaves it alone");
    // Relays the cancel to others, and its sightings now carry the flag.
    let sent: std::vec::Vec<MavMessage> = brain.outgoing(1_050).into_iter().flatten().collect();
    assert!(sent.iter().any(|m| matches!(m, MavMessage::SWARM_SIGNED_COMMAND(c) if c.command == MavCmd::MAV_CMD_SWARM_CANCEL_TARGET)));
    assert!(sent.iter().any(|m| matches!(m, MavMessage::SWARM_TARGET_SIGHTING(s) if s.cancelled == 1)));

    // Out of sight for 10 s (longer than an ordinary target is remembered), then back.
    brain.on_tick(&reading(11_000, Vec2::new(-8.0, 0.0), None), &[]);
    brain.on_tick(&reading(11_050, Vec2::new(0.0, 0.0), Some(target)), &[]);
    assert_eq!(brain.mode(), Mode::Searching);
}

#[test]
fn only_the_operators_command_cancels_a_target() {
    let mut brain = test_brain(1, cfg(), 1);
    let target = Vec2::new(4.0, 0.0);
    // Another drone saying it's cancelled doesn't make it so: any one drone could otherwise
    // call off any target.
    let mut cancelled = sighting(2, target, 0);
    if let MavMessage::SWARM_TARGET_SIGHTING(s) = &mut cancelled.1 {
        s.cancelled = 1;
    }
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[cancelled, sighting(3, target, 0)]);
    assert_eq!(brain.mode(), Mode::Converging);
    // The ground station's command does.
    let id = brain.assigned_target().expect("on it");
    let cancel = gcs_command(0, MavCmd::MAV_CMD_SWARM_CANCEL_TARGET, [id as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[cancel, sighting(2, target, 0), sighting(3, target, 0)]);
    brain.on_tick(&reading(1_100, Vec2::ZERO, None), &[sighting(2, target, 0), sighting(3, target, 0)]);
    assert_eq!(brain.mode(), Mode::Searching);
}

#[test]
fn a_flooded_command_is_relayed_once() {
    let mut brain = test_brain(1, cfg(), 1);
    let for_drone_9 = gcs_command(9, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), core::slice::from_ref(&for_drone_9));
    let relayed = |b: &mut Brain, now| b.outgoing(now).into_iter().flatten().filter(|m| matches!(m, MavMessage::SWARM_SIGNED_COMMAND(_))).count();
    assert_eq!(relayed(&mut brain, 1_000), 1);
    assert_eq!(brain.order(), None, "not for us");
    // The same frame coming back from a neighbor isn't relayed again.
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[for_drone_9]);
    assert_eq!(relayed(&mut brain, 1_050), 0);
}
