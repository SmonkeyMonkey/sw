//! Flying a drone by hand.

use super::*;

/// Stick input, the `seq`th, given at 1 s + 50 ms a frame on the mission clock (a relay
/// chain's tick `seq`).
pub(super) fn sticks(target: u8, forward: i16, right: i16, seq: u16) -> (MavHeader, MavMessage) {
    let header = MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 };
    let plain = mavlink_swarm::MANUAL_CONTROL_DATA { x: forward, y: right, z: 500, r: 0, buttons: seq, target };
    let signed = auth::Operator::new(TEST_MISSION, operator_secret()).sign_sticks(&plain, 1_000 + 50 * seq as u32);
    (header, MavMessage::SWARM_SIGNED_STICKS(signed))
}

#[test]
fn flies_where_the_sticks_push_and_hovers_when_they_go_quiet() {
    let mut brain = test_brain(1, cfg(), 1);
    // Full forward (north) and half right (east).
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[gcs_heartbeat(), sticks(1, 1000, 500, 1)]);
    assert_eq!(brain.mode(), Mode::Operator);
    assert!(matches!(brain.order(), Some(Order::Manual { .. })));
    assert!(out.velocity.y > 1.7 && out.velocity.x > 0.8, "{:?}", out.velocity);
    // Busy to the rest of the swarm.
    let busy = brain.outgoing(1_000).into_iter().flatten().any(|m| matches!(m, MavMessage::HEARTBEAT(hb) if hb.custom_mode & 0xFFFF == ANNOUNCE_OPERATOR));
    assert!(busy);
    // Sticks silent for over a second (the ground station still heard): hover, and say so.
    let out = brain.on_tick(&reading(2_200, Vec2::new(0.0, 2.0), None), &[]);
    assert_eq!(out.velocity, Vec2::ZERO);
    let st = brain.outgoing(2_200).into_iter().flatten().find_map(|m| match m { MavMessage::SWARM_DRONE_STATUS(st) => Some(st), _ => None });
    assert_eq!(st.map(|s| s.order), Some(5));
    // Sticks back: flying again.
    let out = brain.on_tick(&reading(2_250, Vec2::new(0.0, 2.0), None), &[sticks(1, -1000, 0, 2)]);
    assert!(out.velocity.y < -1.9);
}

#[test]
fn sticks_lost_for_good_it_rejoins_the_swarm() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[gcs_heartbeat(), sticks(1, 1000, 0, 1)]);
    // A few seconds without sticks: still hovering, waiting for them.
    let out = brain.on_tick(&reading(1_000 + STICKS_GIVE_UP_MS - 100, Vec2::ZERO, None), &[gcs_heartbeat()]);
    assert_eq!(out.velocity, Vec2::ZERO);
    assert!(brain.sticks_lost(1_000 + STICKS_GIVE_UP_MS - 100));
    // Then it gives up on them and says so at once.
    brain.on_tick(&reading(1_000 + STICKS_GIVE_UP_MS + 100, Vec2::ZERO, None), &[gcs_heartbeat()]);
    assert_eq!(brain.order(), None);
    assert_ne!(brain.mode(), Mode::Operator);
    let st = brain.outgoing(1_000 + STICKS_GIVE_UP_MS + 100).into_iter().flatten().find_map(|m| match m { MavMessage::SWARM_DRONE_STATUS(st) => Some(st), _ => None });
    assert_eq!(st.map(|s| s.order), Some(0));
}

#[test]
fn flown_out_of_contact_and_left_there_it_finds_its_way_back() {
    let mut brain = test_brain(1, cfg(), 1);
    let far = brain.config().home + Vec2::new(20.0, 20.0);
    brain.on_tick(&reading(1_000, far, None), &[gcs_heartbeat(), sticks(1, 1000, 0, 1)]);
    // Nothing heard since: no sticks, no ground station, no neighbors.
    let mut out = brain.on_tick(&reading(2_000, far, None), &[]);
    for t in (2_050..=1_000 + STICKS_GIVE_UP_MS + 200).step_by(50) {
        out = brain.on_tick(&reading(t, far, None), &[]);
    }
    assert_eq!(brain.order(), None);
    let home = brain.config().home - far;
    assert!(out.velocity.x * home.x + out.velocity.y * home.y > 0.9 * out.velocity.length() * home.length(), "{:?}", out.velocity);
}

#[test]
fn down_a_relay_chain_it_rejoins_the_swarm_too() {
    // Sticks for 1 s, then nothing for over `STICKS_GIVE_UP_MS`.
    let (brains, _) = relay_chain(5, 160, |tick| if tick < 20 && tick % 2 == 0 { std::vec![sticks(5, 1000, 0, tick as u16)] } else { std::vec![] });
    assert_eq!(brains[4].order(), None);
}

#[test]
fn a_steady_stick_reaches_a_drone_down_a_relay_chain() {
    // Drone 5 is four hops from the ground station; the operator holds the stick still.
    let (brains, _) = relay_chain(5, 60, |tick| if tick % 2 == 0 { std::vec![sticks(5, 1000, 0, tick as u16)] } else { std::vec![] });
    let Some(Order::Manual { at_ms, .. }) = brains[4].order() else { panic!("drone 5 isn't under manual control: {:?}", brains[4].order()) };
    // Its last stick input is recent, not just the first one from the start.
    assert!(1_000 + 59 * 50 - at_ms < STICK_TIMEOUT_MS, "last input at {at_ms}");
    assert!(brains[..4].iter().all(|b| b.order().is_none()));
}

#[test]
fn the_operator_sets_how_many_drones_respond() {
    let set = gcs_command(0, MavCmd::MAV_CMD_SWARM_SET_RESPONDERS, [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let target = Vec2::new(6.0, 0.0);
    let confirmed = [sighting(2, target, 0), sighting(3, target, 0)];
    // Two drones already on it, closer than us, and the limit is now 2: we stay out.
    let mut brain = test_brain(1, cfg(), 1);
    let mut inbox = std::vec![set.clone()];
    inbox.extend(confirmed.iter().cloned());
    for k in 0..2u8 {
        inbox.push(position(10 + k, target + Vec2::new(1.5 * k as f32, 1.5), Vec2::ZERO));
        inbox.push(heartbeat(10 + k, 0x0201));
    }
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.config().responders, 2);
    assert_eq!(brain.mode(), Mode::Searching);
    // And it passes the setting on.
    let relayed = brain.outgoing(1_000).into_iter().flatten().any(|m| matches!(m, MavMessage::SWARM_SIGNED_COMMAND(c) if c.command == MavCmd::MAV_CMD_SWARM_SET_RESPONDERS));
    assert!(relayed);
}

#[test]
fn old_stick_input_echoing_round_the_relays_doesnt_keep_a_drone_flying() {
    // The operator stops after 1 s; the relays keep bouncing the last frames between them
    // (each passes on anything it hears). Drone 5 must still notice the sticks are gone.
    let (brains, _) = relay_chain(5, 60, |tick| if tick < 20 && tick % 2 == 0 { std::vec![sticks(5, 1000, 0, tick as u16)] } else { std::vec![] });
    assert!(brains[4].sticks_lost(1_000 + 59 * 50), "still flying on echoes: {:?}", brains[4].order());
}
