//! Orders that can't be forged, replayed or flooded, and relaying without repeats.

use super::*;

fn acked(brain: &mut Brain, now: u32, command: MavCmd) -> Option<MavResult> {
    brain.outgoing(now).into_iter().flatten().find_map(|m| match m {
        MavMessage::COMMAND_ACK(a) if a.command == command => Some(a.result),
        _ => None,
    })
}

#[test]
fn a_nan_in_a_command_is_refused() {
    let mut brain = test_brain(1, cfg(), 1);
    let go = gcs_command(1, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, f32::NAN, 3.0, 5.0]);
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[go]);
    assert_eq!(brain.order(), None);
    assert_eq!(acked(&mut brain, 1_000, MavCmd::MAV_CMD_DO_REPOSITION), Some(MavResult::MAV_RESULT_DENIED));
    assert!(out.velocity.is_finite());

    let mode = gcs_command(1, MavCmd::MAV_CMD_SWARM_SET_MODE, [f32::INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[mode]);
    assert_eq!(acked(&mut brain, 1_050, MavCmd::MAV_CMD_SWARM_SET_MODE), Some(MavResult::MAV_RESULT_DENIED));
}

#[test]
fn a_nan_in_a_sighting_or_a_sensor_is_ignored() {
    let mut brain = test_brain(1, cfg(), 1);
    let bad = [
        sighting(2, Vec2::new(f32::NAN, 1.0), 0),
        sighting(3, Vec2::new(2.0, f32::INFINITY), 0),
        position(4, Vec2::new(f32::NAN, 0.0), Vec2::ZERO),
    ];
    let mut r = reading(1_000, Vec2::new(1.0, 1.0), None);
    r.own_velocity = Vec2::new(f32::INFINITY, 0.0);
    r.targets_seen[0] = Some(Vec2::new(f32::NAN, 0.0));
    for t in 0..40 {
        let out = brain.on_tick(&SensorReading { time_ms: 1_000 + t * 50, ..r }, &bad);
        assert!(out.velocity.is_finite() && out.position.is_finite(), "{:?} at {:?}", out.velocity, out.position);
    }
    assert!(brain.tracks.iter().flatten().all(|t| t.pos.is_finite()));
    assert_eq!(brain.tracks.iter().flatten().count(), 0, "a target made of NaN");

    // Without a finite fix the drone carries on as if it had no GPS.
    r.own_pos = Vec2::new(f32::NAN, f32::NAN);
    let out = brain.on_tick(&SensorReading { time_ms: 3_000, ..r }, &[]);
    assert!(out.velocity.is_finite() && out.position.is_finite());
}

#[test]
fn a_go_to_off_the_field_goes_to_its_edge() {
    let mut brain = test_brain(1, cfg(), 1);
    let go = gcs_command(1, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, 1.0e30, -500.0, 5.0]);
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[go]);
    let half = cfg().field_half;
    let Some(Order::GoTo(to)) = brain.order() else { panic!("no go-to: {:?}", brain.order()) };
    assert!(to.x.abs() < half.x && to.y.abs() < half.y, "{to:?} is off a field of {half:?}");
    assert!(out.velocity.is_finite());
}

/// Run in a debug build (`cargo test`) this panics on any `+` that overflows: the clock is
/// a u32 of milliseconds, 49 days long.
#[test]
fn the_clock_running_out_doesnt_panic() {
    let mut brains: std::vec::Vec<Brain> = (1..=3).map(|id| Brain::new(id, cfg(), id as u32, test_keys(id))).collect();
    let mouse = Vec2::new(2.0, 2.0);
    let mut air: std::vec::Vec<(MavHeader, MavMessage)> = std::vec![gcs_heartbeat()];
    for now in (u32::MAX - 1_000..=u32::MAX).step_by(50) {
        let mut next = std::vec::Vec::new();
        for (i, b) in brains.iter_mut().enumerate() {
            let at = Vec2::new(i as f32, 0.0);
            let out = b.on_tick(&reading(now, at, Some(mouse)), &air);
            assert!(out.velocity.is_finite());
            let own = MavHeader { system_id: b.sysid(), component_id: COMPONENT_ID, sequence: 0 };
            next.extend(b.outgoing_frames(now).into_iter().flatten().map(|(relayed, m)| (relayed.unwrap_or(own), m)));
        }
        next.push(gcs_heartbeat());
        air = next;
    }
}

/// Whatever went out to the radio this tick that's an order, a stick input or its answer.
fn orders_sent(brain: &mut Brain, now: u32) -> usize {
    let sent = brain.outgoing(now).into_iter().flatten();
    sent.filter(|m| matches!(m, MavMessage::SWARM_SIGNED_COMMAND(_) | MavMessage::SWARM_SIGNED_STICKS(_) | MavMessage::COMMAND_LONG(_) | MavMessage::MANUAL_CONTROL(_) | MavMessage::COMMAND_ACK(_)))
        .count()
}

#[test]
fn an_unsigned_order_is_neither_carried_out_nor_relayed() {
    // Under the ground station's own header, as anyone on the air could send it.
    let mut brain = test_brain(1, cfg(), 1);
    let header = MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 };
    let rtl = (header, MavMessage::COMMAND_LONG(plain_command(1, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7])));
    let for_9 = (header, MavMessage::COMMAND_LONG(plain_command(9, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7])));
    let few = (header, MavMessage::COMMAND_LONG(plain_command(0, MavCmd::MAV_CMD_SWARM_SET_RESPONDERS, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])));
    let plain_sticks = (header, MavMessage::MANUAL_CONTROL(mavlink_swarm::MANUAL_CONTROL_DATA { x: 1000, y: 0, z: 500, r: 0, buttons: 1, target: 1 }));
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[rtl, for_9, few, plain_sticks]);
    assert_eq!(brain.order(), None);
    assert_eq!(brain.cfg.responders, cfg().responders);
    assert_eq!(orders_sent(&mut brain, 1_000), 0, "passed on or answered");
}

/// An order as the ground station's, but signed with drone 7's own key: what a captured
/// drone can do.
fn forged_order(target_system: u8, command: MavCmd, params: [f32; 7]) -> (MavHeader, MavMessage) {
    let header = MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 };
    let signed = auth::Operator::new(TEST_MISSION, test_keys(7).secret).sign_command(&plain_command(target_system, command, params), 1_000);
    (header, MavMessage::SWARM_SIGNED_COMMAND(signed))
}

#[test]
fn a_captured_drone_cant_give_orders() {
    let mut brain = test_brain(1, cfg(), 1);
    let rtl = forged_order(1, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]);
    let few = forged_order(0, MavCmd::MAV_CMD_SWARM_SET_RESPONDERS, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let mut fake_sticks = sticks(1, 1000, 0, 1);
    if let MavMessage::SWARM_SIGNED_STICKS(st) = &mut fake_sticks.1 {
        *st = auth::Operator::new(TEST_MISSION, test_keys(7).secret).sign_sticks(&mavlink_swarm::MANUAL_CONTROL_DATA { x: 1000, y: 0, z: 500, r: 0, buttons: 1, target: 1 }, 1_000);
    }
    for (i, f) in [rtl, few, fake_sticks].into_iter().enumerate() {
        brain.on_tick(&reading(1_000 + 50 * i as u32, Vec2::ZERO, None), &[f]);
        assert_eq!(orders_sent(&mut brain, 1_000 + 50 * i as u32), 0, "passed on or answered");
    }
    assert_eq!(brain.order(), None);
    assert_eq!(brain.cfg.responders, cfg().responders);
    assert_eq!(brain.orders_rejected(), 3);
}

#[test]
fn an_order_altered_on_the_way_is_refused() {
    let mut brain = test_brain(1, cfg(), 1);
    let (header, mut msg) = gcs_command(1, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, -6.0, 0.0, 5.0]);
    if let MavMessage::SWARM_SIGNED_COMMAND(c) = &mut msg {
        c.param5 = 6.0;
    }
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[(header, msg)]);
    assert_eq!(brain.order(), None);
    assert_eq!(brain.orders_rejected(), 1);
}

#[test]
fn an_old_order_played_back_is_ignored() {
    let mut brain = test_brain(1, cfg(), 1);
    let go = gcs_command_at(1_000, 1, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, -6.0, 0.0, 5.0]);
    let resume = gcs_command_at(1_500, 1, MavCmd::MAV_CMD_MISSION_START, [0.0; 7]);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[go.clone()]);
    brain.on_tick(&reading(1_500, Vec2::ZERO, None), &[resume]);
    assert_eq!(brain.order(), None);
    // Recorded and played back: older than the last order taken.
    brain.on_tick(&reading(2_000, Vec2::ZERO, None), &[go]);
    assert_eq!(brain.order(), None);
    // Nor does an order nobody heard back then work much later: it's stale by now.
    let rtl = gcs_command_at(3_000, 1, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]);
    brain.on_tick(&reading(3_000 + ORDER_MAX_AGE_MS + 50, Vec2::ZERO, None), &[rtl]);
    assert_eq!(brain.order(), None);
    // Nor an order for another mission.
    let mut other = auth::Operator::new(TEST_MISSION + 1, auth::Authority::from_secret([0xA5; 32]).operator_secret(TEST_MISSION + 1));
    let next_mission = other.sign_command(&plain_command(1, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]), 20_000);
    brain.on_tick(&reading(20_000, Vec2::ZERO, None), &[(gcs_header(), MavMessage::SWARM_SIGNED_COMMAND(next_mission))]);
    assert_eq!(brain.order(), None);
}

fn gcs_header() -> MavHeader {
    MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 }
}

#[test]
fn a_repeated_order_is_taken_once_but_passed_on_again() {
    let mut brain = test_brain(1, cfg(), 1);
    let for_me = gcs_command_at(1_000, 1, MavCmd::MAV_CMD_NAV_LOITER_UNLIM, [0.0; 7]);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[for_me.clone()]);
    assert_eq!(acked(&mut brain, 1_000, MavCmd::MAV_CMD_NAV_LOITER_UNLIM), Some(MavResult::MAV_RESULT_ACCEPTED));
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[for_me]);
    assert_eq!(acked(&mut brain, 1_050, MavCmd::MAV_CMD_NAV_LOITER_UNLIM), None, "carried out once");
    // For another drone: the ground station's repeat, a while later, is relayed again (that
    // drone may have missed the first), without another signature check.
    let for_9 = gcs_command_at(1_100, 9, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]);
    brain.on_tick(&reading(1_100, Vec2::ZERO, None), &[for_9.clone()]);
    assert_eq!(orders_sent(&mut brain, 1_100), 1);
    let checks = brain.checks_done;
    let later = 1_100 + RELAY_MEMORY_MS + 100;
    brain.on_tick(&reading(later, Vec2::ZERO, None), &[for_9]);
    assert_eq!(orders_sent(&mut brain, later), 1);
    assert_eq!(brain.checks_done, checks);
}

#[test]
fn a_flood_of_forged_orders_costs_a_check_a_tick_and_ours_go_first() {
    let mut brain = test_brain(1, cfg(), 1);
    let mut inbox: std::vec::Vec<_> = (0..20).map(|k| forged_order(9, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, k as f32, 0.0, 5.0])).collect();
    inbox.push(gcs_command(1, MavCmd::MAV_CMD_NAV_LOITER_UNLIM, [0.0; 7]));
    let checks = brain.checks_done;
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.checks_done - checks, OPERATOR_CHECKS_PER_TICK as u32);
    assert!(matches!(brain.order(), Some(Order::Hold(_))), "{:?}", brain.order());
}

fn revocation(sysid: u8) -> (MavHeader, MavMessage) {
    let r = auth::Authority::from_secret([0xA5; 32]).revoke(TEST_MISSION, sysid);
    (MavHeader { system_id: BASE_SYSID, component_id: 1, sequence: 0 }, MavMessage::SWARM_REVOKE(r))
}

#[test]
fn a_revoked_drone_is_no_longer_listened_to() {
    let target = Vec2::new(5.0, 0.0);
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[revocation(2)]);
    assert!(brain.revoked(2));
    // Relayed on for the rest of the swarm.
    assert!(brain.outgoing(1_000).into_iter().flatten().any(|m| matches!(m, MavMessage::SWARM_REVOKE(r) if r.sysid == 2)));
    // Its sightings, beacons and statuses count for nothing now: drone 3 alone is one
    // witness, not two.
    let status = (MavHeader { system_id: 2, component_id: COMPONENT_ID, sequence: 0 }, MavMessage::SWARM_DRONE_STATUS(SWARM_DRONE_STATUS_DATA { sysid: 2, x: 3.0, y: 3.0, seq: 1, ..Default::default() }));
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[sighting(2, target, 0), sighting(3, target, 0), heartbeat(2, 0), status]);
    assert_ne!(brain.mode(), Mode::Converging);
    assert!(brain.neighbors[2].is_none() && brain.swarm_heard[2].is_none());
}

#[test]
fn only_the_base_can_revoke_a_drone() {
    let mut brain = test_brain(1, cfg(), 1);
    // Signed by drone 7, claiming to be the base.
    let forged = auth::Authority::from_secret(test_keys(7).secret).revoke(TEST_MISSION, 2);
    let msg = (MavHeader { system_id: BASE_SYSID, component_id: 1, sequence: 0 }, MavMessage::SWARM_REVOKE(forged));
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[msg]);
    assert!(!brain.revoked(2));
    assert_eq!(brain.orders_rejected(), 1);
    assert!(!brain.outgoing(1_000).into_iter().flatten().any(|m| matches!(m, MavMessage::SWARM_REVOKE(_))));
}

#[test]
fn a_revoked_drone_comes_home() {
    let mut brain = test_brain(2, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[revocation(2)]);
    assert_eq!(brain.order(), Some(Order::ReturnHome));
}

#[test]
fn a_broadcast_go_to_is_ignored() {
    let mut brain = test_brain(1, cfg(), 1);
    let everyone = gcs_command(0, MavCmd::MAV_CMD_DO_REPOSITION, [0.0, 0.0, 0.0, 0.0, -6.0, 0.0, 5.0]);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[everyone]);
    assert_eq!(brain.order(), None);
}

#[test]
fn a_status_counts_only_from_its_own_drone() {
    let status = |author: u8, about: u8| {
        let header = MavHeader { system_id: author, component_id: COMPONENT_ID, sequence: 0 };
        let msg = MavMessage::SWARM_DRONE_STATUS(SWARM_DRONE_STATUS_DATA { sysid: about, x: 3.0, y: 3.0, hops: 1, seq: 1, ..Default::default() });
        (header, msg)
    };
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[status(7, 9), status(8, 8)]);
    assert!(brain.swarm_heard[9].is_none(), "drone 7 spoke for drone 9");
    assert!(brain.swarm_heard[8].is_some());
    // Only the genuine one is passed on, under its author's header, a moment later.
    brain.on_tick(&reading(1_200, Vec2::ZERO, None), &[]);
    let relayed: std::vec::Vec<_> = brain.outgoing_frames(1_200).into_iter().flatten().filter(|(_, m)| matches!(m, MavMessage::SWARM_DRONE_STATUS(st) if st.sysid != 1)).collect();
    assert_eq!(relayed.len(), 1);
    assert_eq!(relayed[0].0.map(|h| h.system_id), Some(8));
}

#[test]
fn a_sighting_neighbors_keep_repeating_isnt_repeated_by_everyone() {
    let sent = |brain: &mut Brain, now: u32, inbox: &[(MavHeader, MavMessage)]| {
        brain.on_tick(&reading(now, Vec2::ZERO, None), inbox);
        sightings_sent(brain, now).len()
    };
    let report = sighting(2, Vec2::new(5.0, 0.0), 0);
    // New to us: out at once. Then the same observation again only now and then...
    let mut brain = test_brain(1, cfg(), 1);
    assert_eq!(sent(&mut brain, 1_000, core::slice::from_ref(&report)), 1);
    assert_eq!(sent(&mut brain, 1_200, &[]), 0);
    assert_eq!(sent(&mut brain, 1_400, &[]), 1);
    // ...and not at all while three neighbors are relaying it, until the refresh.
    let relayed = {
        let mut r = report.clone();
        if let MavMessage::SWARM_TARGET_SIGHTING(s) = &mut r.1 {
            s.hops = 1;
        }
        r
    };
    let copies = [relayed.clone(), relayed.clone(), relayed];
    assert_eq!(sent(&mut brain, 1_600, &copies), 0);
    assert_eq!(sent(&mut brain, 1_800, &copies), 0);
    assert_eq!(sent(&mut brain, 3_400, &copies), 1);
}

#[test]
fn beacons_stretch_in_a_crowd() {
    let mut brain = test_brain(1, cfg(), 1);
    let crowd: std::vec::Vec<_> = (2..=41).map(|k| position(k, Vec2::new(k as f32 * 0.1, 1.0), Vec2::ZERO)).collect();
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &crowd[..10]);
    assert_eq!(brain.beacon_period(1_000), cfg().beacon_period_ms, "10 neighbors: no crowd");
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &crowd[..32]);
    assert_eq!(brain.beacon_period(1_050), 2 * cfg().beacon_period_ms);
    brain.on_tick(&reading(1_100, Vec2::ZERO, None), &crowd);
    assert_eq!(brain.beacon_period(1_100), 5 * cfg().beacon_period_ms / 2);
}

#[test]
fn a_status_neighbors_already_relayed_isnt_relayed_again() {
    let status = |hops: u8| {
        let header = MavHeader { system_id: 8, component_id: COMPONENT_ID, sequence: 0 };
        (header, MavMessage::SWARM_DRONE_STATUS(SWARM_DRONE_STATUS_DATA { sysid: 8, x: 3.0, y: 3.0, hops, seq: 1, ..Default::default() }))
    };
    let relays_of_8 = |brain: &mut Brain, now: u32| {
        brain.on_tick(&reading(now, Vec2::ZERO, None), &[]);
        brain.outgoing_frames(now).into_iter().flatten().filter(|(_, m)| matches!(m, MavMessage::SWARM_DRONE_STATUS(st) if st.sysid == 8)).count()
    };
    // Heard it, and then three neighbors passing it on: they've got it covered.
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[status(0)]);
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[status(1), status(1), status(1)]);
    assert_eq!(relays_of_8(&mut brain, 1_200), 0);
    // Only one did: this drone's copy may reach someone new.
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[status(0)]);
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[status(1)]);
    assert_eq!(relays_of_8(&mut brain, 1_200), 1);
}

#[test]
fn an_order_three_hops_out_still_comes_from_the_ground_station() {
    // The ground station reaches drones 1 and 2; drone 4 is three hops away (via 2, 3).
    let (brains, _) = relay_chain(4, 20, |tick| if tick == 0 { std::vec![gcs_command(4, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7])] } else { std::vec![] });
    assert_eq!(brains[3].order(), Some(Order::ReturnHome), "the order didn't arrive as the ground station's");
    assert!(brains[..3].iter().all(|b| b.order().is_none()));
}
