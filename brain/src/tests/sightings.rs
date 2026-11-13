//! Sightings: quorum, signatures, tracks and their names, keeping clear of neighbors.

use super::*;

#[test]
fn single_report_nearby_is_investigated_not_trusted() {
    let mut brain = test_brain(1, cfg(), 1);
    let out = brain.on_tick(
        &reading(1_000, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(5.0, 0.0), 0)],
    );
    assert_eq!(brain.mode(), Mode::Investigating);
    // Flies straight at the reported spot to check it.
    assert!(out.velocity.x > 1.9 && out.velocity.y.abs() < 1e-3);
}

#[test]
fn single_report_far_away_is_relayed_but_ignored() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(
        &reading(1_000, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(12.0, 0.0), 0)],
    );
    assert_eq!(brain.mode(), Mode::Searching);
    assert!(
        brain.informed(),
        "still carries the report so it can relay it"
    );
}

#[test]
fn two_independent_witnesses_reach_quorum() {
    let mut brain = test_brain(1, cfg(), 1);
    let inbox = [
        sighting(2, Vec2::new(12.0, 0.0), 0),
        sighting(3, Vec2::new(12.5, 0.5), 0),
    ];
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.mode(), Mode::Converging);
}

#[test]
fn same_witness_twice_is_still_one_vote() {
    let mut brain = test_brain(1, cfg(), 1);
    let inbox = [
        sighting(2, Vec2::new(12.0, 0.0), 200),
        sighting(2, Vec2::new(12.0, 0.0), 0),
    ];
    brain.on_tick(&reading(1_200, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.mode(), Mode::Searching);
}

#[test]
fn investigator_refutes_a_fresh_false_report_and_distrusts_its_source() {
    let mut brain = test_brain(1, cfg(), 1);
    let fake = Vec2::new(0.5, 0.0);
    // Standing on the reported spot, seeing nothing, while the liar keeps insisting.
    let out = brain.on_tick(
        &reading(1_000, Vec2::ZERO, None),
        &[sighting(LIAR, fake, 100)],
    );
    assert_eq!(out.refuted, Some(LIAR));
    assert_eq!(brain.mode(), Mode::Searching);
    assert!(!brain.informed());

    // Further lies are dropped entirely: not adopted, so not relayed either.
    let out = brain.on_tick(
        &reading(1_050, Vec2::ZERO, None),
        &[sighting(LIAR, fake, 0)],
    );
    assert_eq!(out.refuted, None);
    assert!(!brain.informed());

    // Trust comes back after `distrust_ms` (a report a few meters off, so it isn't instantly
    // refuted again from where we stand).
    brain.on_tick(
        &reading(16_100, Vec2::ZERO, None),
        &[sighting(LIAR, Vec2::new(5.0, 0.0), 0)],
    );
    assert_eq!(brain.mode(), Mode::Investigating);
}

#[test]
fn a_made_up_sighting_in_an_honest_drones_name_doesnt_get_it_distrusted() {
    let mut brain = test_brain(1, cfg(), 1);
    let fake = Vec2::new(0.5, 0.0);
    // Drone 9 sends it, naming drone 3 as having seen it, firsthand.
    let mut framed = sighting(3, fake, 0);
    framed.0.system_id = 9;
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[framed]);
    // Nothing there: the report is dropped, but nobody can be blamed for it.
    assert_eq!(out.refuted, None);
    assert!(!brain.informed());
    assert!(!brain.distrusts(3, 1_050));
    // Drone 3's own reports still count.
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[sighting(3, Vec2::new(5.0, 0.0), 0)]);
    assert_eq!(brain.mode(), Mode::Investigating);
}

pub(super) fn sighting_data(msg: &MavMessage) -> SWARM_TARGET_SIGHTING_DATA {
    match msg {
        MavMessage::SWARM_TARGET_SIGHTING(s) => s.clone(),
        _ => panic!("not a sighting"),
    }
}

#[test]
fn a_sighting_signed_by_someone_else_than_its_observer_is_dropped() {
    let mut brain = test_brain(1, cfg(), 1);
    // The liar signs with its own key, but says drone 2 saw it.
    let (header, msg) = sighting(LIAR, Vec2::new(5.0, 0.0), 0);
    let mut s = sighting_data(&msg);
    s.observer_sysid = 2;
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[(header, MavMessage::SWARM_TARGET_SIGHTING(s))]);
    assert!(!brain.informed());
    assert_eq!(brain.sightings_rejected(), (1, 0));
}

#[test]
fn a_sighting_altered_on_the_way_is_dropped() {
    let mut brain = test_brain(1, cfg(), 1);
    let (header, msg) = sighting(2, Vec2::new(5.0, 0.0), 0);
    let mut s = sighting_data(&msg);
    s.x += 3.0;
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[(header, MavMessage::SWARM_TARGET_SIGHTING(s))]);
    assert!(!brain.informed());
    assert_eq!(brain.sightings_rejected(), (1, 0));
    // What relays are free to change, they can.
    let (header, msg) = sighting(2, Vec2::new(5.0, 0.0), 0);
    let mut s = sighting_data(&msg);
    (s.hops, s.age_ms, s.responders, s.target_id) = (3, 400, 2, 0x0301);
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[(header, MavMessage::SWARM_TARGET_SIGHTING(s))]);
    assert!(brain.informed());
}

#[test]
fn a_captured_drone_cant_make_up_a_quorum() {
    // The liar reports a target in its own name (that's one vote, honestly signed) and in
    // the names of three others (forged): still one vote, so nobody's convinced.
    let mut brain = test_brain(1, cfg(), 1);
    let fake = Vec2::new(25.0, 0.0);
    for (k, now) in (1_000..3_000).step_by(200).enumerate() {
        let mut inbox = std::vec![sighting(LIAR, fake, 0)];
        for other in [2, 3, 4] {
            let (_, msg) = sighting(LIAR, fake, 0);
            let mut s = sighting_data(&msg);
            s.observer_sysid = other;
            inbox.push((MavHeader { system_id: LIAR, component_id: COMPONENT_ID, sequence: k as u8 }, MavMessage::SWARM_TARGET_SIGHTING(s)));
        }
        brain.on_tick(&reading(now, Vec2::ZERO, None), &inbox);
        assert!(brain.known_targets(now).iter().flatten().all(|&(_, confirmed)| !confirmed), "confirmed by one liar at {now}");
    }
    assert_eq!(brain.mode(), Mode::Searching, "too far to investigate, and never confirmed");
}

#[test]
fn an_old_sighting_replayed_as_new_is_still_old() {
    let mut brain = test_brain(1, cfg(), 1);
    let header = MavHeader { system_id: 2, component_id: COMPONENT_ID, sequence: 0 };
    let at = |x: f32, observed_ms: u32, age_ms: u32| {
        let (_, msg) = sighting(2, Vec2::new(x, 0.0), age_ms);
        let mut s = sighting_data(&msg);
        test_keys(2).sign_sighting(&mut s, observed_ms);
        (header, MavMessage::SWARM_TARGET_SIGHTING(s))
    };
    // Drone 2 saw it at (5, 0), and five seconds later at (5.5, 0).
    let old = at(5.0, 100, 0);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), core::slice::from_ref(&old));
    brain.on_tick(&reading(6_000, Vec2::ZERO, None), &[at(5.5, 5_100, 0)]);
    // Someone replays the first one, claiming it's brand new.
    let replay = (old.0, MavMessage::SWARM_TARGET_SIGHTING(SWARM_TARGET_SIGHTING_DATA { age_ms: 0, ..sighting_data(&old.1) }));
    brain.on_tick(&reading(6_100, Vec2::ZERO, None), &[replay]);
    let s = sent_sighting(&mut brain, 6_100);
    assert_eq!(s.x, 5.5, "the replay passed for news");
    assert!(s.age_ms >= 100, "age {}", s.age_ms);
}

#[test]
fn a_sighting_waits_for_its_observers_certificate() {
    let mut brain = Brain::new(1, cfg(), 1, test_keys(1));
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[sighting(2, Vec2::new(5.0, 0.0), 0)]);
    assert!(!brain.informed(), "taken without knowing whose key signed it");
    // A certificate from some other authority, or for another mission, teaches it nothing.
    let header = MavHeader { system_id: 2, component_id: COMPONENT_ID, sequence: 0 };
    let rogue = auth::Authority::from_secret([0x11; 32]).issue(TEST_MISSION, 2, [0x22; 32]);
    let old_mission = auth::Authority::from_secret([0xA5; 32]).issue(TEST_MISSION - 1, 2, [2 ^ 0x3C; 32]);
    for keys in [rogue, old_mission] {
        brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[(header, MavMessage::SWARM_KEY_CERT(keys.certificate_msg()))]);
        assert!(!brain.informed());
    }
    // The real one: the sighting waiting since the first tick checks out.
    brain.on_tick(&reading(1_100, Vec2::ZERO, None), &[(header, MavMessage::SWARM_KEY_CERT(test_keys(2).certificate_msg()))]);
    assert!(brain.informed());
}

#[test]
fn only_a_few_signatures_are_checked_a_tick() {
    let mut brain = test_brain(1, cfg(), 1);
    let inbox: std::vec::Vec<_> = (2..=6).map(|o| sighting(o, Vec2::new(5.0, 0.0), 0)).collect();
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    let witnesses = |b: &Brain| b.tracks.iter().flatten().map(|t| t.witnesses.iter().flatten().count()).sum::<usize>();
    assert_eq!(witnesses(&brain), MAX_VERIFY_PER_TICK as usize);
    // The rest next tick, along with whatever copies of the first arrive: those are free.
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &inbox[..2]);
    assert_eq!(witnesses(&brain), 5);
    assert_eq!(brain.sightings_rejected(), (0, 0));
}

#[test]
fn a_drone_new_to_the_swarm_checks_sightings_from_what_comes_with_them() {
    // Neither knows the other: the certificate goes out with the sighting.
    let mut a = Brain::new(2, cfg(), 1, test_keys(2));
    let mut b = Brain::new(3, cfg(), 2, test_keys(3));
    a.on_tick(&reading(1_000, Vec2::ZERO, Some(Vec2::new(2.0, 0.0))), &[]);
    let header = MavHeader { system_id: 2, component_id: COMPONENT_ID, sequence: 0 };
    let sent: std::vec::Vec<_> = a.outgoing(1_000).into_iter().flatten().map(|m| (header, m)).collect();
    assert!(sent.iter().any(|(_, m)| matches!(m, MavMessage::SWARM_KEY_CERT(c) if c.sysid == 2)));
    b.on_tick(&reading(1_050, Vec2::new(10.0, 0.0), None), &sent);
    assert!(b.informed());
    // And b passes a's certificate on with a's sighting, for drones farther off.
    let relayed: std::vec::Vec<_> = b.outgoing(1_050).into_iter().flatten().collect();
    assert!(relayed.iter().any(|m| matches!(m, MavMessage::SWARM_KEY_CERT(c) if c.sysid == 2)));
    assert!(relayed.iter().any(|m| matches!(m, MavMessage::SWARM_TARGET_SIGHTING(s) if s.observer_sysid == 2)));
}

#[test]
fn stale_report_is_dropped_without_blaming_its_source() {
    let mut brain = test_brain(1, cfg(), 1);
    // 2 s old: the target may just have moved on since.
    let out = brain.on_tick(
        &reading(3_000, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(0.5, 0.0), 2000)],
    );
    assert_eq!(out.refuted, None);
    assert!(!brain.informed());
    brain.on_tick(
        &reading(3_050, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(5.0, 0.0), 0)],
    );
    assert_eq!(
        brain.mode(),
        Mode::Investigating,
        "observer 2 is still trusted"
    );
}

#[test]
fn a_nearer_lie_is_checked_then_the_real_target_wins() {
    let mut brain = test_brain(1, cfg(), 1);
    let real = Vec2::new(-8.0, 0.0);
    let fake = Vec2::new(5.0, 0.0);
    let inbox = [sighting(2, real, 100), sighting(3, real, 100), sighting(LIAR, fake, 0)];
    // The lone report is nearer, so it's the one to check first.
    let out = brain.on_tick(&reading(1_100, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.mode(), Mode::Investigating);
    assert!(out.velocity.x > 0.0);
    // There, nothing to see: the liar is caught, and the confirmed target is next.
    let inbox = [sighting(2, real, 0), sighting(3, real, 0), sighting(LIAR, fake, 0)];
    let out = brain.on_tick(&reading(3_600, fake, None), &inbox);
    assert_eq!(out.refuted, Some(LIAR));
    let out = brain.on_tick(&reading(3_650, fake, None), &[sighting(2, real, 0), sighting(3, real, 0), sighting(LIAR, fake, 0)]);
    assert_eq!(brain.mode(), Mode::Converging);
    assert!(out.velocity.x < 0.0, "heads for the real target");
}

#[test]
fn seeing_the_target_needs_no_quorum() {
    let mut brain = test_brain(1, cfg(), 1);
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, Some(Vec2::new(2.5, 0.0))), &[]);
    assert_eq!(brain.mode(), Mode::Converging);
    assert!(matches!(out.event, Some(TraceEvent::Spotted)));
}

pub(super) fn position(sender: u8, pos: Vec2, vel: Vec2) -> (MavHeader, MavMessage) {
    let header = MavHeader {
        system_id: sender,
        component_id: COMPONENT_ID,
        sequence: 0,
    };
    let msg = MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
        time_boot_ms: 0,
        x: pos.x,
        y: pos.y,
        z: 0.0,
        vx: vel.x,
        vy: vel.y,
        vz: 0.0,
    });
    (header, msg)
}

#[test]
fn dodges_a_head_on_neighbor_before_it_is_close() {
    let mut brain = test_brain(1, cfg(), 1);
    // Investigating a report straight ahead, so it's flying +x at full speed...
    brain.on_tick(
        &reading(1_000, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(8.0, 0.0), 0)],
    );
    // ...while drone 3, 3 m ahead (well outside the separation radius), flies straight at it.
    let cruising = SensorReading {
        own_velocity: Vec2::new(2.0, 0.0),
        ..reading(1_050, Vec2::ZERO, None)
    };
    let out = brain.on_tick(
        &cruising,
        &[
            position(3, Vec2::new(3.0, 0.0), Vec2::new(-2.0, 0.0)),
            sighting(2, Vec2::new(8.0, 0.0), 0),
        ],
    );
    assert!(
        out.velocity.y.abs() > 0.3,
        "no sideways dodge: {:?}",
        out.velocity
    );
}

#[test]
fn ignores_a_neighbor_flying_parallel() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(
        &reading(1_000, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(8.0, 0.0), 0)],
    );
    let cruising = SensorReading {
        own_velocity: Vec2::new(2.0, 0.0),
        ..reading(1_050, Vec2::ZERO, None)
    };
    let out = brain.on_tick(
        &cruising,
        &[
            position(3, Vec2::new(0.0, 1.5), Vec2::new(2.0, 0.0)),
            sighting(2, Vec2::new(8.0, 0.0), 0),
        ],
    );
    assert!(
        out.velocity.y.abs() < 1e-3,
        "dodged a drone that isn't in the way: {:?}",
        out.velocity
    );
}

pub(super) fn sent_sighting(brain: &mut Brain, now: u32) -> SWARM_TARGET_SIGHTING_DATA {
    brain
        .outgoing(now)
        .into_iter()
        .flatten()
        .find_map(|m| match m {
            MavMessage::SWARM_TARGET_SIGHTING(s) => Some(s),
            _ => None,
        })
        .expect("no sighting broadcast")
}

#[test]
fn news_travels_between_unsynchronized_clocks() {
    // A has been up for 1000 s, B for 7 s: their clocks have nothing in common.
    let mut a = test_brain(2, cfg(), 1);
    let mut b = test_brain(3, cfg(), 2);
    a.on_tick(&reading(1_000_000, Vec2::ZERO, Some(Vec2::new(2.0, 0.0))), &[]);
    let report = sent_sighting(&mut a, 1_000_300);
    assert_eq!(report.age_ms, 300);

    let header = MavHeader { system_id: 2, component_id: COMPONENT_ID, sequence: 0 };
    b.on_tick(&reading(7_000, Vec2::new(30.0, 0.0), None), &[(header, MavMessage::SWARM_TARGET_SIGHTING(report))]);
    assert!(b.informed(), "a report 'from the future' must not be dropped");
    // B relays it 500 ms later on its own clock: the age keeps growing, the clock doesn't matter.
    assert_eq!(sent_sighting(&mut b, 7_500).age_ms, 800);
}

#[test]
fn fresher_observation_wins_across_clocks() {
    // Just booted (clock at 50 ms): one target, seen by 2 two seconds ago and by 3 just now.
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(
        &reading(50, Vec2::ZERO, None),
        &[sighting(2, Vec2::new(12.0, 0.0), 2_000), sighting(3, Vec2::new(12.5, 0.0), 100)],
    );
    brain.on_tick(&reading(100, Vec2::ZERO, None), &[sighting(2, Vec2::new(12.0, 0.0), 2_050)]);
    let s = sent_sighting(&mut brain, 100);
    assert_eq!((s.observer_sysid, s.x), (3, 12.5));
}

#[test]
fn estimates_target_velocity_from_its_own_sightings() {
    let mut brain = test_brain(1, cfg(), 1);
    for tick in 0..20u32 {
        let t = 1_000 + tick * 50;
        // Target moving +y at 0.8 m/s.
        let target = Vec2::new(1.0, 0.8 * (tick as f32 * 0.05));
        brain.on_tick(&reading(t, Vec2::ZERO, Some(target)), &[]);
    }
    let s = sent_sighting(&mut brain, 1_950);
    assert!((s.vy - 0.8).abs() < 0.05 && s.vx.abs() < 0.05, "estimated ({}, {})", s.vx, s.vy);
}

#[test]
fn heads_for_where_a_moving_target_will_be() {
    let mut brain = test_brain(1, cfg(), 1);
    // Seen 1 s ago at (5, 0), moving +y at 2 m/s: by now it's at (5, 2).
    let header = MavHeader { system_id: 2, component_id: COMPONENT_ID, sequence: 0 };
    let report = MavMessage::SWARM_TARGET_SIGHTING(signed(SWARM_TARGET_SIGHTING_DATA {
        target_id: 0x0201,
        age_ms: 1_000,
        x: 5.0,
        y: 0.0,
        vx: 0.0,
        vy: 2.0,
        observer_sysid: 2,
        cancelled: 0,
        responders: 0,
        hops: 0,
        trace_id: [0; 16],
        parent_span_id: [0; 8],
        observed_ms: 0,
        signature: [0; 64],
    }, 2));
    let out = brain.on_tick(&reading(5_000, Vec2::ZERO, None), &[(header, report)]);
    assert_eq!(brain.mode(), Mode::Investigating);
    let expected = Vec2::new(5.0, 2.0).normalized() * 2.0;
    assert!((out.velocity - expected).length() < 0.05, "flew {:?}, expected {:?}", out.velocity, expected);
}

pub(super) fn sightings_sent(brain: &mut Brain, now: u32) -> std::vec::Vec<SWARM_TARGET_SIGHTING_DATA> {
    brain
        .outgoing(now)
        .into_iter()
        .flatten()
        .filter_map(|m| match m {
            MavMessage::SWARM_TARGET_SIGHTING(s) => Some(s),
            _ => None,
        })
        .collect()
}

#[test]
fn tells_two_targets_apart() {
    let mut brain = test_brain(1, cfg(), 1);
    let mut seen = [None; MAX_SEEN];
    seen[0] = Some(Vec2::new(1.0, 0.0));
    seen[1] = Some(Vec2::new(-1.0, 4.0));
    let both = SensorReading { targets_seen: seen, ..reading(1_000, Vec2::ZERO, None) };
    brain.on_tick(&both, &[]);
    brain.on_tick(&SensorReading { time_ms: 1_050, ..both }, &[]);
    let mut ids: std::vec::Vec<_> = sightings_sent(&mut brain, 1_050).iter().map(|s| s.target_id).collect();
    ids.sort();
    assert_eq!(ids, [0x0101, 0x0102], "one track per target, kept across ticks");
}

#[test]
fn only_the_closest_drones_respond() {
    let target = Vec2::new(6.0, 0.0);
    let confirmed = [sighting(2, target, 0), sighting(3, target, 0)];
    let near_target = |n: u8| -> std::vec::Vec<(MavHeader, MavMessage)> {
        (0..n).map(|k| position(10 + k, target + Vec2::new(0.5 * k as f32, 1.0), Vec2::ZERO)).collect()
    };

    // Four drones already closer: the target has its responders, so keep searching.
    let mut brain = test_brain(1, cfg(), 1);
    let inbox: std::vec::Vec<_> = near_target(4).into_iter().chain(confirmed.clone()).collect();
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.mode(), Mode::Searching);
    assert!(brain.informed(), "still knows about it, and relays it");

    // Only three closer: this drone is the fourth responder.
    let mut brain = test_brain(1, cfg(), 1);
    let inbox: std::vec::Vec<_> = near_target(3).into_iter().chain(confirmed).collect();
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.mode(), Mode::Converging);
    assert_eq!(brain.assigned_target(), Some(0x0201));
}

#[test]
fn duplicate_tracks_merge_into_the_lowest_id() {
    let mut brain = test_brain(1, cfg(), 1);
    // Drones 5 and 2 spotted the same target independently, each naming it after itself.
    brain.on_tick(
        &reading(1_000, Vec2::new(-10.0, 0.0), None),
        &[sighting(5, Vec2::new(4.0, 0.0), 0), sighting(2, Vec2::new(4.5, 0.0), 0)],
    );
    let sent = sightings_sent(&mut brain, 1_000);
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0].target_id, 0x0201);
    assert_eq!(brain.mode(), Mode::Converging, "two witnesses: confirmed");
}

pub(super) fn heartbeat(sender: u8, assigned: TargetId) -> (MavHeader, MavMessage) {
    let header = MavHeader { system_id: sender, component_id: COMPONENT_ID, sequence: 0 };
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        // No route to the ground station (link age 127); settled, if on a target.
        custom_mode: assigned as u32 | (ANNOUNCE_NO_LINK << 16) | if assigned != 0 { ANNOUNCE_SETTLED } else { 0 },
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });
    (header, msg)
}

#[test]
fn neighbors_busy_with_another_target_dont_count() {
    let target = Vec2::new(6.0, 0.0);
    let mut inbox = std::vec![sighting(2, target, 0), sighting(3, target, 0)];
    // Four drones closer to this target, but all holding another one nearby.
    for k in 0..4u8 {
        inbox.push(position(10 + k, target + Vec2::new(0.5 * k as f32, 1.0), Vec2::ZERO));
        inbox.push(heartbeat(10 + k, 0x0901));
    }
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    assert_eq!(brain.assigned_target(), Some(0x0201));
    // And it tells its neighbors right away.
    let announced = brain.outgoing(1_000).into_iter().flatten().find_map(|m| match m {
        MavMessage::HEARTBEAT(hb) => Some(hb.custom_mode & 0xFFFF),
        _ => None,
    });
    assert_eq!(announced, Some(0x0201));
}

#[test]
fn an_oversubscribed_target_sheds_its_newest_responder() {
    let target = Vec2::new(4.0, 0.0);
    // New observations every time: the same signed one over and over is a replay, as old
    // as when it was first heard.
    let confirmed = || [sighting(2, target, 0), sighting(3, target, 0)];
    let on_ring = |k: u8| {
        let angle = k as f32 * 1.5;
        target + Vec2::new(libm::cosf(angle), libm::sinf(angle)) * 2.0
    };
    let me = target + Vec2::new(0.0, -2.0);

    // Just took the target on, then hears four others already holding the ring: it leaves,
    // whatever its sysid.
    for sysid in [5, 20] {
        let mut brain = test_brain(sysid, cfg(), 1);
        brain.on_tick(&reading(1_000, me, None), &confirmed());
        assert_eq!(brain.assigned_target(), Some(0x0201));
        let mut inbox = confirmed().to_vec();
        for k in 0..4u8 {
            inbox.push(position(10 + k, on_ring(k), Vec2::ZERO));
            inbox.push(heartbeat(10 + k, 0x0201));
        }
        brain.on_tick(&reading(1_050, me, None), &inbox);
        assert_eq!(brain.assigned_target(), None, "sysid {sysid}");
    }

    // Settled on the ring with three others; then drone 1 arrives and joins too. The
    // newcomer is the one too many, not this drone — though its sysid is the highest.
    let mut brain = test_brain(20, cfg(), 1);
    let ring = |free_or_on: TargetId| {
        let mut inbox = confirmed().to_vec();
        for k in 0..3u8 {
            inbox.push(position(10 + k, on_ring(k), Vec2::ZERO));
            inbox.push(heartbeat(10 + k, 0x0201));
        }
        inbox.push(position(1, on_ring(3), Vec2::ZERO));
        // Not settled yet: it just got here.
        inbox.push(heartbeat_raw(1, free_or_on as u32 | (ANNOUNCE_NO_LINK << 16)));
        inbox
    };
    for now in (1_000..=8_000).step_by(1_000) {
        brain.on_tick(&reading(now, me, None), &ring(0));
    }
    assert_eq!(brain.assigned_target(), Some(0x0201));
    brain.on_tick(&reading(8_050, me, None), &ring(0x0201));
    assert_eq!(brain.assigned_target(), Some(0x0201), "a newcomer pushed a settled responder off");
}

pub(super) fn seeing(time_ms: u32, targets: &[Vec2]) -> SensorReading {
    let mut seen = [None; MAX_SEEN];
    for (slot, &t) in seen.iter_mut().zip(targets) {
        *slot = Some(t);
    }
    SensorReading { targets_seen: seen, ..reading(time_ms, Vec2::ZERO, None) }
}

#[test]
fn two_targets_passing_each_other_keep_their_names() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&seeing(1_000, &[Vec2::new(-1.5, 0.0), Vec2::new(1.5, 0.0)]), &[]);
    // They pass within 0.4 m of each other, then part.
    brain.on_tick(&seeing(1_050, &[Vec2::new(-0.2, 0.0), Vec2::new(0.2, 0.0)]), &[]);
    brain.on_tick(&seeing(1_100, &[Vec2::new(-1.5, 0.5), Vec2::new(1.5, 0.5)]), &[]);
    let mut ids: std::vec::Vec<_> = sightings_sent(&mut brain, 1_100).iter().map(|s| s.target_id).collect();
    ids.sort();
    assert_eq!(ids, [0x0101, 0x0102]);
}

#[test]
fn a_name_on_the_wrong_target_doesnt_drag_ours_along() {
    let mut brain = test_brain(1, cfg(), 1);
    // Our track 0x0201 is at (5, 0) — confirmed by two witnesses.
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[sighting(2, Vec2::new(5.0, 0.0), 0), sighting(3, Vec2::new(5.0, 0.0), 0)]);
    // A fresher report also called 0x0201, but 12 m away: some other target with a mixed-up name.
    let mut stray = sighting(4, Vec2::new(5.0, 12.0), 0);
    if let MavMessage::SWARM_TARGET_SIGHTING(s) = &mut stray.1 {
        // The name isn't signed: it's the relays' to settle.
        s.target_id = 0x0201;
    }
    brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[stray]);
    let sent = sightings_sent(&mut brain, 1_050);
    let ours = sent.iter().find(|s| s.x == 5.0 && s.y == 0.0);
    assert!(ours.is_some(), "our track stayed put: {:?}", sent.iter().map(|s| (s.target_id, s.x, s.y)).collect::<std::vec::Vec<_>>());
    assert_eq!(sent.len(), 2, "the stray report is a separate track");
}
