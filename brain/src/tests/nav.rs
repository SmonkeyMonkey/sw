//! Navigating with a jammed or spoofed GPS.

use super::*;

/// Not a pass/fail test: how the scenarios above fare with consumer-grade GPS error.
/// `cargo test --release gps_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn gps_survey() {
    for sigma_cm in [0u32, 150] {
        GPS_SIGMA_CM.with(|v| v.set(sigma_cm));
        RADIO_LOSS_PERCENT.with(|v| v.set(10));
        let runaway: std::vec::Vec<f32> = (1..=6).map(|s| follow_a_runaway(false, s)).collect();
        let passer: std::vec::Vec<f32> = (1..=6).map(runaway_passerby).collect();
        LATE_JOINER_TICK.with(|v| v.set(200));
        let newcomer_went: usize = (1..=6).map(|s| newcomer_at(s, &[Vec2::new(7.0, 0.0)])[10..].iter().filter(|(m, _)| *m != Mode::Searching).count()).sum();
        LATE_JOINER_TICK.with(|v| v.set(0));
        // Names: one name per mouse, and one mouse per name, with two mice 20 m apart.
        let targets = [Vec2::new(-10.0, 0.0), Vec2::new(10.0, 0.0)];
        let starts: std::vec::Vec<Vec2> = (0..12).map(|i| Vec2::new(-11.0 + 2.0 * i as f32, 1.0 + (i % 3) as f32)).collect();
        let mut names_per_mouse = [0usize; 2];
        for seed in 1..=6 {
            let drones = fly_swarm_with(RINGS_OF_FOUR, seed, &starts, &targets, 8.0, 1200, |_, _| false);
            for (k, t) in targets.iter().enumerate() {
                let ids: std::collections::BTreeSet<TargetId> = drones.iter().filter(|(_, p)| (*p - *t).length() < 4.0).filter_map(|(b, _)| b.assigned_target()).collect();
                names_per_mouse[k] += ids.len();
            }
        }
        RADIO_LOSS_PERCENT.with(|v| v.set(0));
        GPS_SIGMA_CM.with(|v| v.set(0));
        std::println!(
            "gps {:.1} m: runaway watched {:.0}%, passerby watched {:.0}%, newcomer seconds on a full target {newcomer_went}, names per mouse (ideal 6 over 6 runs) {names_per_mouse:?}",
            sigma_cm as f32 / 100.0,
            100.0 * runaway.iter().sum::<f32>() / 6.0,
            100.0 * passer.iter().map(|p| p.max(0.0)).sum::<f32>() / 6.0,
        );
    }
}

fn estimator_status(sender: u8, accuracy: f32) -> (MavHeader, MavMessage) {
    let header = MavHeader { system_id: sender, component_id: COMPONENT_ID, sequence: 0 };
    let msg = MavMessage::ESTIMATOR_STATUS(ESTIMATOR_STATUS_DATA {
        time_usec: 0,
        vel_ratio: 0.0,
        pos_horiz_ratio: 0.0,
        pos_vert_ratio: 0.0,
        mag_ratio: 0.0,
        hagl_ratio: 0.0,
        tas_ratio: 0.0,
        pos_horiz_accuracy: accuracy,
        pos_vert_accuracy: 0.0,
        flags: EstimatorStatusFlags::ESTIMATOR_POS_HORIZ_ABS,
    });
    (header, msg)
}

/// We're truly at `me`; two neighbors sure of their positions, which we range.
fn with_two_references(me: Vec2, gps: Option<Vec2>, time_ms: u32) -> (SensorReading, std::vec::Vec<(MavHeader, MavMessage)>) {
    let (a, b) = (me + Vec2::new(3.0, 0.0), me + Vec2::new(0.0, -4.0));
    let mut nearby = [None; MAX_NEARBY];
    nearby[0] = Some((2u8, a - me, Vec2::ZERO));
    nearby[1] = Some((3u8, b - me, Vec2::ZERO));
    let reading = SensorReading {
        time_ms,
        own_pos: gps.unwrap_or(Vec2::ZERO),
        gps_fix: gps.is_some(),
        own_velocity: Vec2::ZERO,
        targets_seen: [None; MAX_SEEN],
        nearby: Some(nearby),
        battery: f32::NAN,
    };
    let inbox = std::vec![position(2, a, Vec2::ZERO), estimator_status(2, 1.5), position(3, b, Vec2::ZERO), estimator_status(3, 1.5)];
    (reading, inbox)
}

#[test]
fn a_jammed_drone_navigates_by_its_neighbors() {
    let mut brain = test_brain(1, cfg(), 1);
    let me = Vec2::new(4.0, 6.0);
    let (reading, inbox) = with_two_references(me, None, 1_000);
    brain.on_tick(&reading, &inbox);
    let (pos, accuracy, nav) = brain.position();
    assert_eq!(nav, Nav::Neighbors);
    assert!((pos - me).length() < 0.01, "placed at {pos:?}");
    assert!(accuracy < 2.0);
}

#[test]
fn a_jammed_drone_alone_dead_reckons_and_grows_unsure() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::new(0.0, 0.0), None), &[]);
    let mut r = reading(1_000, Vec2::ZERO, None);
    r.gps_fix = false;
    r.own_velocity = Vec2::new(2.0, 0.0);
    for k in 1..=200u32 {
        brain.on_tick(&SensorReading { time_ms: 1_000 + 50 * k, ..r }, &[]);
    }
    let (pos, accuracy, nav) = brain.position();
    assert_eq!(nav, Nav::DeadReckoning);
    assert!((pos - Vec2::new(20.0, 0.0)).length() < 0.1, "integrated to {pos:?}");
    assert!(accuracy > 3.0, "ten seconds blind and still sure to {accuracy} m");
}

#[test]
fn spoofed_gps_is_caught_by_the_neighbors_and_ignored() {
    let mut brain = test_brain(1, cfg(), 1);
    let me = Vec2::new(4.0, 6.0);
    // GPS has been dragged 8 m away.
    let (reading, inbox) = with_two_references(me, Some(me + Vec2::new(8.0, 0.0)), 1_000);
    brain.on_tick(&reading, &inbox);
    let (pos, _, nav) = brain.position();
    assert_eq!(nav, Nav::Neighbors);
    assert!((pos - me).length() < 0.01);
    // ...and it tells everyone its GPS is off.
    let glitch = brain.outgoing(1_000).into_iter().flatten().any(|m| matches!(m, MavMessage::ESTIMATOR_STATUS(e) if e.flags.contains(EstimatorStatusFlags::ESTIMATOR_GPS_GLITCH)));
    assert!(glitch);
}

#[test]
fn ordinary_gps_error_is_not_mistaken_for_spoofing() {
    let mut brain = test_brain(1, cfg(), 1);
    let me = Vec2::new(4.0, 6.0);
    let (reading, inbox) = with_two_references(me, Some(me + Vec2::new(1.5, -1.0)), 1_000);
    brain.on_tick(&reading, &inbox);
    assert_eq!(brain.position().2, Nav::Gps);
}

#[test]
fn unsure_neighbors_are_no_reference() {
    let mut brain = test_brain(1, cfg(), 1);
    let me = Vec2::new(4.0, 6.0);
    let (mut reading, mut inbox) = with_two_references(me, None, 1_000);
    // Both are dead reckoning themselves, 6 m unsure.
    inbox[1] = estimator_status(2, 6.0);
    inbox[3] = estimator_status(3, 6.0);
    reading.gps_fix = false;
    brain.on_tick(&reading, &inbox);
    assert_eq!(brain.position().2, Nav::DeadReckoning);
}
