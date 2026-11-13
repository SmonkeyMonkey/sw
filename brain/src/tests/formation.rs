//! Fronts and the line they fly.

use super::*;

fn strip() -> BrainConfig {
    let half = Vec2::new(110.0, 15.0);
    BrainConfig { field_half: half, home: Vec2::new(2.0 - half.x, 2.0 - half.y), fleet: 22, ..BrainConfig::SWARM }.with_mode(SwarmMode::Autonomous)
}

fn square(side: f32) -> BrainConfig {
    let half = Vec2::new(side / 2.0, side / 2.0);
    BrainConfig { field_half: half, home: Vec2::new(2.0 - half.x, 2.0 - half.y), fleet: 12, ..BrainConfig::SWARM }.with_mode(SwarmMode::Autonomous)
}

/// Drone `sysid` after `secs` of flying its formation slot, hearing drones `others` free and
/// flying theirs (as it works them out), and drones `busy` on a mouse. From 1 s in.
fn in_formation(brain: &mut Brain, others: &[u8], busy: &[u8], from_s: u32, secs: u32) {
    for t in from_s * 20..(from_s + secs) * 20 {
        let now = 1_000 + 50 * t;
        let at = |m: u8| brain.slot_of(m, now).unwrap_or(brain.cfg.home);
        let moving = |m: u8| (brain.slot_of(m, now + 500).unwrap_or(at(m)) - at(m)) * 2.0;
        let mut inbox: std::vec::Vec<_> = others.iter().map(|&m| heartbeat(m, 0)).collect();
        inbox.extend(others.iter().map(|&m| position(m, at(m), moving(m))));
        inbox.extend(busy.iter().map(|&m| heartbeat(m, 0x0301)));
        let me = at(brain.sysid);
        brain.on_tick(&reading(now, me, None), &inbox);
    }
}

/// The loop `brain`'s front flies, closed up.
fn closed_path(brain: &Brain) -> FrontPath {
    let f = brain.front.unwrap();
    assert!(f.closed, "{f:?}");
    brain.cfg.front_path(f.fronts, f.index, f.count)
}

/// The largest gap between two drones side by side in `brain`'s front at `now`.
fn widest_gap(brain: &Brain, now: u32) -> f32 {
    let slots: std::vec::Vec<Vec2> = brain.front_places().iter().map(|&m| brain.slot_of(m, now).unwrap()).collect();
    slots.windows(2).map(|w| (w[1] - w[0]).length()).fold(0.0, f32::max)
}

#[test]
fn a_few_free_drones_stay_a_line_in_radio_range() {
    // Four of twelve left free on a 60 x 60 m field: not spread over the twelve's places
    // (29 m apart), but a line a spacing apart, mowing the field in lanes.
    let mut brain = test_brain(1, square(60.0), 1);
    in_formation(&mut brain, &[2, 3, 4], &(5..=12).collect::<std::vec::Vec<u8>>(), 0, 3);
    assert_eq!(brain.front_members(), &[1, 2, 3, 4]);
    let path = closed_path(&brain);
    assert!(path.lanes >= 3, "{path:?}");
    for now in (4_000..300_000).step_by(1_000) {
        let gap = widest_gap(&brain, now);
        assert!(gap <= brain.cfg.formation_spacing() + 0.01, "{gap} m apart at {now} ms");
    }
}

#[test]
fn a_line_closes_up_when_a_drone_leaves() {
    let mut brain = test_brain(1, square(60.0), 1);
    in_formation(&mut brain, &[2, 3, 4, 5, 6], &[], 0, 30);
    let now = 1_000 + 50 * 30 * 20;
    let before = brain.slot_of(1, now).unwrap();
    // Drone 6 goes for a mouse: the line holds its gap a while, then closes up.
    in_formation(&mut brain, &[2, 3, 4, 5], &[6], 30, 1);
    let held = brain.slot_of(1, now + 1_000).unwrap();
    assert_eq!(brain.front_members(), &[1, 2, 3, 4, 5, 6], "not yet");
    assert!((held - before).length() < 2.0, "{before:?} {held:?}");
    in_formation(&mut brain, &[2, 3, 4, 5], &[6], 31, 6);
    assert_eq!(brain.front_members(), &[1, 2, 3, 4, 5]);
    // Too few for the six places of their segment a spacing apart: a spacing apart, closed
    // up toward the middle of the front, where the other segment is.
    let gap = widest_gap(&brain, now + 7_000);
    assert!((gap - brain.cfg.formation_spacing()).abs() < 0.01, "{gap}");
    let middle = brain.cfg.front_path(1, 0, 12).center(0.0);
    let (first, last) = (brain.slot_of(1, now + 7_000).unwrap(), brain.slot_of(5, now + 7_000).unwrap());
    assert!((last - middle).length() < (first - middle).length(), "{first:?} {last:?}");
}

#[test]
fn where_a_front_is_goes_by_the_clock_not_by_whom_a_drone_hears() {
    // 180 x 175 m, 35 drones, one front in five segments of seven: drones 30 and 34, in the
    // fifth, hearing the drones of their segment and of the one beside it toward the middle
    // (each hears the others, not itself), work out the same places for their segment as
    // each other and as the plan with all of them, whenever (the segments farther off they
    // count as the plan does, by whom they take to be free).
    let cfg = BrainConfig { fleet: 35, ..field(180.0, 175.0) };
    let heard = |me: u8| (22..=35).filter(move |&m| m != me).collect::<std::vec::Vec<u8>>();
    let (mut a, mut b) = (test_brain(30, cfg, 1), test_brain(34, cfg, 1));
    let mut from_s = 0;
    for now in [5_000, 60_000, 200_000] {
        let to_s = now / 1000 - 1;
        in_formation(&mut a, &heard(30), &[], from_s, to_s - from_s);
        in_formation(&mut b, &heard(34), &[], from_s, to_s - from_s);
        from_s = to_s;
        for m in [30, 31, 32, 33] {
            let (pa, pb) = (a.slot_of(m, now).unwrap(), b.slot_of(m, now).unwrap());
            let planned = cfg.formation_slot(m as usize - 1, 35, now).unwrap();
            assert!((pa - pb).length() < 0.01 && (pa - planned).length() < 0.01, "drone {m} at {now} ms: {pa:?} {pb:?} {planned:?}");
        }
    }
}

#[test]
fn a_front_short_of_drones_closes_up_in_its_own_stretch() {
    // 220 x 30 m: eighteen drones are three fronts, a stretch each. With most on mice, the
    // four left of the western front close up into a line mowing its stretch in lanes.
    assert_eq!(strip().sweep_layout(18).rows, 3);
    let mut brain = test_brain(1, strip(), 1);
    in_formation(&mut brain, &[2, 3, 4], &(5..=22).collect::<std::vec::Vec<u8>>(), 0, 3);
    assert_eq!(brain.front_members(), &[1, 2, 3, 4]);
    let path = closed_path(&brain);
    assert!(path.band.1 < -30.0 && path.lanes > 1, "{path:?}");
}

#[test]
fn a_line_short_of_drones_is_as_far_along_as_one_full() {
    // However many lanes a front's drones take, it's as far along its stretch at any time:
    // one leaving or joining moves it across the field, not along.
    let cfg = strip();
    for q in (0..2_000).map(|q| q as f32 * 0.7) {
        let v = |members: usize| cfg.front_path(3, 1, members).at(q).1;
        assert!((1..=6).all(|m| (v(m) - v(6)).abs() < 1e-3), "{q}: {:?}", (1..=6).map(v).collect::<std::vec::Vec<_>>());
    }
    // And looks over every lane both ways.
    let path = cfg.front_path(3, 1, 2);
    assert_eq!(path.lanes, 3);
    let passes: std::vec::Vec<usize> = (0..6).map(|p| path.lane_of(p)).collect();
    assert_eq!(passes, [0, 1, 2, 2, 1, 0]);
}

#[test]
fn fronts_set_out_from_the_ends_nearer_the_base() {
    // 220 x 30 m, three fronts: starting from the ends of their stretches nearer the base
    // (the first at the base's end, the other two at the boundary between them), not the
    // third at the far end.
    let cfg = strip();
    assert_eq!(cfg.sweep_layout(22).rows, 3);
    let farthest = (0..3).map(|k| (cfg.front_path(3, k, 7).center(0.0) - cfg.home).length()).fold(0.0, f32::max);
    assert!(farthest < 150.0, "{farthest}");
}

#[test]
fn segments_line_up_from_the_middle_whatever_is_free_in_each() {
    // 80 x 80 m, 20 drones, one front in three segments: the middle one gone for mice, the
    // others' free drones close up across it into one line, a spacing apart.
    let cfg = BrainConfig { fleet: 20, ..field(80.0, 80.0) };
    let mut brain = test_brain(1, cfg, 1);
    let others: std::vec::Vec<u8> = [2, 3, 4].into_iter().chain(15..=20).collect();
    let busy: std::vec::Vec<u8> = (5..=14).collect();
    in_formation(&mut brain, &others, &busy, 0, 3);
    let gathered = (brain.front.unwrap().start_ms + ASSEMBLE_MS) / 1000;
    in_formation(&mut brain, &others, &busy, gathered, 2);
    let f = brain.front.unwrap();
    assert!(f.segments == 3 && !f.closed, "{f:?}");
    let gap = widest_gap(&brain, (gathered + 3) * 1000);
    assert!(gap <= brain.cfg.formation_spacing() + 0.01, "{gap} m apart");
}

#[test]
fn a_front_keeps_its_highest_two_searching() {
    let reserve = |sysid: u8| {
        let mut brain = test_brain(sysid, square(60.0), 1);
        let others: std::vec::Vec<u8> = (1..=8).filter(|&m| m != sysid).collect();
        in_formation(&mut brain, &others, &[], 0, 3);
        brain.in_reserve(4_000)
    };
    assert!(reserve(8) && reserve(7));
    assert!(!reserve(6) && !reserve(1));
}

#[test]
fn a_front_goes_by_whom_it_hears_once_gathered() {
    // Drone 4 goes quiet (out of range on a mouse, say) without drone 1 ever hearing it
    // busy: drones 2 and 3, which drone 1 hears, may have. While the front is still
    // gathering, it keeps its place (it may just be on its way); once gathered, the front is
    // the drones heard free, as everyone hearing them takes it to be.
    let busy: std::vec::Vec<u8> = (5..=12).collect();
    let mut brain = test_brain(1, square(60.0), 1);
    in_formation(&mut brain, &[2, 3, 4], &busy, 0, 3);
    in_formation(&mut brain, &[2, 3], &busy, 3, 10);
    assert_eq!(brain.front_places(), &[1, 2, 3, 4], "gathering");
    let gathered = (brain.front.unwrap().start_ms + ASSEMBLE_MS) / 1000;
    in_formation(&mut brain, &[2, 3], &busy, gathered, 2);
    assert_eq!(brain.front_places(), &[1, 2, 3]);
}

#[test]
fn a_drone_out_of_contact_keeps_its_place_in_the_front() {
    // Heard nobody for 20 s: it flies its place on, not back to where it last heard them.
    let mut brain = test_brain(1, square(60.0), 1);
    in_formation(&mut brain, &[2, 3, 4], &[], 0, 40);
    let mut pos = brain.slot_of(1, 41_000).unwrap();
    for t in 0..400 {
        let now = 41_000 + 50 * t;
        pos = pos + brain.on_tick(&reading(now, pos, None), &[]).velocity * 0.05;
    }
    assert!(brain.isolated(61_000));
    let off = (brain.slot_of(1, 61_000).unwrap() - pos).length();
    assert!(off < 2.0, "{off} m off its place");
}

/// Where each of 12 drones on a 130 x 30 m field got to along it (x, m): the least and the
/// most, over the first `secs`.
fn sweep_reach(fleet: u16, secs: u32) -> std::vec::Vec<(f32, f32)> {
    let half = Vec2::new(65.0, 15.0);
    let cfg = BrainConfig { field_half: half, home: Vec2::new(2.0 - half.x, 2.0 - half.y), fleet, ..BrainConfig::SWARM }.with_mode(SwarmMode::Autonomous);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| cfg.home + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let mut reach = std::vec![(f32::MAX, f32::MIN); 12];
    fly_swarm_moving(cfg, 1, &starts, |_| std::vec![], 8.0, secs * 20, None, |_, drones| {
        for (r, (_, p)) in reach.iter_mut().zip(drones) {
            *r = (r.0.min(p.x), r.1.max(p.x));
        }
        false
    });
    reach
}

#[test]
fn drones_launched_later_get_stretches_of_their_own() {
    // The mission started with 6 (one stretch on 130 x 30 m); 6 more were launched since.
    let reach = sweep_reach(6, 150);
    let (west, east): (std::vec::Vec<_>, std::vec::Vec<_>) = (0..12).partition(|&i| i < 6);
    assert!(west.iter().all(|&i| reach[i].1 < 8.0) && east.iter().all(|&i| reach[i].1 > 58.0), "{reach:?}");
}

#[test]
fn each_group_sweeps_its_own_stretch_end_to_end() {
    // 130 x 30 m, 12 drones: two stretches of 64 m, six drones each, whoever hears whom.
    let reach = sweep_reach(12, 150);
    let (west, east): (std::vec::Vec<_>, std::vec::Vec<_>) = (0..12).partition(|&i| i < 6);
    // Drones 1 to 6 (the first front) from the west end to the middle; 7 to 12 the rest.
    assert!(west.iter().all(|&i| reach[i].1 > -8.0 && reach[i].1 < 8.0), "{reach:?}");
    assert!(east.iter().all(|&i| reach[i].1 > 58.0), "{reach:?}");
}

#[test]
fn a_long_strip_is_swept_by_groups_side_by_side_not_one_crowded_line() {
    // 220 x 30 m: six drones span its width, so 18 make three groups, a stretch each.
    let cfg = strip();
    let plan = cfg.plan(1);
    assert_eq!((plan.sweep, plan.rings, plan.relays, plan.total), (18, 2, 0, 20), "{plan:?}");
    assert!(plan.pass_s < 60.0, "{plan:?}");
    for t_ms in (0..120_000).step_by(1_000) {
        let slots: std::vec::Vec<Vec2> = (0..18).map(|r| cfg.formation_slot(r, 18, t_ms).unwrap()).collect();
        let closest = (0..18).flat_map(|a| (a + 1..18).map(move |b| (a, b))).map(|(a, b)| (slots[a] - slots[b]).length()).fold(f32::MAX, f32::min);
        assert!(closest > 3.0, "slots {closest} m apart at {t_ms} ms");
        assert!(slots.iter().all(|p| p.x.abs() <= 110.0 && p.y.abs() <= 15.0), "off the field at {t_ms} ms");
    }
    // Three fronts of six (by system ID), each in its own third of the strip.
    let xs: std::vec::Vec<f32> = [0, 6, 12].iter().map(|&r| cfg.formation_slot(r, 18, 0).unwrap().x).collect();
    assert!(xs[0] < -36.0 && xs[1].abs() < 37.0 && xs[2] > 36.0, "{xs:?}");
}

/// Seconds until 22 drones from the base on the 220 x 30 m strip have found three mice spread
/// along it.
fn strip_search(seed: u32) -> f32 {
    let cfg = strip();
    let base = cfg.home;
    let starts: std::vec::Vec<Vec2> = (0..22).map(|i| base + Vec2::new((i % 6) as f32 * 1.2, (i / 6) as f32 * 1.2)).collect();
    let mice = [Vec2::new(-40.0, 8.0), Vec2::new(30.0, -9.0), Vec2::new(95.0, 3.0)];
    let mut found = [false; 3];
    let mut when = None;
    fly_swarm_moving(cfg, seed, &starts, |_| mice.to_vec(), 8.0, 6_000, None, |tick, drones| {
        for (k, m) in mice.iter().enumerate() {
            found[k] |= drones.iter().any(|(_, p)| (*p - *m).length() <= 2.5);
        }
        if when.is_none() && found.iter().all(|&f| f) {
            when = Some(tick as f32 * 0.05);
        }
        when.is_some()
    });
    when.unwrap_or(300.0)
}

/// Not a pass/fail test: how long the strip takes to search.
/// `cargo test --release --features std strip_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn strip_survey() {
    let times: std::vec::Vec<f32> = (1..=6).map(strip_search).collect();
    std::println!("found all 3 mice at (s): {times:?}, mean {:.0}", times.iter().sum::<f32>() / times.len() as f32);
}
