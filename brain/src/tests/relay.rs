//! Relay chains back to the base, and the leash.

use super::*;

/// Drones in a line 6 m apart (radio reaches 8 m, so only to the next one), with the ground
/// station next to the first. Frames hop one link per tick.
pub(super) fn relay_chain(len: usize, ticks: u32, gcs_sends: impl Fn(u32) -> std::vec::Vec<(MavHeader, MavMessage)>) -> (std::vec::Vec<Brain>, std::vec::Vec<MavMessage>) {
    let positions: std::vec::Vec<Vec2> = (0..len).map(|i| Vec2::new(-13.0 + 6.0 * i as f32, -13.0)).collect();
    let gcs_pos = Vec2::new(-14.0, -13.0);
    let mut brains: std::vec::Vec<Brain> = (0..len).map(|i| Brain::new(i as u8 + 1, cfg(), i as u32 + 1, test_keys(i as u8 + 1))).collect();
    // (where from, who sent it, the frame), as in `fly_swarm_moving`.
    let mut air: std::vec::Vec<(Vec2, u8, (MavHeader, MavMessage))> = std::vec::Vec::new();
    let mut gcs_heard = std::vec::Vec::new();
    for tick in 0..ticks {
        let now = 1_000 + tick * 50;
        for f in gcs_sends(tick) {
            air.push((gcs_pos, GCS_SYSID, f));
        }
        let mut next = std::vec::Vec::new();
        for (i, brain) in brains.iter_mut().enumerate() {
            let inbox: std::vec::Vec<_> = air
                .iter()
                .filter(|(from, sender, _)| *sender != brain.sysid() && (*from - positions[i]).length() <= 8.0)
                .map(|(_, _, f)| f.clone())
                .collect();
            brain.on_tick(&reading(now, positions[i], None), &inbox);
            let own = MavHeader { system_id: brain.sysid(), component_id: COMPONENT_ID, sequence: 0 };
            for (relayed, m) in brain.outgoing_frames(now).into_iter().flatten() {
                if (positions[i] - gcs_pos).length() <= 8.0 {
                    gcs_heard.push(m.clone());
                }
                next.push((positions[i], brain.sysid(), (relayed.unwrap_or(own), m)));
            }
        }
        air = next;
    }
    (brains, gcs_heard)
}

#[test]
fn commands_and_acknowledgements_cross_the_swarm_hop_by_hop() {
    // Drone 5 is 25 m from the ground station, which hears drones 1 and 2 directly.
    let (brains, gcs_heard) = relay_chain(5, 40, |tick| {
        if tick == 0 { std::vec![gcs_command(5, MavCmd::MAV_CMD_NAV_LOITER_UNLIM, [0.0; 7])] } else { std::vec![] }
    });
    assert_eq!(brains[4].order(), Some(Order::Hold(Vec2::new(11.0, -13.0))));
    assert!(brains[..4].iter().all(|b| b.order().is_none()), "only drone 5 obeys");
    let ack = gcs_heard.iter().find_map(|m| match m {
        MavMessage::SWARM_DRONE_STATUS(st) if st.sysid == 5 && st.ack_command == MavCmd::MAV_CMD_NAV_LOITER_UNLIM as u16 => Some(st.clone()),
        _ => None,
    });
    let ack = ack.expect("drone 5's acknowledgement reached the ground station");
    assert_eq!(ack.hops, 3, "relayed by 4, 3 and 2");
}

#[test]
fn a_busy_responder_reports_a_new_target_passing_by() {
    let mut brain = test_brain(1, cfg(), 1);
    let first = Vec2::new(2.0, 0.0);
    // On the ring around the first mouse, which it can see.
    brain.on_tick(&reading(1_000, Vec2::ZERO, Some(first)), &[]);
    let on = brain.assigned_target().expect("responding to the first");
    // A second mouse runs past, closer than the first.
    let second = Vec2::new(-1.0, 0.5);
    brain.on_tick(&seeing(1_050, &[first, second]), &[]);
    let sent = sightings_sent(&mut brain, 1_050);
    assert_eq!(sent.len(), 2, "tells everyone about both");
    std::println!("assigned before {on:04x}, after {:?}", brain.assigned_target().map(|a| std::format!("{a:04x}")));
    assert_eq!(brain.assigned_target(), Some(on), "keeps to its own target");
}

#[test]
fn routes_to_the_base_hop_by_hop() {
    // Hears the ground station itself: one hop.
    let mut near = test_brain(1, cfg(), 1);
    near.on_tick(&reading(1_000, Vec2::new(-12.0, -12.0), None), &[gcs_heartbeat()]);
    assert_eq!(near.route, Some((0, 0)), "fresh news, straight from the ground station");
    // Hears only that drone: two hops, and that drone is its parent.
    let mut far = test_brain(2, cfg(), 2);
    // Drone 1 announces news 0.3 s old, no parent (it hears the ground station itself).
    let parent_hb = heartbeat_raw(1, 3 << 16);
    far.on_tick(&reading(1_000, Vec2::new(-7.0, -12.0), None), &[parent_hb, position(1, Vec2::new(-12.0, -12.0), Vec2::ZERO)]);
    assert_eq!(far.route, Some((300, 1)), "through drone 1, news 0.3 s old");
    // A second without hearing from drone 1: the news has aged with it.
    far.on_tick(&reading(2_000, Vec2::new(-7.0, -12.0), None), &[]);
    assert_eq!(far.route, Some((1_300, 1)));
    // And past the timeout it's gone.
    far.on_tick(&reading(1_000 + LINK_TIMEOUT_MS, Vec2::new(-7.0, -12.0), None), &[]);
    assert_eq!(far.route, None);
}

pub(super) fn heartbeat_raw(sender: u8, custom_mode: u32) -> (MavHeader, MavMessage) {
    let (header, mut msg) = heartbeat(sender, 0);
    if let MavMessage::HEARTBEAT(hb) = &mut msg {
        hb.custom_mode = custom_mode;
    }
    (header, msg)
}

#[test]
fn the_leash_stops_a_drone_at_the_edge_of_its_parents_reach() {
    let mut brain = test_brain(1, cfg(), 1);
    // One hop: the base at (-13, -13) is its parent; it's 6.3 m out, heading further out.
    let pos = Vec2::new(-13.0 + 6.3, -13.0);
    let report_far = sighting(2, Vec2::new(5.0, -13.0), 0);
    brain.on_tick(&reading(1_000, pos, None), &[gcs_heartbeat(), report_far.clone()]);
    let out = brain.on_tick(&reading(1_050, pos, None), &[gcs_heartbeat(), report_far]);
    assert!(out.velocity.x < 0.3, "kept flying away from the base: {:?}", out.velocity);
    // Heading back toward the base is never held back.
    assert!(brain.keep_on_leash(pos, Vec2::new(-2.0, 0.0), 1_050).x <= -1.99);
}

#[test]
fn a_drone_that_loses_the_link_heads_home_then_gives_up() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::new(-12.0, -12.0), None), &[gcs_heartbeat()]);
    // Ten seconds later it's heard nobody for a while (it drifted, or everyone left).
    let out = brain.on_tick(&reading(11_000, Vec2::new(0.0, 0.0), None), &[]);
    assert!(out.velocity.x < -1.0 && out.velocity.y < -1.0, "not heading home: {:?}", out.velocity);
    // Past the timeout the ground station is presumed gone: back to searching freely.
    brain.on_tick(&reading(40_000, Vec2::new(0.0, 0.0), None), &[]);
    assert!(!brain.link_lost(40_000));
}

#[test]
fn without_a_ground_station_nothing_is_leashed() {
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::new(5.0, 5.0), None), &[]);
    assert!(!brain.leashed());
    assert!(!brain.link_lost(1_000));
}

/// Share of drone-ticks (after launch) with a radio path to the ground station, and when
/// all targets were found, for a swarm launched from the base corner.
pub(super) fn link_and_search(keep_link: bool, seed: u32) -> (f32, Option<f32>) {
    link_and_search_with(keep_link, false, seed)
}

pub(super) fn link_and_search_with(keep_link: bool, relay_chains: bool, seed: u32) -> (f32, Option<f32>) {
    let cfg = BrainConfig { keep_link, relay_chains, ..RINGS_OF_FOUR };
    let base = Vec2::new(-13.0, -13.0);
    let targets = [Vec2::new(-4.0, 2.0), Vec2::new(6.0, -8.0)];
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let (mut connected, mut total) = (0u32, 0u32);
    let mut found = [false; 2];
    let mut all_found = None;
    // Five minutes: a target far from the base needs a long chain of relays to reach.
    fly_swarm_full(cfg, seed, &starts, &targets, 8.0, 6_000, Some(base), |tick, drones| {
        for (k, t) in targets.iter().enumerate() {
            found[k] |= drones.iter().any(|(_, p)| (*p - *t).length() <= 2.5);
        }
        if all_found.is_none() && found.iter().all(|&f| f) {
            all_found = Some(tick as f32 * 0.05);
        }
        // Who can the base reach, hop by hop?
        let mut reached = std::vec![false; drones.len()];
        let mut frontier = std::vec![base];
        while let Some(p) = frontier.pop() {
            for (i, (_, q)) in drones.iter().enumerate() {
                if !reached[i] && (*q - p).length() <= 8.0 {
                    reached[i] = true;
                    frontier.push(*q);
                }
            }
        }
        if tick > 200 {
            connected += reached.iter().filter(|&&r| r).count() as u32;
            total += drones.len() as u32;
        }
        false
    });
    (connected as f32 / total as f32, all_found)
}

/// Not a pass/fail test: how keeping far rings in contact with relay chains costs the
/// search, over 8 runs. `cargo test --release relay_chains_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn relay_chains_survey() {
    let off: std::vec::Vec<_> = (1..=8).map(|s| link_and_search_with(true, false, s)).collect();
    let on: std::vec::Vec<_> = (1..=8).map(|s| link_and_search_with(true, true, s)).collect();
    std::println!("(connected share, all found at s) without chains: {off:?}");
    std::println!("with chains:                                     {on:?}");
}

#[test]
fn keeping_the_link_keeps_the_swarm_in_contact_and_still_finds_targets() {
    let free: std::vec::Vec<_> = (1..=4).map(|s| link_and_search(false, s)).collect();
    let kept: std::vec::Vec<_> = (1..=8).map(|s| link_and_search(true, s)).collect();
    std::println!("without keep_link (connected share, all found at s): {free:?}");
    std::println!("with keep_link:                                       {kept:?}");
    // Not 100%: a responder may follow its target out of contact on purpose (a target beats
    // the link), until a free drone comes over to relay for it. How much varies a lot with
    // the seed (0.69 to 1.0), so the mean of eight: 0.83 to 0.84 however sightings are
    // relayed.
    let mean = kept.iter().map(|&(c, _)| c).sum::<f32>() / kept.len() as f32;
    assert!(mean > 0.75 && kept.iter().all(|&(c, _)| c > 0.6), "lost contact with keep_link: mean {mean}, {kept:?}");
    assert!(kept.iter().all(|&(_, f)| f.is_some()), "a target was never found with keep_link: {kept:?}");
}

#[test]
fn split_horizon_two_drones_dont_route_through_each_other() {
    // Drone 2 says its route goes through us (drone 1); we hear no one else.
    let mut brain = test_brain(1, cfg(), 1);
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[heartbeat_raw(2, (1 << 24) | (2 << 16)), position(2, Vec2::new(3.0, 0.0), Vec2::ZERO)]);
    assert_eq!(brain.route, None, "drone 2's news came through us");
    assert_eq!(brain.route, None);
}

/// A mouse found near the base that then runs to the far corner of the field: how much of
/// the time after it's first reached does some drone stay within 4 m of it?
pub(super) fn follow_a_runaway(keep_link: bool, seed: u32) -> f32 {
    let cfg = BrainConfig { keep_link, ..RINGS_OF_FOUR };
    let base = Vec2::new(-13.0, -13.0);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    // Waits at (-6,-6) for 10 s, then runs to (13,13) at 0.8 m/s (about 34 s), then sits there.
    let mouse = |tick: u32| {
        let t = tick as f32 * 0.05;
        let from = Vec2::new(-6.0, -6.0);
        let to = Vec2::new(13.0, 13.0);
        let along = ((t - 10.0) * 0.8 / (to - from).length()).clamp(0.0, 1.0);
        std::vec![from + (to - from) * along]
    };
    let (mut reached, mut held, mut total) = (false, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, mouse, 8.0, 2_400, Some(base), |tick, drones| {
        let m = mouse(tick)[0];
        let near = drones.iter().any(|(_, p)| (*p - m).length() < 4.0);
        reached |= near;
        if reached {
            total += 1;
            held += near as u32;
        }
        false
    });
    held as f32 / total.max(1) as f32
}

/// Like `follow_a_runaway` (keeping the link), returning (share of the time from when the
/// mouse is first reached that a drone is within 4 m of it, share of that time that some
/// drone on it has a route to the ground station).
pub(super) fn runaway_chain(relay_chains: bool, seed: u32) -> (f32, f32) {
    // Connected mode: it keeps everyone in contact (see `SwarmMode`).
    let cfg = if relay_chains { RINGS_OF_FOUR.with_mode(SwarmMode::Connected) } else { RINGS_OF_FOUR };
    let base = Vec2::new(-13.0, -13.0);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let mouse = |tick: u32| {
        let t = tick as f32 * 0.05;
        let from = Vec2::new(-6.0, -6.0);
        let to = Vec2::new(13.0, 13.0);
        let along = ((t - 10.0) * 0.8 / (to - from).length()).clamp(0.0, 1.0);
        std::vec![from + (to - from) * along]
    };
    let (mut reached, mut held, mut linked, mut total) = (false, 0u32, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, mouse, 8.0, 2_400, Some(base), |tick, drones| {
        let m = mouse(tick)[0];
        let near = drones.iter().any(|(_, p)| (*p - m).length() < 4.0);
        reached |= near;
        if reached {
            total += 1;
            held += near as u32;
            linked += drones.iter().any(|(b, _)| b.assigned_target().is_some() && b.route.is_some()) as u32;
        }
        false
    });
    (held as f32 / total.max(1) as f32, linked as f32 / total.max(1) as f32)
}

#[test]
fn a_ring_chasing_a_runaway_stays_in_contact_through_relays() {
    let without: std::vec::Vec<(f32, f32)> = (1..=4).map(|s| runaway_chain(false, s)).collect();
    let results: std::vec::Vec<(f32, f32)> = (1..=4).map(|s| runaway_chain(true, s)).collect();
    std::println!("(share of time watched, share of time the ring is in contact) without chains: {without:?}, with: {results:?}");
    // Without chains the ring is in contact about a quarter of the time. The gaps left are
    // mostly the mouse outrunning the chain as it grows.
    let mean = results.iter().map(|r| r.1).sum::<f32>() / results.len() as f32;
    assert!(results.iter().all(|&(held, linked)| held > 0.9 && linked > 0.65) && mean > 0.8, "{results:?}");
}

/// Twelve drones from the base corner, one mouse near the middle, a second one appearing at
/// 2 minutes in the far corner. Returns (share of the field within 5 m of some drone, on
/// average from 60 s on; share of drone-ticks in contact with the base; seconds from the
/// second mouse appearing to a drone within sensor range of it).
/// None: neither mode (keeping the link, with no relay chains).
pub(super) fn spread(mode: Option<SwarmMode>, seed: u32) -> (f32, f32, Option<f32>) {
    // Within the reach of a ray from the base. One past it, in the far corner (10, 9), the
    // connected swarm found only when a drone left over from the rays happened to search
    // there: the same run either always or never, whatever the seed.
    let second = Vec2::new(5.0, 12.0);
    let cfg = match mode {
        Some(mode) => RINGS_OF_FOUR.with_mode(mode),
        None => RINGS_OF_FOUR,
    };
    let base = Vec2::new(-13.0, -13.0);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let second_at = 2_400;
    let mice = move |tick: u32| if tick < second_at { std::vec![Vec2::new(-2.0, 0.0)] } else { std::vec![Vec2::new(-2.0, 0.0), second] };
    let (mut covered, mut samples, mut connected, mut total) = (0.0f32, 0u32, 0u32, 0u32);
    let mut found = None;
    fly_swarm_moving(cfg, seed, &starts, mice, 8.0, 7_200, Some(base), |tick, drones| {
        if tick >= second_at && found.is_none() && drones.iter().any(|(_, p)| (*p - second).length() <= 2.5) {
            found = Some((tick - second_at) as f32 * 0.05);
        }
        if tick < 1_200 || tick % 20 != 0 {
            return false;
        }
        let mut hits = 0;
        for x in 0..30 {
            for y in 0..30 {
                let q = Vec2::new(-14.5 + x as f32, -14.5 + y as f32);
                hits += drones.iter().any(|(_, p)| (*p - q).length() <= 5.0) as u32;
            }
        }
        covered += hits as f32 / 900.0;
        samples += 1;
        let mut reached = std::vec![false; drones.len()];
        let mut frontier = std::vec![base];
        while let Some(p) = frontier.pop() {
            for (i, (_, q)) in drones.iter().enumerate() {
                if !reached[i] && (*q - p).length() <= 8.0 {
                    reached[i] = true;
                    frontier.push(*q);
                }
            }
        }
        connected += reached.iter().filter(|&&r| r).count() as u32;
        total += drones.len() as u32;
        false
    });
    (covered / samples.max(1) as f32, connected as f32 / total.max(1) as f32, found)
}

/// Twelve mice spread over the field, still, and twelve drones setting out from the base
/// corner. Returns how many mice have a drone on them (within 2.5 m), on average over the last
/// minute of five.
pub(super) fn as_many_mice_as_drones(adaptive: bool, seed: u32) -> f32 {
    let cfg = BrainConfig { adaptive_responders: adaptive, ..RINGS_OF_FOUR };
    let mice: std::vec::Vec<Vec2> = (0..12).map(|i| Vec2::new(-11.0 + (i % 4) as f32 * 7.3, -10.0 + (i / 4) as f32 * 10.0)).collect();
    // All from the base corner.
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| Vec2::new(-13.0 + (i % 4) as f32 * 1.5, -13.0 + (i / 4) as f32 * 1.5)).collect();
    let (mut watched, mut samples) = (0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, |_| mice.clone(), 8.0, 6_000, None, |tick, drones| {
        if tick >= 4_800 {
            samples += 1;
            watched += mice.iter().filter(|m| drones.iter().any(|(_, p)| (*p - **m).length() <= 2.5)).count() as u32;
        }
        false
    });
    watched as f32 / samples.max(1) as f32
}

#[test]
fn adaptive_responders_spread_one_drone_to_a_mouse() {
    let fixed: std::vec::Vec<f32> = (1..=3).map(|s| as_many_mice_as_drones(false, s)).collect();
    let adaptive: std::vec::Vec<f32> = (1..=3).map(|s| as_many_mice_as_drones(true, s)).collect();
    std::println!("mice of 12 with a drone on them — fixed 4 per mouse: {fixed:?}, adaptive: {adaptive:?}");
    // Fixed, the first three mice found take all twelve drones (rings of 4) and nobody
    // searches for the rest; adaptive, the rings shrink toward one drone a mouse.
    assert!(adaptive.iter().all(|&a| a >= 8.0) && fixed.iter().all(|&f| f <= 5.0), "{fixed:?} {adaptive:?}");
}
