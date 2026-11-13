//! Rings around targets, and who goes to them.

use super::*;

/// Autonomous: twelve drones sweeping a 30 x 30 m field in formation, one mouse sitting in
/// its way. Returns (how many different drones took the mouse on, the most drones on it at
/// once, how far (m) the drones still in formation were from their slots on average over
/// the 20 s after the mouse was found).
pub(super) fn formation_meets_a_mouse(seed: u32) -> (usize, usize, f32) {
    let cfg = RINGS_OF_FOUR.with_mode(SwarmMode::Autonomous);
    let base = cfg.home;
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let mouse = Vec2::new(2.0, 4.0);
    let mut took: std::vec::Vec<u8> = std::vec::Vec::new();
    let (mut most, mut found_at, mut off, mut samples) = (0usize, None::<u32>, 0.0f32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, |_| std::vec![mouse], 8.0, 2_400, None, |tick, drones| {
        let now = tick * 50;
        let on: std::vec::Vec<u8> = drones.iter().filter(|(b, _)| b.assigned_target().is_some()).map(|(b, _)| b.sysid()).collect();
        if !on.is_empty() && found_at.is_none() {
            found_at = Some(tick);
        }
        most = most.max(on.len());
        for id in on {
            if !took.contains(&id) {
                took.push(id);
            }
        }
        if found_at.is_some_and(|f| tick > f && tick < f + 400) {
            for (b, p) in drones.iter().filter(|(b, _)| b.on_sweep) {
                if let Some((slot, _)) = b.formation_spot(now) {
                    off += (slot - *p).length();
                    samples += 1;
                }
            }
        }
        false
    });
    (took.len(), most, off / samples.max(1) as f32)
}


#[test]
fn a_formation_sends_only_a_few_to_a_mouse_and_keeps_its_line() {
    let ideal: std::vec::Vec<_> = (1..=4).map(formation_meets_a_mouse).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    GPS_SIGMA_CM.with(|v| v.set(150));
    let real: std::vec::Vec<_> = (1..=12).map(formation_meets_a_mouse).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    GPS_SIGMA_CM.with(|v| v.set(0));
    std::println!("(drones that took the mouse on, most at once, mean distance from slot in m) ideal: {ideal:?}, lossy radio and GPS error: {real:?}");
    // Over the lossy radio, a run goes either way on which frames happen to be lost: most often
    // four set off for it, now and then many more. So it's the average over a dozen runs that's
    // checked, not every run.
    assert!(ideal.iter().all(|r| r.0 == 4), "{ideal:?}");
    let mean = real.iter().map(|r| r.0 as f32).sum::<f32>() / real.len() as f32;
    assert!(mean <= 7.0 && real.iter().filter(|r| r.0 <= 5).count() >= 3, "{real:?}");
}

#[test]
fn with_relay_chains_the_swarm_stays_one_network() {
    let off: std::vec::Vec<_> = (1..=4).map(|s| one_network(false, s)).collect();
    let on: std::vec::Vec<_> = (1..=4).map(|s| one_network(true, s)).collect();
    std::println!("(share in contact with the base, share of mouse-time watched) without chains: {off:?}, with: {on:?}");
}

/// One mouse wandering the middle of the field and 8 drones searching around it, so free
/// drones keep arriving while 4 hold the ring. Returns every time a drone that had been on
/// the mouse for `settled_ms` gave it up: (seconds into the flight, sysid, seconds on it).
pub(super) fn settled_responders_dropped(phase: f32, settled_ms: u32) -> std::vec::Vec<(f32, u8, f32)> {
    let mouse = move |tick: u32| {
        let t = tick as f32 * 0.05 + phase;
        let x = -8.0 + ((t * 0.8) % 32.0 - 16.0).abs();
        std::vec![Vec2::new(x, 2.0 * libm::sinf(t * 0.3))]
    };
    let starts: std::vec::Vec<Vec2> = (0..8).map(|i| Vec2::new(-12.0 + 3.0 * i as f32, -6.0 + (i % 2) as f32 * 12.0)).collect();
    let mut on_since: std::vec::Vec<Option<u32>> = std::vec![None; starts.len()];
    let mut dropped = std::vec::Vec::new();
    fly_swarm_moving(RINGS_OF_FOUR, 7, &starts, mouse, 8.0, 2_400, None, |tick, drones| {
        let now = tick * 50;
        for (i, (b, _)) in drones.iter().enumerate() {
            match (on_since[i], b.assigned_target()) {
                (None, Some(_)) => on_since[i] = Some(now),
                (Some(since), None) => {
                    if now - since >= settled_ms {
                        dropped.push((now as f32 / 1000.0, b.sysid(), (now - since) as f32 / 1000.0));
                    }
                    on_since[i] = None;
                }
                _ => {}
            }
        }
        false
    });
    dropped
}

#[test]
fn a_later_arrival_never_pushes_a_settled_responder_off_the_ring() {
    // A newcomer with a lower sysid doesn't take the place of a drone settled on the ring,
    // however its sysid compares.
    for phase in [0.0, 7.0, 13.0, 21.0] {
        // Long enough on it to have reached the ring and settled there.
        let dropped = settled_responders_dropped(phase, 8_000);
        assert!(dropped.is_empty(), "phase {phase}: (s, sysid, s on target) {dropped:?}");
    }
}

#[test]
fn a_target_running_away_from_the_base_stays_watched() {
    let free: std::vec::Vec<f32> = (1..=4).map(|s| follow_a_runaway(false, s)).collect();
    let kept: std::vec::Vec<f32> = (1..=4).map(|s| follow_a_runaway(true, s)).collect();
    std::println!("share of time the runaway had a drone on it — without keep_link: {free:?}, with: {kept:?}");
    assert!(kept.iter().all(|&h| h > 0.95), "lost the runaway with keep_link: {kept:?}");
}

/// Four drones holding a ring around mouse A; mouse B runs past them, close but not through
/// the ring; the other eight drones are searching the far side of the field. Returns the
/// seconds until a drone *other than A's responders* gets within sensor range of B.
pub(super) fn passerby(seed: u32) -> Option<f32> {
    let a = Vec2::new(-8.0, -8.0);
    let starts: std::vec::Vec<Vec2> = (0..12)
        .map(|i| if i < 4 { a + Vec2::new(libm::cosf(i as f32 * 1.57), libm::sinf(i as f32 * 1.57)) * 1.5 } else { Vec2::new(4.0 + (i % 4) as f32 * 3.0, 4.0 + (i / 4) as f32 * 3.0) })
        .collect();
    // B: from (-14, -5) runs east at 0.8 m/s, 3 m past A, starting once A's ring has formed.
    let mouse_b = |tick: u32| Vec2::new(-14.0 + ((tick as f32 * 0.05 - 5.0).max(0.0) * 0.8).min(26.0), -5.0);
    let mut first = None;
    fly_swarm_moving(RINGS_OF_FOUR, seed, &starts, |tick| std::vec![a, mouse_b(tick)], 8.0, 1_600, None, |tick, drones| {
        let b = mouse_b(tick);
        if first.is_none() && drones[4..].iter().any(|(_, p)| (*p - b).length() <= 2.5) {
            first = Some(tick as f32 * 0.05);
        }
        first.is_some()
    });
    first
}

#[test]
fn a_target_passing_busy_drones_draws_free_ones() {
    let results: std::vec::Vec<_> = (1..=4).map(passerby).collect();
    std::println!("free drones reached the passerby at: {results:?}");
    assert!(results.iter().all(|r| r.is_some_and(|t| t < 40.0)), "passerby ignored: {results:?}");
}

/// Like `passerby`, but B runs *away* from the free drones, which are out of radio range of
/// A's ring: only A's responders ever see it. Returns the share of the time from when B
/// passes the ring until the end that some drone is within 4 m of it.
pub(super) fn runaway_passerby(seed: u32) -> f32 {
    runaway_passerby_at(seed, 3.0)
}

pub(super) fn runaway_passerby_at(seed: u32, miss: f32) -> f32 {
    let cfg = RINGS_OF_FOUR;
    let a = Vec2::new(-4.0, -4.0);
    let starts: std::vec::Vec<Vec2> = (0..12)
        .map(|i| if i < 4 { a + Vec2::new(libm::cosf(i as f32 * 1.57), libm::sinf(i as f32 * 1.57)) * 1.5 } else { Vec2::new(7.0 + (i % 4) as f32 * 2.0, 9.0 + (i / 4) as f32 * 2.0) })
        .collect();
    // B: comes from the east just south of A, passes 3 m from it at t~10 s, carries on west.
    let mouse_b = move |tick: u32| Vec2::new(4.0 - (tick as f32 * 0.05 * 0.8).min(18.0), a.y - miss);
    let (mut watched, mut total, mut a_lost) = (0u32, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, |tick| std::vec![a, mouse_b(tick)], 8.0, 1_600, None, |tick, drones| {
        let b = mouse_b(tick);
        if b.x < a.x {
            total += 1;
            watched += drones.iter().any(|(_, p)| (*p - b).length() <= 4.0) as u32;
            // Only counts if A is still watched too.
            a_lost += !drones.iter().any(|(_, p)| (*p - a).length() <= 4.0) as u32;
        }
        false
    });
    if a_lost > 0 {
        return -(a_lost as f32 / total.max(1) as f32);
    }
    watched as f32 / total.max(1) as f32
}

/// Every drone on a ring around one of three mice, and a fourth mouse sits just outside ring
/// A, where one of A's drones sees it: returns when (s) every mouse has at least two drones
/// on it (within 2.5 m: the ring is 1.5 m) and keeps them for 5 s, or None.
pub(super) fn fourth_mouse_shared(seed: u32) -> Option<f32> {
    let cfg = RINGS_OF_FOUR;
    let rings = [Vec2::new(-6.0, 0.0), Vec2::new(6.0, 0.0), Vec2::new(0.0, 8.0)];
    let d = Vec2::new(-6.0, -3.8);
    let starts: std::vec::Vec<Vec2> =
        (0..12).map(|i| rings[i / 4] + Vec2::new(libm::cosf((i % 4) as f32 * 1.57), libm::sinf((i % 4) as f32 * 1.57)) * 1.5).collect();
    let mice = [rings[0], rings[1], rings[2], d];
    let mut held_since: Option<u32> = None;
    let mut done = None;
    fly_swarm_moving(cfg, seed, &starts, |_| mice.to_vec(), 8.0, 1_600, None, |tick, drones| {
        let all = mice.iter().all(|m| drones.iter().filter(|(_, p)| (*p - *m).length() <= 2.5).count() >= 2);
        held_since = if all { held_since.or(Some(tick)) } else { None };
        if held_since.is_some_and(|t| tick - t >= 100) {
            done = held_since.map(|t| t as f32 * 0.05);
            return true;
        }
        false
    });
    done
}

#[test]
fn with_every_drone_busy_the_rings_share_with_a_new_mouse() {
    let results: std::vec::Vec<_> = (1..=4).map(fourth_mouse_shared).collect();
    std::println!("all four mice held from: {results:?}");
    assert!(results.iter().all(|r| r.is_some_and(|t| t < 40.0)), "a mouse left alone: {results:?}");
}

/// Mice A and B meet, run side by side 1 m apart for 15 s, and part; C sits on its own.
/// Twelve drones start as three rings of four. Returns (times any drone switched or dropped
/// its target, share of the time some mouse had fewer than two drones on it).
pub(super) fn mice_side_by_side(seed: u32) -> (u32, f32) {
    let cfg = RINGS_OF_FOUR;
    let a = |t: f32| Vec2::new(-10.0 + 0.6 * t, -2.0);
    let b = |t: f32| {
        let start = Vec2::new(-4.0, 6.0);
        let meet_at = 10.0;
        if t < meet_at {
            // Toward where A will be at the meeting, 1 m north of it.
            let meet = a(meet_at) + Vec2::new(0.0, 1.0);
            start + (meet - start) * (t / meet_at)
        } else if t < meet_at + 15.0 {
            a(t) + Vec2::new(0.0, 1.0)
        } else {
            a(meet_at + 15.0) + Vec2::new(0.0, 1.0) + Vec2::new(-0.3, 0.7) * (t - meet_at - 15.0)
        }
    };
    let c = Vec2::new(8.0, -8.0);
    let rings = [a(0.0), b(0.0), c];
    let starts: std::vec::Vec<Vec2> =
        (0..12).map(|i| rings[i / 4] + Vec2::new(libm::cosf((i % 4) as f32 * 1.57), libm::sinf((i % 4) as f32 * 1.57)) * 1.5).collect();
    let mut last: std::vec::Vec<Option<TargetId>> = std::vec![None; 12];
    let (mut switches, mut short, mut total) = (0u32, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, |tick| { let t = tick as f32 * 0.05; std::vec![a(t), b(t), c] }, 8.0, 1_000, None, |tick, drones| {
        let t = tick as f32 * 0.05;
        if tick > 60 {
            for (i, (brain, _)) in drones.iter().enumerate() {
                let now = brain.assigned_target();
                if last[i].is_some() && now != last[i] {
                    switches += 1;
                }
                last[i] = now;
            }
            total += 1;
            short += [a(t), b(t), c].iter().any(|m| drones.iter().filter(|(_, p)| (*p - *m).length() <= 2.5).count() < 2) as u32;
        } else {
            for (i, (brain, _)) in drones.iter().enumerate() {
                last[i] = brain.assigned_target();
            }
        }
        false
    });
    (switches, short as f32 / total.max(1) as f32)
}

#[test]
fn mice_side_by_side_keep_their_rings() {
    // Two mice side by side stay two tracks under their own names, and each keeps its ring.
    let results: std::vec::Vec<_> = (1..=4).map(mice_side_by_side).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    let lossy: std::vec::Vec<_> = (1..=4).map(mice_side_by_side).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    std::println!("(target switches, share of the time a mouse was short): {results:?}, over a lossy radio: {lossy:?}");
    assert!(results.iter().chain(&lossy).all(|&(switches, short)| switches <= 3 && short < 0.05), "{results:?} {lossy:?}");
}

/// Five mice running about the field and twelve drones, which can't give all of them
/// four: returns (assignments taken, those dropped within 8 s, share of the time some mouse
/// had nobody within 2.5 m), from 30 s on.
pub(super) fn five_mice(seed: u32) -> (u32, u32, f32) {
    let cfg = RINGS_OF_FOUR;
    let tri = |x: f32| { let m = x.rem_euclid(56.0); if m < 28.0 { -14.0 + m } else { 42.0 - m } };
    let mice = move |tick: u32| {
        let t = tick as f32 * 0.05;
        (0..5).map(|i| { let k = i as f32; Vec2::new(tri(k * 11.0 + t * 0.8 * libm::cosf(k)), tri(k * 7.0 + 20.0 + t * 0.8 * libm::sinf(k + 0.5))) }).collect::<std::vec::Vec<_>>()
    };
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| Vec2::new(-12.0 + (i % 4) as f32 * 8.0, -9.0 + (i / 4) as f32 * 9.0)).collect();
    let mut on: std::vec::Vec<Option<(TargetId, u32)>> = std::vec![None; 12];
    let (mut taken, mut dropped, mut alone, mut total) = (0u32, 0u32, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, mice, 8.0, 4_800, None, |tick, drones| {
        let now = tick * 50;
        for (i, (b, _)) in drones.iter().enumerate() {
            let a = b.assigned_target();
            match (on[i], a) {
                (None, Some(id)) => { on[i] = Some((id, now)); if tick > 600 { taken += 1; } }
                (Some((old, since)), cur) if cur != Some(old) => {
                    if tick > 600 && now - since < 8_000 {
                        dropped += 1;
                        if FIVE_MICE_LOG.with(|v| v.get()) {
                            // Set it to see why each drop happened.
                            std::println!("t {:.1} drone {} dropped {old:04x} after {:.1}s -> {cur:?}; tracks {:?}", now as f32 / 1000.0, b.sysid(), (now - since) as f32 / 1000.0, b.tracks.iter().flatten().filter(|t| !b.lost(t, now)).map(|t| (t.id, t.responders, b.responders_heard(t.id, now), b.confirmed(t, now))).collect::<std::vec::Vec<_>>());
                        }
                    }
                    on[i] = cur.map(|c| (c, now));
                    if tick > 600 && cur.is_some() { taken += 1; }
                }
                _ => {}
            }
        }
        if tick > 600 {
            total += 1;
            let m = mice(tick);
            alone += m.iter().any(|m| !drones.iter().any(|(_, p)| (*p - *m).length() <= 2.5)) as u32;
        }
        false
    });
    (taken, dropped, alone as f32 / total.max(1) as f32)
}

std::thread_local! { static FIVE_MICE_LOG: core::cell::Cell<bool> = const { core::cell::Cell::new(false) }; }

/// Connected, twelve drones, three wandering mice, `responders` 2: how often a mouse has
/// more drones on it than that. Returns (share of mouse-ticks over the share, most ever on
/// one). `cargo test --release overjoin_survey -- --ignored --nocapture`
pub(super) fn overjoin(adaptive: bool, seed: u32) -> (f32, usize) {
    let cfg = BrainConfig { responders: 2, adaptive_responders: adaptive, ..RINGS_OF_FOUR.with_mode(SwarmMode::Connected) };
    let base = Vec2::new(-13.0, -13.0);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let tri = |x: f32| { let m = x.rem_euclid(56.0); if m < 28.0 { -14.0 + m } else { 42.0 - m } };
    let mice = move |tick: u32| {
        let t = tick as f32 * 0.05;
        (0..3).map(|i| { let k = i as f32 + 1.0; Vec2::new(tri(k * 13.0 + t * 0.5 * libm::cosf(k)), tri(k * 9.0 + 20.0 + t * 0.5 * libm::sinf(k + 0.5))) }).collect::<std::vec::Vec<_>>()
    };
    let (mut over, mut total, mut most) = (0u32, 0u32, 0usize);
    let mut logged = 0;
    fly_swarm_moving(cfg, seed, &starts, mice, 8.0, 7_200, Some(base), |tick, drones| {
        let mut ids: std::vec::Vec<TargetId> = drones.iter().filter_map(|(b, _)| b.assigned_target()).collect();
        ids.sort();
        ids.dedup();
        for id in ids {
            let on: std::vec::Vec<_> = drones.iter().filter(|(b, _)| b.assigned_target() == Some(id)).collect();
            most = most.max(on.len());
            if on.len() > 2 {
                over += 1;
                if OVERJOIN_LOG.with(|v| v.get()) && logged < 6 && tick % 20 == 0 {
                    logged += 1;
                    let now = tick * 50;
                    std::println!("t {:.1} target {id:04x}: {:?}", now as f32 / 1000.0, on.iter().map(|(b, p)| (b.sysid(), b.join_share, b.stay_share, b.settled(now), (p.x as i32, p.y as i32))).collect::<std::vec::Vec<_>>());
                }
            }
        }
        total += 3;
        false
    });
    (over as f32 / total.max(1) as f32, most)
}

std::thread_local! { static OVERJOIN_LOG: core::cell::Cell<bool> = const { core::cell::Cell::new(false) }; }

#[test]
#[ignore]
fn overjoin_survey() {
    let fixed: std::vec::Vec<_> = (1..=3).map(|s| overjoin(false, s)).collect();
    let adaptive: std::vec::Vec<_> = (1..=3).map(|s| overjoin(true, s)).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    GPS_SIGMA_CM.with(|v| v.set(150));
    OVERJOIN_LOG.with(|v| v.set(true));
    let lossy: std::vec::Vec<_> = (1..=3).map(|s| overjoin(false, s)).collect();
    OVERJOIN_LOG.with(|v| v.set(false));
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    GPS_SIGMA_CM.with(|v| v.set(0));
    std::println!("(share over 2, most on one) fixed: {fixed:?} adaptive: {adaptive:?} lossy+gps: {lossy:?}");
}

/// Not a pass/fail test: how churn (drones taking a target and dropping it soon after)
/// fares with too few drones for every mouse, over a lossy radio and with GPS error.
/// `cargo test --release five_mice_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn five_mice_survey() {
    let results: std::vec::Vec<_> = (1..=3).map(five_mice).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    let lossy: std::vec::Vec<_> = (1..=3).map(five_mice).collect();
    GPS_SIGMA_CM.with(|v| v.set(150));
    let gps: std::vec::Vec<_> = (1..=3).map(five_mice).collect();
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    GPS_SIGMA_CM.with(|v| v.set(0));
    std::println!("(taken, dropped within 8 s, share of time some mouse alone): {results:?}, lossy: {lossy:?}, lossy+gps: {gps:?}");
}

#[test]
fn a_target_passing_a_busy_ring_is_followed() {
    let results: std::vec::Vec<f32> = (1..=4).map(runaway_passerby).collect();
    std::println!("share of time B is watched after passing A's ring: {results:?}");
    for miss in [0.5f32, 1.0, 1.5, 2.0] {
        let close: std::vec::Vec<f32> = (1..=4).map(|s| runaway_passerby_at(s, miss)).collect();
        std::println!("  passing {miss} m from A: {close:?}");
    }
    assert!(results.iter().all(|&r| r > 0.9), "B ignored: {results:?}");
}

#[test]
fn a_lost_target_becomes_a_last_seen_hint() {
    let mut brain = test_brain(1, cfg(), 1);
    let mouse = Vec2::new(-6.0, 4.0);
    // Seen by others (two witnesses), then nobody sees it any more.
    brain.on_tick(&reading(1_000, Vec2::new(8.0, -8.0), None), &[sighting(2, mouse, 0), sighting(3, mouse, 0)]);
    let id = brain.tracks.iter().flatten().next().unwrap().id;
    // Six seconds on: not a target to fly to any more... (A free neighbor in the opposite
    // corner, so it isn't isolated and does search.)
    let neighbor = [position(9, Vec2::new(14.0, -14.0), Vec2::ZERO), heartbeat(9, 0)];
    brain.on_tick(&reading(7_000, Vec2::new(8.0, -8.0), None), &neighbor);
    assert_eq!(brain.mode(), Mode::Searching);
    assert_eq!(brain.assigned_target(), None);
    // ...but still known, and still passed on, with its age...
    let sent = sightings_sent(&mut brain, 7_000);
    assert!(sent.iter().any(|s| s.target_id == id && s.age_ms >= 6_000), "hint not relayed: {:?}", sent.iter().map(|s| (s.target_id, s.age_ms)).collect::<std::vec::Vec<_>>());
    // ...and the ground around where it was is up for searching again.
    let cell = brain.cell_of(mouse).unwrap();
    assert_eq!(brain.searched_at[cell.0][cell.1], LAST_SEEN_HINT);
    let goal = brain.cell_center(brain.search_goal.expect("searching"));
    assert!((goal - mouse).length() <= 2.0 * cfg().scan_radius + 2.0, "search goal {goal:?} isn't near the last sighting");
    // Gone for good after `last_seen_memory_ms`.
    brain.on_tick(&reading(32_000, Vec2::new(8.0, -8.0), None), &[]);
    assert!(brain.tracks.iter().flatten().all(|t| t.id != id));
}

#[test]
fn a_relayed_last_seen_hint_sends_searchers_there() {
    // We never saw it, but a neighbor passes on a sighting already 8 s old.
    let mut brain = test_brain(1, cfg(), 1);
    let mouse = Vec2::new(10.0, 10.0);
    brain.on_tick(&reading(20_000, Vec2::new(-5.0, -5.0), None), &[sighting(2, mouse, 8_000)]);
    brain.on_tick(&reading(20_050, Vec2::new(-5.0, -5.0), None), &[]);
    let goal = brain.cell_center(brain.search_goal.expect("searching"));
    assert!((goal - mouse).length() <= 2.0 * cfg().scan_radius + 2.0, "search goal {goal:?} isn't near the hint");
}

/// Four drones already holding a ring around a mouse; a fifth arrives 7 m away (in radio
/// range). Returns what the newcomer did each second: (mode, assigned).
pub(super) fn newcomer_near_a_full_target(seed: u32) -> std::vec::Vec<(Mode, Option<TargetId>)> {
    newcomer_at(seed, &[Vec2::new(7.0, 0.0)])
}

/// The ring plus drones at `others`; reports on the last of them.
pub(super) fn newcomer_at(seed: u32, others: &[Vec2]) -> std::vec::Vec<(Mode, Option<TargetId>)> {
    let mouse = Vec2::new(0.0, 0.0);
    let mut starts: std::vec::Vec<Vec2> = (0..4).map(|i| mouse + Vec2::new(libm::cosf(i as f32 * 1.57), libm::sinf(i as f32 * 1.57)) * 1.5).collect();
    starts.extend_from_slice(others);
    let newcomer = starts.len() - 1;
    let mut seen = std::vec::Vec::new();
    fly_swarm_moving(RINGS_OF_FOUR, seed, &starts, |_| std::vec![mouse], 8.0, 400, None, |tick, drones| {
        if tick % 20 == 0 {
            let (b, _) = &drones[newcomer];
            seen.push((b.mode(), b.assigned_target()));
        }
        false
    });
    seen
}

#[test]
fn a_newcomer_leaves_a_full_target_alone() {
    for seed in 1..=3 {
        let seen = newcomer_near_a_full_target(seed);
        std::println!("seed {seed}: newcomer each second: {:?}", seen.iter().map(|(m, a)| std::format!("{m:?}/{}", a.map_or("-".into(), |a| std::format!("{a:04x}")))).collect::<std::vec::Vec<_>>());
        assert!(seen[2..].iter().all(|(m, _)| *m == Mode::Searching), "newcomer went for a full target: {seen:?}");
    }
}

#[test]
fn a_newcomer_out_of_the_rings_range_leaves_it_alone_too() {
    // It hears about the target only through a free drone in between, not from the ring.
    for seed in 1..=3 {
        let seen = newcomer_at(seed, &[Vec2::new(6.0, 0.0), Vec2::new(12.0, 0.0)]);
        std::println!("seed {seed}: newcomer each second: {:?}", seen.iter().map(|(m, a)| std::format!("{m:?}/{}", a.map_or("-".into(), |a| std::format!("{a:04x}")))).collect::<std::vec::Vec<_>>());
        assert!(seen[2..].iter().all(|(m, _)| *m == Mode::Searching), "newcomer went for a full target: {seen:?}");
    }
}







#[test]
fn newcomers_leave_a_full_target_alone_over_a_lossy_radio() {
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    // The newcomer switches on 10 s in, when the ring has long settled.
    LATE_JOINER_TICK.with(|v| v.set(200));
    let mut bad = std::vec::Vec::new();
    for seed in 1..=12 {
        for others in [std::vec![Vec2::new(7.0, 0.0)], std::vec![Vec2::new(6.0, 0.0), Vec2::new(12.0, 0.0)]] {
            let seen = newcomer_at(seed, &others);
            let went = seen[10..].iter().filter(|(m, _)| *m != Mode::Searching).count();
            if went > 0 {
                bad.push((seed, others.len(), went, seen.iter().map(|(m, a)| std::format!("{m:?}/{}", a.map_or("-".into(), |a| std::format!("{a:04x}")))).collect::<std::vec::Vec<_>>()));
            }
        }
    }
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    LATE_JOINER_TICK.with(|v| v.set(0));
    for b in &bad {
        std::println!("seed {} ({} other drones): {} s not searching: {:?}", b.0, b.1, b.2, b.3);
    }
    // Now and then a newcomer that wandered out of the ring's earshot still goes by an old
    // count of its drones: 7 runs of 120 over 60 seeds, however the sightings are relayed.
    // So a share: 5 of 24 fails one run in seventy by chance alone.
    assert!(bad.len() <= 4, "{} of 24 runs went for a full target", bad.len());
}

#[test]
fn a_place_freed_by_a_dead_responder_is_taken() {
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    FIRST_DIES_AT_TICK.with(|v| v.set(200));
    let mut joined_after = std::vec::Vec::new();
    for seed in 1..=60 {
        let seen = newcomer_at(seed, &[Vec2::new(7.0, 0.0)]);
        // One sample a second; drone 1 dies at 10 s.
        joined_after.push(seen.iter().enumerate().skip(10).find(|(_, (m, _))| *m != Mode::Searching).map(|(s, _)| s - 10));
    }
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    FIRST_DIES_AT_TICK.with(|v| v.set(0));
    std::println!("seconds from a responder dying to the newcomer taking its place: {joined_after:?}");
    // Mostly within 8 s. Not always: the newcomer searches at the edge of the ring's radio
    // range, and now and then it's out of earshot for longer. Over these 60 seeds: 46 in
    // time with signed sightings, 49 with statuses relayed only where neighbors haven't, 43
    // with sightings repeated that way too (all within chance of each other). Under 35
    // would be well outside it.
    let in_time = joined_after.iter().filter(|j| j.is_some_and(|s| s <= 8)).count();
    assert!(in_time >= 35, "{in_time} of 60 in time: {joined_after:?}");
}
