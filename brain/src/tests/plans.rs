//! Fleet plans and covering the field.

use super::*;

#[test]
fn the_fleet_plan_grows_with_the_field() {
    let cfg = RINGS_OF_FOUR;
    let small = cfg.plan(2);
    // 30 x 30 m from the corner base: 3 drones abreast look over it in two lanes in 47 s,
    // within the planned minute (6, as wide as the field, would take 19 s); 2 mice of 4; 4
    // relays each (4.5 m apart) out to 23 m (60% of the way to the far corner, 38 m off).
    assert_eq!((small.sweep, small.rings, small.relays, small.total), (3, 8, 8, 19), "{small:?}");
    assert!(small.pass_s <= PLANNED_PASS_S, "{small:?}");
    let big = BrainConfig { field_half: Vec2::new(40.0, 30.0), home: Vec2::new(-38.0, -28.0), ..cfg }.plan(2);
    assert!(big.sweep > small.sweep && big.relays > small.relays && big.pass_s <= PLANNED_PASS_S, "{big:?}");
    assert_eq!(cfg.plan(0).total, small.sweep);
}

pub(super) fn field(w: f32, h: f32) -> BrainConfig {
    let half = Vec2::new(w / 2.0, h / 2.0);
    BrainConfig { field_half: half, home: Vec2::new(2.0 - half.x, 2.0 - half.y), ..BrainConfig::SWARM }.with_mode(SwarmMode::Autonomous)
}

#[test]
fn a_wide_field_is_swept_by_one_front_in_segments_that_hold_together() {
    // 180 x 175 m: 35 drones abreast span it, one network from end to end, in segments short
    // enough for their drones to agree on who's in them.
    let cfg = field(180.0, 175.0);
    let l = cfg.sweep_layout(35);
    assert_eq!((l.rows, l.per_row, l.lanes), (1, 35, 1), "{l:?}");
    assert!(l.efficiency > 0.95 && (l.pass_s - 118.7).abs() < 1.0, "{l:?}");
    for n in 1..=128 {
        let l = cfg.sweep_layout(n);
        let spacing = 2.0 * (cfg.field_half.x.min(cfg.field_half.y) - SWEEP_EDGE_MARGIN) / (l.lanes * l.per_row) as f32;
        let longest = (0..l.rows).flat_map(|r| (0..l.places(r)).map(move |q| l.segment(r, q).2)).max().unwrap();
        assert!(longest <= cfg.segment_max(spacing), "{n}: {l:?}");
    }
    // Every two drones side by side within radio range, all the way across, whenever.
    for t_ms in (0..300_000).step_by(5_000) {
        let slots: std::vec::Vec<Vec2> = (0..35).map(|r| cfg.formation_slot(r, 35, t_ms).unwrap()).collect();
        assert!(slots.windows(2).all(|w| (w[1] - w[0]).length() <= 5.5), "at {t_ms} ms: {slots:?}");
    }
}

#[test]
fn the_fleets_worth_flying_each_sweep_sooner() {
    let cfg = field(180.0, 175.0);
    let options: std::vec::Vec<SweepLayout> = cfg.sweep_options(128).collect();
    assert!(options.windows(2).all(|w| w[1].drones > w[0].drones && w[1].pass_s < 0.98 * w[0].pass_s), "{options:?}");
    // The fewest for a pass in a minute: two fronts as wide as the field.
    let minute = cfg.sweep_for(60.0);
    assert_eq!((minute.drones, minute.rows, minute.lanes), (70, 2, 1), "{minute:?}");
    assert_eq!(cfg.sweep_for(1.0), *options.last().unwrap());
}

#[test]
fn two_fronts_meet_at_their_boundary_every_pass() {
    // 70 drones on 180 x 175 m: two fronts, out of range of each other between meetings.
    let cfg = field(180.0, 175.0);
    let l = cfg.sweep_layout(70);
    assert_eq!(l.rows, 2, "{l:?}");
    let gap = |t: u32| {
        let a: std::vec::Vec<Vec2> = (0..70).step_by(2).map(|r| cfg.formation_slot(r, 70, t).unwrap()).collect();
        let b: std::vec::Vec<Vec2> = (1..70).step_by(2).map(|r| cfg.formation_slot(r, 70, t).unwrap()).collect();
        a.iter().flat_map(|p| b.iter().map(move |q| (*p - *q).length())).fold(f32::MAX, f32::min)
    };
    // Within radio range of each other again every other pass: out to the field's end and back.
    let period_ms = (2.2 * l.pass_s * 1000.0) as u32;
    for start in (100_000..500_000).step_by(period_ms as usize) {
        let met = (start..start + period_ms).step_by(500).any(|t| gap(t) <= 8.0);
        assert!(met, "the fronts didn't meet between {start} and {} ms", start + period_ms);
    }
}

#[test]
fn the_formation_covers_a_rectangular_field() {
    // 50 x 30 m, 8 free drones: the formation (40 m wide) mows it in lanes along its long
    // side, never leaving it, and in one round trip passes within sensor range of all of it.
    let half = Vec2::new(25.0, 15.0);
    let cfg = BrainConfig { relay_chains: true, field_half: half, home: Vec2::new(-23.0, -13.0), ..RINGS_OF_FOUR };
    let round_trip = 2.0 * cfg.cover_time_s(8);
    let mut seen = [[false; 30]; 50];
    for t in (0..(round_trip * 1000.0) as u32).step_by(250) {
        for rank in 0..8 {
            let p = cfg.formation_slot(rank, 8, t).expect("a slot");
            assert!(p.x.abs() <= half.x && p.y.abs() <= half.y, "{p:?} outside");
            for (x, column) in seen.iter_mut().enumerate() {
                for (y, cell) in column.iter_mut().enumerate() {
                    let q = Vec2::new(-24.5 + x as f32, -14.5 + y as f32);
                    *cell |= (q - p).length() <= cfg.scan_radius + 0.5;
                }
            }
        }
    }
    let covered = seen.iter().flatten().filter(|&&c| c).count() as f32 / 1500.0;
    std::println!("share of the field the formation passed over: {covered}, in {round_trip:.0} s");
    assert!(covered > 0.95, "{covered}");
}

#[test]
fn with_relay_chains_the_swarm_spreads_over_the_field_in_contact() {
    let neither: std::vec::Vec<_> = (1..=4).map(|s| spread(None, s)).collect();
    let connected: std::vec::Vec<_> = (1..=4).map(|s| spread(Some(SwarmMode::Connected), s)).collect();
    let autonomous: std::vec::Vec<_> = (1..=4).map(|s| spread(Some(SwarmMode::Autonomous), s)).collect();
    std::println!("(field covered, in contact, second mouse found after s) neither mode: {neither:?}");
    std::println!("connected:                                                         {connected:?}");
    std::println!("autonomous:                                                        {autonomous:?}");
    // Connected: the 8 free drones sweep the field in one ray out from the base, in contact
    // all the while, and the second mouse, out toward the far side, is found (without either
    // mode, it never was).
    assert!(connected.iter().all(|&(covered, contact, found)| covered > 0.3 && contact > 0.99 && found.is_some()), "{connected:?}");
    // Autonomous finds it sooner, out of the base's reach much of the time.
    assert!(autonomous.iter().zip(&connected).all(|(a, c)| a.2.is_some_and(|a| c.2.is_some_and(|c| a <= c))), "{autonomous:?}");
}

/// Three mice in different parts of the field, twelve drones launched from the base corner:
/// every drone a responder once all three are found, with nobody free. Returns (share of
/// drone-ticks with a radio path to the base, share of mouse-ticks with a drone within 4 m),
/// from 60 s on, over 8 minutes.
pub(super) fn one_network(relay_chains: bool, seed: u32) -> (f32, f32) {
    let cfg = BrainConfig { relay_chains, ..RINGS_OF_FOUR };
    let base = Vec2::new(-13.0, -13.0);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let mice = [Vec2::new(-6.0, 6.0), Vec2::new(6.0, -6.0), Vec2::new(5.0, 5.0)];
    let (mut connected, mut total, mut watched, mut samples) = (0u32, 0u32, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, |_| mice.to_vec(), 8.0, 9_600, Some(base), |tick, drones| {
        if tick < 1_200 {
            return false;
        }
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
        samples += 1;
        watched += mice.iter().filter(|m| drones.iter().any(|(_, p)| (*p - **m).length() <= 4.0)).count() as u32;
        false
    });
    (connected as f32 / total.max(1) as f32, watched as f32 / (samples * mice.len() as u32).max(1) as f32)
}

/// Not a pass/fail test: how many relays the chains take, and how the swarm does, with
/// three mice apart and with two close together.
/// `cargo test --release relay_tree_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn relay_tree_survey() {
    for (name, mice) in [("three apart", [Vec2::new(-6.0, 6.0), Vec2::new(6.0, -6.0), Vec2::new(5.0, 5.0)]), ("two close", [Vec2::new(8.0, 4.0), Vec2::new(9.0, 9.0), Vec2::new(-10.0, 4.0)])] {
        let mut rows = std::vec::Vec::new();
        for seed in 1..=3 {
            let cfg = BrainConfig { relay_chains: true, ..RINGS_OF_FOUR };
            let base = Vec2::new(-13.0, -13.0);
            let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
            let (mut relays, mut connected, mut watched, mut n) = (0u32, 0u32, 0u32, 0u32);
            fly_swarm_moving(cfg, seed, &starts, |_| mice.to_vec(), 8.0, 6_000, Some(base), |tick, drones| {
                if tick < 1_200 { return false; }
                n += 1;
                relays += drones.iter().filter(|(b, _)| b.relaying).count() as u32;
                connected += drones.iter().filter(|(b, _)| b.route.is_some()).count() as u32;
                watched += mice.iter().filter(|m| drones.iter().any(|(_, p)| (*p - **m).length() <= 4.0)).count() as u32;
                false
            });
            rows.push((relays as f32 / n as f32, connected as f32 / (n * 12) as f32, watched as f32 / n as f32));
        }
        std::println!("{name}: (mean relays, share with a route, mice watched of 3) {rows:?}");
    }
}

/// A mouse in the far corner and the chain out to it, over a lossy radio with GPS error:
/// returns (share of the relays' links back toward the base — to the drone their route goes
/// through, or the base — shorter than 3 m, share longer than 6.5 m), sampled once a second
/// from 30 s on.
pub(super) fn chain_gaps(seed: u32, proximity_dm: u32) -> (f32, f32) {
    let cfg = BrainConfig { relay_chains: true, ..RINGS_OF_FOUR };
    let base = Vec2::new(-13.0, -13.0);
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| base + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let mice = [Vec2::new(8.0, 8.0)];
    RADIO_LOSS_PERCENT.with(|v| v.set(10));
    GPS_SIGMA_CM.with(|v| v.set(150));
    PROXIMITY_DM.with(|v| v.set(proximity_dm));
    let (mut short, mut long, mut gaps) = (0u32, 0u32, 0u32);
    fly_swarm_moving(cfg, seed, &starts, |_| mice.to_vec(), 8.0, 2_400, Some(base), |tick, drones| {
        if tick % 20 == 0 && tick >= 600 {
            // Each relay's link back toward the base: to the drone its route goes through, or
            // the base itself.
            let at = |id: u8| drones.iter().find(|(b, _)| b.sysid() == id).map(|(_, p)| *p);
            for (b, p) in drones.iter().filter(|(b, _)| b.relaying) {
                let Some((_, parent)) = b.route else { continue };
                let Some(up) = (if parent == 0 { Some(base) } else { at(parent) }) else { continue };
                let gap = (*p - up).length();
                gaps += 1;
                short += (gap < 3.0) as u32;
                long += (gap > 6.5) as u32;
            }
        }
        false
    });
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    GPS_SIGMA_CM.with(|v| v.set(0));
    PROXIMITY_DM.with(|v| v.set(0));
    (short as f32 / gaps.max(1) as f32, long as f32 / gaps.max(1) as f32)
}

/// Not a pass/fail test: how evenly the relays space themselves along a chain over a lossy
/// radio with GPS error. Each relay goes to its spot by its own GPS, so they're uneven by the
/// drones' different GPS errors (a few meters).
/// `cargo test --release relay_spacing_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn relay_spacing_survey() {
    let by_gps: std::vec::Vec<_> = (1..=4).map(|s| chain_gaps(s, 0)).collect();
    let ranged: std::vec::Vec<_> = (1..=4).map(|s| chain_gaps(s, 80)).collect();
    std::println!("(share of gaps under 3 m, over 6.5 m) by GPS alone: {by_gps:?}, ranging neighbors within 8 m: {ranged:?}");
}

/// `count` drones from the base corner of a `size` field with relay chains on and no mice:
/// seconds until they've had 95% of the field within sensor range (2.5 m), up to 10 minutes.
pub(super) fn time_to_cover(mode: SwarmMode, count: usize, size: Vec2, seed: u32) -> Option<f32> {
    let half = size * 0.5;
    let home = Vec2::new(2.0 - half.x, 2.0 - half.y);
    let cfg = BrainConfig { field_half: half, home, ..RINGS_OF_FOUR.with_mode(mode) };
    let starts: std::vec::Vec<Vec2> = (0..count).map(|i| home + Vec2::new((i % 4) as f32 * 1.5, (i / 4) as f32 * 1.5)).collect();
    let (w, h) = (size.x as usize, size.y as usize);
    let mut seen = std::vec![false; w * h];
    let mut left = w * h;
    let mut done = None;
    fly_swarm_moving(cfg, seed, &starts, |_| std::vec::Vec::new(), 8.0, 12_000, Some(home), |tick, drones| {
        for (_, p) in drones {
            let (x0, x1) = (((p.x + half.x - 2.5).floor().max(0.0)) as usize, ((p.x + half.x + 2.5).ceil() as usize).min(w));
            let (y0, y1) = (((p.y + half.y - 2.5).floor().max(0.0)) as usize, ((p.y + half.y + 2.5).ceil() as usize).min(h));
            for x in x0..x1 {
                for y in y0..y1 {
                    let q = Vec2::new(x as f32 + 0.5 - half.x, y as f32 + 0.5 - half.y);
                    if !seen[x * h + y] && (q - *p).length() <= 2.5 {
                        seen[x * h + y] = true;
                        left -= 1;
                    }
                }
            }
        }
        if left * 20 <= w * h {
            done = Some(tick as f32 * 0.05);
            return true;
        }
        false
    });
    done
}

#[test]
#[ignore]
fn coverage_time_survey() {
    for (count, size) in [(8, Vec2::new(30.0, 30.0)), (12, Vec2::new(30.0, 30.0)), (12, Vec2::new(50.0, 30.0)), (27, Vec2::new(60.0, 60.0))] {
        let connected: std::vec::Vec<_> = (1..=2).map(|s| time_to_cover(SwarmMode::Connected, count, size, s)).collect();
        let autonomous: std::vec::Vec<_> = (1..=2).map(|s| time_to_cover(SwarmMode::Autonomous, count, size, s)).collect();
        std::println!("{count} drones, {:.0} x {:.0} m: 95% covered after (s) connected {connected:?}, autonomous {autonomous:?}", size.x, size.y);
    }
}
