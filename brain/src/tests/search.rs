//! Searching without a formation.

use super::*;

#[test]
fn far_apart_targets_never_share_a_name() {
    let targets = [Vec2::new(-10.0, 0.0), Vec2::new(10.0, 0.0)];
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| Vec2::new(-11.0 + 2.0 * i as f32, 1.0 + (i % 3) as f32)).collect();
    let drones = fly_swarm(&starts, &targets, 8.0, 1200);
    // Which real target does each drone's track by that name point at?
    let mut names: std::collections::BTreeMap<TargetId, std::collections::BTreeSet<usize>> = Default::default();
    for (brain, _) in &drones {
        for t in brain.tracks.iter().flatten() {
            let k = if (t.pos - targets[0]).length() < (t.pos - targets[1]).length() { 0 } else { 1 };
            names.entry(t.id).or_default().insert(k);
        }
    }
    std::println!("names -> targets: {names:?}");
    assert!(names.values().all(|ks| ks.len() == 1), "one name used for two targets: {names:?}");
    let per_target: std::vec::Vec<usize> = (0..2).map(|k| drones.iter().filter(|(_, p)| (*p - targets[k]).length() < 4.0).count()).collect();
    std::println!("drones within 4 m of each target: {per_target:?}");
    // Both targets get their responders (give or take one still leaving).
    assert!(per_target.iter().all(|&n| (3..=5).contains(&n)), "drones per target: {per_target:?}");
}

#[test]
fn coverage_heads_for_unsearched_ground() {
    let mut brain = test_brain(1, cfg(), 1);
    // Hovering at the field center: the ground around it is searched, so it moves on.
    brain.on_tick(&reading(1_000, Vec2::ZERO, None), &[]);
    let out = brain.on_tick(&reading(1_050, Vec2::ZERO, None), &[]);
    let goal = brain.search_goal.expect("has a goal");
    assert!(brain.searched_at[goal.0][goal.1] == NEVER, "goal {goal:?} was already searched");
    assert!(out.velocity.length() > 1.9);
}

#[test]
fn free_neighbors_split_the_field() {
    // A free neighbor 6 m east of us: the east is its to search, so we head west.
    let mut brain = test_brain(1, cfg(), 1);
    let inbox = [position(2, Vec2::new(6.0, 0.0), Vec2::ZERO), heartbeat(2, 0)];
    let out = brain.on_tick(&reading(1_000, Vec2::ZERO, None), &inbox);
    let goal = brain.cell_center(brain.search_goal.expect("has a goal"));
    assert!(goal.x < 3.0, "goal {goal:?} is closer to the neighbor");
    assert!(out.velocity.x <= 0.0 || (goal - Vec2::ZERO).length() < (goal - Vec2::new(6.0, 0.0)).length());
}

/// Seconds until every target has been within sensor range of some drone, for a swarm
/// launched together from one corner of the field. Three targets: with 4 responders each,
/// that's all 12 drones — a fourth would have nobody left to look for it.
fn time_to_find_all(search: SearchStrategy, seed: u32) -> f32 {
    let cfg = BrainConfig { search, ..RINGS_OF_FOUR };
    let targets = [Vec2::new(12.0, 12.0), Vec2::new(-12.0, 11.0), Vec2::new(11.0, -12.0)];
    let starts: std::vec::Vec<Vec2> = (0..12).map(|i| Vec2::new(-13.0 + (i % 4) as f32 * 1.5, -13.0 + (i / 4) as f32 * 1.5)).collect();
    let mut found = [false; 3];
    let mut when = None;
    fly_swarm_with(cfg, seed, &starts, &targets, 8.0, 6_000, |tick, drones| {
        for (k, t) in targets.iter().enumerate() {
            found[k] |= drones.iter().any(|(_, p)| (*p - *t).length() <= 2.5);
        }
        if found.iter().all(|&f| f) {
            when = Some(tick);
        }
        when.is_some()
    });
    when.map_or(300.0, |tick| tick as f32 * 0.05)
}

#[test]
fn coverage_search_finds_targets_faster_than_a_random_walk() {
    let seeds = 1..=6;
    let random: std::vec::Vec<f32> = seeds.clone().map(|s| time_to_find_all(SearchStrategy::RandomWalk, s)).collect();
    let coverage: std::vec::Vec<f32> = seeds.map(|s| time_to_find_all(SearchStrategy::Coverage, s)).collect();
    let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len() as f32;
    std::println!("time to find all 3 targets, s — random walk: {random:?} (mean {:.0}), coverage: {coverage:?} (mean {:.0})", mean(&random), mean(&coverage));
    assert!(mean(&coverage) < 0.6 * mean(&random));
}
