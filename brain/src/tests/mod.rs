//! The brain's tests, a file per topic, on the shared helpers here and the harness (harness.rs).

use super::*;

mod sightings;
mod harness;
mod search;
mod operator;
mod relay;
mod formation;
mod survey;
mod plans;
mod rings;
mod nav;
mod sticks;
mod orders;

use sightings::*;
use harness::*;
use operator::*;
use relay::*;
use plans::*;
use rings::*;
use sticks::*;

/// The swarm's configuration with four drones to a target and a status a second, as most of
/// these tests were written for: they're about how rings form, hold and give way.
const RINGS_OF_FOUR: BrainConfig = BrainConfig { responders: 4, status_period_ms: 1_000, ..BrainConfig::SWARM };

const LIAR: u8 = 7;

fn cfg() -> BrainConfig {
    BrainConfig {
        max_speed: 2.0,
        field_half: Vec2::new(15.0, 15.0),
        forget_after_ms: 5_000,
        last_seen_memory_ms: 30_000,
        beacon_period_ms: 200,
        heartbeat_period_ms: 1_000,
        standoff_radius: 2.0,
        separation_radius: 1.2,
        avoidance_horizon_ms: 1_000,
        neighbor_timeout_ms: 2_000,
        isolation_timeout_ms: 3_000,
        heartbeat_timeout_ms: 3_500,
        search: SearchStrategy::Coverage,
        wander_turn_chance: 0.0,
        scan_radius: 2.5,
        travel_cost_s_per_m: 2.0,
        quorum: 2,
        agree_radius: 3.0,
        merge_radius: 0.5,
        vote_window_ms: 3_000,
        investigate_radius: 10.0,
        verify_radius: 1.0,
        refute_max_age_ms: 1_000,
        distrust_ms: 15_000,
        responders: 4,
    adaptive_responders: false,
        home: Vec2::new(-13.0, -13.0),
        cancelled_forget_after_ms: 60_000,
        status_period_ms: 1_000,
        status_max_hops: 6,
        keep_link: true,
    relay_chains: false,
    fleet: 0,
    sweep_pattern: SweepPattern::Cells,
        link_range: 6.5,
        link_lost_timeout_ms: 30_000,
        dead_reckoning_drift: 0.15,
        gps_disagreement_m: 5.0,
        gps_reject_ms: 10_000,
        reference_accuracy_m: 4.0,
        // Zero in tests, so an age in a message is exactly the age the brain sees.
        hop_latency_ms: 0,
        max_prediction_ms: 2_000,
    }
}

fn reading(time_ms: u32, own_pos: Vec2, target_seen: Option<Vec2>) -> SensorReading {
    let mut targets_seen = [None; MAX_SEEN];
    targets_seen[0] = target_seen.map(|t| t - own_pos);
    SensorReading {
        time_ms,
        own_pos,
        gps_fix: true,
        own_velocity: Vec2::ZERO,
        targets_seen,
        nearby: None,
        battery: f32::NAN,
    }
}

std::thread_local! { static CAUSES: core::cell::Cell<[u32; 5]> = const { core::cell::Cell::new([0; 5]) }; }

/// Why a heartbeat or beacon went out (see `traffic_survey`): 0 heartbeat on schedule,
/// 1 announcement changed, 2 link beat, 3 beacon on schedule, 4 urgent beacon.
pub(super) fn count_cause(i: usize) {
    CAUSES.with(|c| {
        let mut v = c.get();
        v[i] += 1;
        c.set(v);
    });
}

/// Not a pass/fail test: what a big swarm puts on the air, per drone and second.
/// `cargo test --release --features std traffic_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn traffic_survey() {
    let gps_cm = std::env::var("SURVEY_GPS_CM").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    GPS_SIGMA_CM.with(|v| v.set(gps_cm));
    std::println!("GPS error {gps_cm} cm");
    for n in [12usize, 32, 64] {
        CAUSES.with(|c| c.set([0; 5]));
        let base = Vec2::new(-13.0, -13.0);
        let starts: std::vec::Vec<Vec2> = (0..n).map(|i| base + Vec2::new((i % 8) as f32 * 1.0, (i / 8) as f32 * 1.0)).collect();
        let targets = [Vec2::new(-4.0, 2.0), Vec2::new(6.0, -8.0), Vec2::new(8.0, 9.0)];
        let cfg = RINGS_OF_FOUR.with_mode(SwarmMode::Autonomous);
        let mut sent: std::collections::BTreeMap<&'static str, u32> = std::collections::BTreeMap::new();
        let mut heard_max = 0usize;
        let ticks = 1_200;
        let mut last: std::vec::Vec<(Brain, Vec2)> = std::vec::Vec::new();
        last.clear();
        fly_swarm_moving(cfg, 1, &starts, |_| targets.to_vec(), 8.0, ticks, None, |_, drones| {
            // Who hears whom, from where they are now: what each drone's radio takes in.
            for (_, p) in drones {
                heard_max = heard_max.max(drones.iter().filter(|(_, q)| (*q - *p).length() <= 8.0).count() - 1);
            }
            false
        });
        // Count what went out, by type: replay a few seconds' outgoing from the survivors.
        let drones = fly_swarm_moving(cfg, 1, &starts, |_| targets.to_vec(), 8.0, ticks, None, |_, _| false);
        let _ = drones;
        FRAMES_SENT.with(|f| {
            for (k, v) in f.borrow().iter() {
                *sent.entry(k).or_default() += v;
            }
            f.borrow_mut().clear();
        });
        let secs = 2.0 * ticks as f32 * 0.05 * n as f32;
        let per: std::vec::Vec<_> = sent.iter().map(|(k, v)| std::format!("{k} {:.2}", *v as f32 / secs)).collect();
        let c = CAUSES.with(|c| c.get());
        std::println!("{n} drones: sent per drone per s: {per:?}");
        std::println!("   heartbeats: scheduled {:.2} changed {:.2} link {:.2}; beacons: scheduled {:.2} urgent {:.2} (per drone per s); most neighbors in range {heard_max}",
            c[0] as f32 / secs, c[1] as f32 / secs, c[2] as f32 / secs, c[3] as f32 / secs, c[4] as f32 / secs);
    }
}

/// Radio traffic in the harness, from when a test last cleared it.
#[derive(Clone, Copy, Default)]
struct RadioStats {
    sent_frames: u64,
    sent_bytes: u64,
    /// Frames on the air within range of a drone, summed over drones and ticks.
    heard_frames: u64,
    drone_ticks: u64,
    /// The most frames any drone had on the air within range in one tick.
    busiest_tick: u64,
}

std::thread_local! { static RADIO: core::cell::Cell<RadioStats> = const { core::cell::Cell::new(RadioStats { sent_frames: 0, sent_bytes: 0, heard_frames: 0, drone_ticks: 0, busiest_tick: 0 }) }; }

std::thread_local! { static FRAMES_SENT: core::cell::RefCell<std::collections::BTreeMap<&'static str, u32>> = const { core::cell::RefCell::new(std::collections::BTreeMap::new()) }; }

const TEST_MISSION: u32 = 7;

/// Keys for every system ID, from one test authority.
fn fleet_keys() -> &'static [Keys] {
    static KEYS: std::sync::OnceLock<std::vec::Vec<Keys>> = std::sync::OnceLock::new();
    KEYS.get_or_init(|| {
        let authority = auth::Authority::from_secret([0xA5; 32]);
        (0..=255u8).map(|id| authority.issue(TEST_MISSION, id, [id ^ 0x3C; 32])).collect()
    })
}

fn test_keys(sysid: u8) -> Keys {
    fleet_keys()[sysid as usize]
}

/// A drone that already has every test drone's certificate, as if it had heard them all —
/// so a test can hand it sightings without their certificates. Swarms of brains talking to
/// each other (`fly_swarm_moving`, `relay_chain`) start without any, like real drones.
fn test_brain(sysid: u8, cfg: BrainConfig, seed: u32) -> Brain {
    let mut brain = Brain::new(sysid, cfg, seed, test_keys(sysid));
    for k in fleet_keys() {
        brain.certificates[k.sysid as usize] = Some((k.public(), k.certificate));
    }
    brain
}

/// Signed by `observer` as a new observation of its own (a later one each time).
fn signed(mut s: SWARM_TARGET_SIGHTING_DATA, observer: u8) -> SWARM_TARGET_SIGHTING_DATA {
    static OBSERVED: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(1);
    test_keys(observer).sign_sighting(&mut s, OBSERVED.fetch_add(1, core::sync::atomic::Ordering::Relaxed));
    s
}

/// A firsthand report of a stationary target, as `observer` broadcasts it `age_ms` after
/// seeing it.
fn sighting(observer: u8, pos: Vec2, age_ms: u32) -> (MavHeader, MavMessage) {
    let header = MavHeader {
        system_id: observer,
        component_id: COMPONENT_ID,
        sequence: 0,
    };
    let msg = MavMessage::SWARM_TARGET_SIGHTING(signed(SWARM_TARGET_SIGHTING_DATA {
        // Each observer's own first track number.
        target_id: ((observer as TargetId) << 8) | 1,
        age_ms,
        x: pos.x,
        y: pos.y,
        vx: 0.0,
        vy: 0.0,
        observer_sysid: observer,
        cancelled: 0,
        responders: 0,
        hops: 0,
        trace_id: [0; 16],
        parent_span_id: [0; 8],
        observed_ms: 0,
        signature: [0; 64],
    }, observer));
    (header, msg)
}
