//! The harness: a swarm of brains flown together over an emulated radio, and its switches.

use super::*;

// Proximity sensing (ranging other drones) in the harness: its range in dm (0 = none).
std::thread_local! { pub(super) static PROXIMITY_DM: core::cell::Cell<u32> = const { core::cell::Cell::new(0) }; }
// GPS error in the harness: standard deviation of the shared part, in cm (0 = exact).
std::thread_local! { pub(super) static GPS_SIGMA_CM: core::cell::Cell<u32> = const { core::cell::Cell::new(0) }; }
// If set, the harness's first drone dies (goes still and silent) at this tick.
std::thread_local! { pub(super) static FIRST_DIES_AT_TICK: core::cell::Cell<u32> = const { core::cell::Cell::new(0) }; }
// If set, the harness keeps its last drone switched off until this tick.
std::thread_local! { pub(super) static LATE_JOINER_TICK: core::cell::Cell<u32> = const { core::cell::Cell::new(0) }; }
// If set, the harness launches its drones three at a time, this many ticks apart, like the
// simulator does: drone k sits switched off until tick (k / 3) times this.
std::thread_local! { pub(super) static LAUNCH_EVERY_TICKS: core::cell::Cell<u32> = const { core::cell::Cell::new(0) }; }
// Harness radio loss in percent (0 = perfect radio), for tests that want it lossy. These
// harness settings are per thread: each test runs on its own, so they don't leak between
// tests running in parallel.
std::thread_local! { pub(super) static RADIO_LOSS_PERCENT: core::cell::Cell<u32> = const { core::cell::Cell::new(0) }; }
// Harness jamming, from..until (ticks): every frame lost. (0, 0) is off.
std::thread_local! { pub(super) static RADIO_JAM_TICKS: core::cell::Cell<(u32, u32)> = const { core::cell::Cell::new((0, 0)) }; }

/// A whole little swarm, kinematic and with a perfect radio within `range`: enough to check
/// properties that only show up between brains, like whether names stay consistent.
pub(super) fn fly_swarm(starts: &[Vec2], targets: &[Vec2], range: f32, ticks: u32) -> std::vec::Vec<(Brain, Vec2)> {
    fly_swarm_with(RINGS_OF_FOUR, 7, starts, targets, range, ticks, |_, _| false)
}


/// A ground station heartbeat from `GCS_SYSID`, as heard over the radio.
pub(super) fn gcs_heartbeat() -> (MavHeader, MavMessage) {
    let header = MavHeader { system_id: GCS_SYSID, component_id: 190, sequence: 0 };
    let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });
    (header, msg)
}

/// `fly_swarm`, with a config, a seed, and a hook called after every tick that can stop
/// the flight early by returning true.
pub(super) fn fly_swarm_with(
    cfg: BrainConfig,
    seed: u32,
    starts: &[Vec2],
    targets: &[Vec2],
    range: f32,
    ticks: u32,
    stop: impl FnMut(u32, &[(Brain, Vec2)]) -> bool,
) -> std::vec::Vec<(Brain, Vec2)> {
    fly_swarm_full(cfg, seed, starts, targets, range, ticks, None, stop)
}

/// `fly_swarm_with`, optionally with a ground station at `gcs` sending a heartbeat a second.
#[allow(clippy::too_many_arguments)]
pub(super) fn fly_swarm_full(
    cfg: BrainConfig,
    seed: u32,
    starts: &[Vec2],
    targets: &[Vec2],
    range: f32,
    ticks: u32,
    gcs: Option<Vec2>,
    stop: impl FnMut(u32, &[(Brain, Vec2)]) -> bool,
) -> std::vec::Vec<(Brain, Vec2)> {
    let fixed = targets.to_vec();
    fly_swarm_moving(cfg, seed, starts, move |_| fixed.clone(), range, ticks, gcs, stop)
}

/// `fly_swarm_full` with targets that move: `targets_at(tick)` gives their positions.
#[allow(clippy::too_many_arguments)]
pub(super) fn fly_swarm_moving(
    cfg: BrainConfig,
    seed: u32,
    starts: &[Vec2],
    targets_at: impl Fn(u32) -> std::vec::Vec<Vec2>,
    range: f32,
    ticks: u32,
    gcs: Option<Vec2>,
    mut stop: impl FnMut(u32, &[(Brain, Vec2)]) -> bool,
) -> std::vec::Vec<(Brain, Vec2)> {
    let mut drones: std::vec::Vec<(Brain, Vec2)> =
        starts.iter().enumerate().map(|(i, &p)| (Brain::new(i as u8 + 1, cfg, seed.wrapping_mul(31) + i as u32, test_keys(i as u8 + 1)), p)).collect();
    // Like netem: each delivery is lost with this probability, and arrives one or two
    // ticks (50-100 ms) after it was sent. Off unless a test turns it on.
    let loss = RADIO_LOSS_PERCENT.with(|v| v.get()) as f32 / 100.0;
    // Jammed from..until (ticks): nothing gets through, though it's still sent.
    let (jam_from, jam_until) = RADIO_JAM_TICKS.with(|v| v.get());
    let mut rng = XorShift32::new(seed.wrapping_mul(7_919) ^ 0xA5A5);
    let gps_sigma = GPS_SIGMA_CM.with(|v| v.get()) as f32 / 100.0;
    let gauss = |r: &mut XorShift32| {
        let (u1, u2) = (r.next_f32().max(1e-6), r.next_f32());
        let m = libm::sqrtf(-2.0 * libm::logf(u1));
        Vec2::new(m * libm::cosf(core::f32::consts::TAU * u2), m * libm::sinf(core::f32::consts::TAU * u2))
    };
    // Its own random stream, so turning GPS error on or off doesn't change which radio
    // frames are lost.
    let mut gps_rng = XorShift32::new(seed.wrapping_mul(104_729) ^ 0x5A5A);
    let common_err = gauss(&mut gps_rng) * gps_sigma;
    let own_err: std::vec::Vec<Vec2> = (0..starts.len()).map(|_| gauss(&mut gps_rng) * (gps_sigma * 0.47)).collect();
    // (arrives at tick, sent from, frame)
    // (when it arrives, where from, who sent it, the frame): a relay goes out under its
    // original header, so the sender is kept apart from it, like netem knows it by address.
    let mut air: std::vec::Vec<(u32, Vec2, u8, (MavHeader, MavMessage))> = std::vec::Vec::new();
    for tick in 0..ticks {
        let now = tick * 50;
        let mut next_air = std::vec::Vec::new();
        if let Some(at) = gcs.filter(|_| tick % 20 == 0) {
            air.push((tick, at, GCS_SYSID, gcs_heartbeat()));
        }
        let late = LATE_JOINER_TICK.with(|v| v.get());
        let count = drones.len();
        let proximity = PROXIMITY_DM.with(|v| v.get()) as f32 / 10.0;
        let everyone: std::vec::Vec<(u8, Vec2)> = drones.iter().map(|(b, p)| (b.sysid(), *p)).collect();
        for (k, (brain, pos)) in drones.iter_mut().enumerate() {
            // The last drone sits switched off until `late`: it joins a swarm already at work.
            if late > 0 && k == count - 1 && tick < late {
                continue;
            }
            if tick < (k as u32 / 3) * LAUNCH_EVERY_TICKS.with(|v| v.get()) {
                continue;
            }
            // The first drone dies at `dies`: it stops, and goes silent.
            let dies = FIRST_DIES_AT_TICK.with(|v| v.get());
            if dies > 0 && k == 0 && tick >= dies {
                continue;
            }
            let inbox: std::vec::Vec<_> = air
                .iter()
                .filter(|(at, from, sender, _)| *at <= tick && *sender != brain.sysid() && (*from - *pos).length() <= range)
                .filter(|_| loss == 0.0 || rng.next_f32() >= loss)
                .filter(|_| !(jam_from..jam_until).contains(&tick))
                .map(|(_, _, _, f)| f.clone())
                .collect();
            // What its radio takes in this tick: everything sent within range (lost or not,
            // it's on the air).
            let on_air: u64 = air.iter().filter(|(at, from, sender, _)| *at == tick && *sender != brain.sysid() && (*from - *pos).length() <= range).count() as u64;
            RADIO.with(|r| {
                let mut v = r.get();
                v.heard_frames += on_air;
                v.drone_ticks += 1;
                v.busiest_tick = v.busiest_tick.max(on_air);
                r.set(v);
            });
            let targets = targets_at(tick);
            // GPS error (when a test turns it on): a shared part and this drone's own part,
            // both fixed for the run (they drift over minutes), plus white noise. Targets are
            // seen relative to the drone, so they inherit its error.
            let err = if gps_sigma > 0.0 {
                common_err + own_err[k] + Vec2::new(gps_rng.next_f32() - 0.5, gps_rng.next_f32() - 0.5) * 0.7
            } else {
                Vec2::ZERO
            };
            let fix = *pos + err;
            let seen: std::vec::Vec<Vec2> = targets.iter().copied().filter(|t| (*t - *pos).length() <= 2.5).map(|t| t - *pos).collect();
            let reading = seeing(now, &seen);
            // Ranging: the nearest drones within range, exactly.
            let nearby = (proximity > 0.0).then(|| {
                let mut near: std::vec::Vec<(u8, Vec2)> = everyone.iter().filter(|(id, p)| *id != brain.sysid() && (*p - *pos).length() <= proximity).map(|&(id, p)| (id, p - *pos)).collect();
                near.sort_by(|a, b| a.1.length().total_cmp(&b.1.length()));
                let mut out = [None; MAX_NEARBY];
                for (slot, (id, rel)) in out.iter_mut().zip(near) {
                    *slot = Some((id, rel, Vec2::ZERO));
                }
                out
            });
            let reading = SensorReading { own_pos: fix, nearby, ..reading };
            let out = brain.on_tick(&reading, &inbox);
            let own = MavHeader { system_id: brain.sysid(), component_id: COMPONENT_ID, sequence: 0 };
            for (relayed, m) in brain.outgoing_frames(now).into_iter().flatten() {
                FRAMES_SENT.with(|f| *f.borrow_mut().entry(m.message_name()).or_default() += 1);
                // On the air, as MAVLink 2: header, payload, checksum.
                let mut buf = [0u8; 280];
                let bytes = 12 + m.ser(MavlinkVersion::V2, &mut buf) as u64 + 2;
                RADIO.with(|r| {
                    let mut v = r.get();
                    v.sent_frames += 1;
                    v.sent_bytes += bytes;
                    r.set(v);
                });
                let delay = if loss == 0.0 { 1 } else { 1 + (rng.next_f32() < 0.5) as u32 };
                next_air.push((tick + delay, *pos, brain.sysid(), (relayed.unwrap_or(own), m)));
            }
            *pos = *pos + out.velocity * 0.05;
        }
        // Delivered frames are gone; ones still in flight stay.
        air.retain(|(at, _, _, _)| *at > tick);
        air.extend(next_air);
        if stop(tick, &drones) {
            break;
        }
    }
    drones
}
