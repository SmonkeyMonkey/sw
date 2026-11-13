//! The simulated physical world: ground truth for where every drone and the target actually
//! are. Each tick it feeds every live drone its own sensor reading, waits briefly for each
//! drone's velocity command, flies every airframe toward its command through the physics
//! (physics.rs), and publishes the state for the viewer and the network emulator. Drones never
//! read this state directly — only through their own sensors and MAVLink messages from each
//! other.
//!
//! It also applies changes from the viewer while the mission runs. A killed drone stops
//! receiving sensor readings, which ends its task, so it falls silent on the radio exactly like
//! a drone that crashed — the other drones only find out by its heartbeats stopping. Its rotors
//! stop too, so it also falls out of the sky. A removed drone goes silent the same way but
//! simply leaves; an added one is launched mid-mission with the next free system ID.

use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use rand::{Rng, RngExt};
use tokio::sync::{broadcast, mpsc};

use crate::body::{AgentReport, Fault};
use crate::fleet::Fleet;
use crate::gps::{Gps, GpsConfig};
use crate::observability::Telemetry;
use crate::physics::{Physics, MAVIC_CLASS};
use crate::ws::{AgentState, Command, JammerState, KnownTarget, ParkedState, Simulation, SwarmState, TargetState};
use swarm_brain::{Mode, Nav, SensorReading, Vec2, GRID, MAX_NEARBY, MAX_SEEN};

#[derive(Clone, Copy)]
pub struct WorldConfig {
    /// Drones launched from the base at mission start.
    pub num_agents: usize,
    /// The launch base, where drones take off and land.
    pub base: Vec2,
    /// One drone takes off from the base every this often while there are launches queued.
    pub launch_interval: Duration,
    /// Half the field's width (x) and height (y); it's centered on the origin.
    pub field_half: Vec2,
    pub sensor_range: f32,
    /// The radio's range, to tell which drones the ground station at the base can reach.
    pub radio_range: f32,
    /// Each drone's GNSS receiver (see gps.rs).
    pub gps: GpsConfig,
    /// Range of each drone's onboard proximity sensing of other drones (None: drones have none
    /// and avoid each other by their broadcast positions alone).
    pub proximity_range: Option<f32>,
    /// A cell counts toward `swarm.search.coverage` if flown over within this many seconds.
    pub recent_coverage_s: f32,
    pub num_targets: usize,
    pub target_speed: f32,
    /// Per-tick probability that the target picks a new random heading.
    pub target_turn_chance: f64,
    pub tick: Duration,
    /// Each drone's battery (None: they never run down, and don't know their charge).
    pub battery: Option<Battery>,
}

/// The base's pads, shared with the base's link to the ground station (base.rs): how the
/// operator set them, and what's on them.
#[derive(Clone, Copy, Debug)]
pub struct Dock {
    /// Spare airframes to put on the pads at the start of a mission (MAV_CMD_SWARM_SET_SPARES
    /// changes it, and the pads, at once).
    pub spares: u32,
    /// A charged airframe takes off in the place of each drone down to charge, flying as it
    /// (MAV_CMD_SWARM_SET_ROTATION).
    pub rotation: bool,
    /// Airframes on the pads, those charged enough to take off, and drones down to charge with
    /// none to take over from them yet.
    pub parked: u32,
    pub ready: u32,
    pub waiting: u32,
    /// The lowest charge of a drone in the air (None without batteries, or with none in the air).
    pub lowest_charge: Option<f32>,
    /// Whether this mission's drones have batteries that run down.
    pub batteries: bool,
}

impl Dock {
    pub fn new(spares: u32) -> Self {
        Self { spares, rotation: true, parked: 0, ready: 0, waiting: 0, lowest_charge: None, batteries: false }
    }
}

/// A drone's battery: how long a full charge flies it (s, more when flying fast), and how long
/// it takes to charge from empty on the base's pads (s).
#[derive(Clone, Copy, Debug)]
pub struct Battery {
    pub flight_s: f32,
    pub charge_s: f32,
}

/// An airframe on one of the base's pads, charging.
struct Parked {
    pad: Vec2,
    battery: f32,
}

/// Charged this far, an airframe on the pads takes off in the place of one that came back to
/// charge.
const READY_CHARGE: f32 = 0.95;
/// A drone landing within this far of the base lands on a free pad (to charge, with batteries):
/// the base guides it in, as a drone dock does.
const DOCK_ZONE_M: f32 = 12.0;
/// Over its pad within this, it descends.
const DOCK_ALIGN_M: f32 = 0.3;

pub struct WorldLinks {
    pub fleet: Fleet,
    pub report_rx: mpsc::Receiver<AgentReport>,
    /// Shared with the network emulator, which needs true positions to decide radio range.
    /// Indexed like the world's drone slots: entry `i` is system ID `i + 1`.
    pub positions: Arc<RwLock<Vec<Vec2>>>,
    pub ws_tx: broadcast::Sender<SwarmState>,
    pub command_rx: mpsc::Receiver<Command>,
    pub telemetry: Arc<Telemetry>,
    /// The base's pads: how the operator set them, and what's on them now.
    pub dock: Arc<RwLock<Dock>>,
    /// How the next mission will be simulated: the viewer sets it.
    pub simulation: Arc<RwLock<Simulation>>,
    /// When the mission starts: the mission clock (every drone's `SensorReading::time_ms`)
    /// counts from it, and so does the one the base gives the ground station to date its
    /// orders by (see `base::Mission`).
    pub started: Instant,
}

/// More than this and the field is mostly mice; also keeps a drone's sensor list (`MAX_SEEN`)
/// meaningful.
const MAX_TARGETS: usize = 8;

/// The autopilot's offboard failsafe. A drone whose brain sends no command holds its position
/// (every tick that happens); after this long without one, the autopilot gives up on the brain
/// and flies the drone back to the base by its own GPS (a return-to-launch, as PX4 does once
/// offboard control is lost), where it lands and leaves the mission, to be picked up. Without
/// a GPS fix it can't find the base, and lands where it is. A board that resets is back within
/// about a second, well inside it. A drone whose brain died doesn't hover on as an obstacle
/// nobody knows of (gone quiet, the swarm takes it for lost).
const OFFBOARD_LOSS_S: f32 = 3.0;
/// How fast the autopilot flies a drone home, and how close to the base it lands.
const FAILSAFE_SPEED: f32 = 2.0;
const FAILSAFE_HOME_M: f32 = 1.0;

/// Drones in the field at once: a big swarm on a big field. The brain's tables have room for
/// 255; the radio holds up because crowds stretch their beacons and relay only what neighbors
/// haven't (see the brain's `BEACON_CROWD` and `STATUS_COPIES`).
const MAX_DRONES: usize = 160;

/// System IDs run 1..=255, so a mission can launch at most this many drones in total.
const MAX_SLOTS: usize = 255;

/// Where a drone that left the mission is, as far as the radio emulator is concerned: out of
/// everyone's range.
const GONE: Vec2 = Vec2::new(1.0e6, 1.0e6);

/// Everything the world tracks about one drone. Slot `i` is system ID `i + 1` for the whole
/// mission; a drone that leaves keeps its slot (as `present: false`), so IDs never shift.
struct DroneSlot {
    /// `None` once the drone is dead or gone — dropping the sender is what ends its task.
    sensor_tx: Option<mpsc::Sender<SensorReading>>,
    fault_tx: mpsc::Sender<Fault>,
    /// Still in the field (alive, or a wreck on the ground). False once removed.
    present: bool,
    alive: bool,
    velocity: Vec2,
    informed: bool,
    known: [Option<(Vec2, bool)>; swarm_brain::MAX_TRACKS],
    role: swarm_brain::Role,
    isolated: bool,
    mode: Mode,
    liar: bool,
    /// The fake target it's broadcasting, if compromised.
    fake_target: Option<Vec2>,
    /// The swarm's ID of the target it's responding to.
    target: Option<u16>,
    /// Descending to land at base, after which it leaves the mission.
    landing: bool,
    /// Since when (s) its brain hasn't sent a command, and whether its autopilot gave up on it
    /// and is flying it home (see `OFFBOARD_LOSS_S`).
    silent_since_s: Option<f32>,
    failsafe: bool,
    /// Where its brain thinks it is, and where that came from (from its reports).
    believed: Vec2,
    nav: Nav,
    /// Its own velocity sensing (IMU, optical flow) is off by this: what makes dead reckoning
    /// drift.
    flow_bias: Vec2,
    /// GPS spoofing: the direction its fix is dragged, and since when (s).
    spoof: Option<(Vec2, f32)>,
    /// Its battery's charge, 0 to 1 (see `WorldConfig::battery`).
    battery: f32,
    /// The pad the base is guiding it onto, to charge, and whether it's over it and descending.
    docking: Option<(Vec2, bool)>,
    /// It asked to land to charge (see `swarm_brain::TickOutput::to_charge`), not sent home.
    to_charge: bool,
    /// Landed on a pad to charge: its system ID waits for a charged airframe to fly as it.
    vacant: bool,
}

impl DroneSlot {
    fn sysid(index: usize) -> u8 {
        index as u8 + 1
    }

    fn flying(&self) -> bool {
        self.present && self.alive
    }
}

/// How a mission ended: a reset from the viewer, or a new field (width, height) for the next
/// mission (from the viewer or the operator's ground station).
pub enum Ended {
    /// Start over with this many drones: everyone this mission launched or still had queued
    /// (the backend's starting count, or the operator's launches since a new field).
    Reset(u32),
    /// A new field (width, height), and how many drones to launch on it (None: as many as the
    /// last mission).
    NewField(Vec2, Option<u32>),
}

/// Runs one mission. Returns the command channel when the viewer asks for a reset or a new field
/// (so the next mission can keep listening to the same viewers), and which, or `None` if the
/// viewer side is gone.
pub async fn run(cfg: WorldConfig, mut links: WorldLinks) -> Option<(mpsc::Receiver<Command>, Ended)> {
    let half = cfg.field_half;
    let mut rng = rand::rng();

    let mut positions: Vec<Vec2> = Vec::new();
    let mut physics = Physics::new(MAVIC_CLASS, &[]);
    let mut gps = Gps::new(cfg.gps);
    // GPS jammers: (where, radius).
    let mut jammers: Vec<(Vec2, f32)> = Vec::new();
    let mut drones: Vec<DroneSlot> = Vec::new();
    // Every drone takes off from the base, one at a time.
    let mut launch_queue = cfg.num_agents as u32;
    let mut next_launch_s = 0.0f32;
    *links.positions.write().unwrap() = positions.clone();
    // Airframes on the base's pads, charging: the spares to begin with.
    let mut hangar: Vec<Parked> = Vec::new();
    let spares = links.dock.read().unwrap().spares;
    links.dock.write().unwrap().batteries = cfg.battery.is_some();
    set_spares(&mut hangar, spares, cfg.base, half, &positions, &drones);

    let mut next_target_id = 1;
    let mut targets: Vec<Target> = Vec::new();
    for _ in 0..cfg.num_targets {
        targets.push(Target::new(&mut next_target_id, random_point(&mut rng, half), &mut rng, 0.0));
    }
    // Ground truth for search: when some drone last flew over each cell.
    let mut visited_at = vec![f32::NEG_INFINITY; GRID * GRID];
    let cell_size = Vec2::new(2.0 * half.x / GRID as f32, 2.0 * half.y / GRID as f32);

    let start = links.started;
    let dt = cfg.tick.as_secs_f32();
    let mut interval = tokio::time::interval(cfg.tick);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut seq: u32 = 0;

    loop {
        interval.tick().await;
        let time_ms = start.elapsed().as_millis() as u32;
        let t_s = time_ms as f32 / 1000.0;

        // Changes requested from the viewer since the last tick.
        loop {
            let command = match links.command_rx.try_recv() {
                Ok(command) => command,
                Err(mpsc::error::TryRecvError::Empty) => break,
                Err(mpsc::error::TryRecvError::Disconnected) => return None,
            };
            match command {
                Command::Reset => {
                    // Dropping `links` and `drones` (and with them every drone's sensor feed)
                    // ends the drones' tasks.
                    let fleet = drones.len() as u32 + launch_queue;
                    return Some((links.command_rx, Ended::Reset(fleet)));
                }
                Command::SetSpares(n) => {
                    set_spares(&mut hangar, n, cfg.base, half, &positions, &drones);
                    links.dock.write().unwrap().spares = n;
                    println!("t={t_s:6.1}s base: {} airframes on the pads", hangar.len());
                }
                Command::Simulation(next) => {
                    *links.simulation.write().unwrap() = next;
                    println!("t={t_s:6.1}s simulation from the next mission on: {next:?}");
                }
                Command::SetRotation(on) => {
                    links.dock.write().unwrap().rotation = on;
                    println!("t={t_s:6.1}s base: charged airframes {} drones down to charge", if on { "take over from" } else { "don't take over from" });
                }
                Command::SetField(size, launch) => {
                    // The drones are told the field when they launch: a new one is a new mission.
                    let size = Vec2::new(size.x.clamp(MIN_FIELD, MAX_FIELD), size.y.clamp(MIN_FIELD, MAX_FIELD));
                    println!("t={t_s:6.1}s new field: {:.0} x {:.0} m", size.x, size.y);
                    return Some((links.command_rx, Ended::NewField(size, launch)));
                }
                Command::AddTarget(at) => {
                    if targets.len() >= MAX_TARGETS {
                        continue;
                    }
                    let pos = at.map_or_else(|| random_point(&mut rng, half), |p| clamp_to_field(p, half));
                    let target = Target::new(&mut next_target_id, pos, &mut rng, t_s);
                    println!("t={t_s:6.1}s target {} added at {pos:?}", target.id);
                    targets.push(target);
                }
                Command::RemoveTarget(id) => {
                    if let Some(k) = targets.iter().position(|t| t.id == id) {
                        targets.remove(k);
                        // Its gauge series would otherwise keep showing the last value.
                        links.telemetry.target_drones_near.record(0, &[opentelemetry::KeyValue::new("target", id as i64)]);
                        println!("t={t_s:6.1}s target {id} removed");
                    }
                }
                // Without a spot: one more in the base's launch queue. With one: dropped in right
                // there, already flying — a shortcut for setting up scenarios.
                Command::AddDrone(None) => launch_queue = launch_queue.saturating_add(1).min(MAX_DRONES as u32),
                Command::AddDrone(Some(at)) => {
                    if drones.iter().filter(|d| d.present).count() >= MAX_DRONES || drones.len() >= MAX_SLOTS {
                        continue;
                    }
                    let at = clamp_to_field(at, half);
                    let i = launch(&mut drones, &mut positions, &mut physics, &links.fleet, at, false);
                    println!("t={t_s:6.1}s drone {} placed at {at:?}", DroneSlot::sysid(i));
                }
                Command::Launch(count) => {
                    // Never more queued than could ever fly at once.
                    launch_queue = launch_queue.saturating_add(count).min(MAX_DRONES as u32);
                    println!("t={t_s:6.1}s base: {count} more to launch ({launch_queue} queued)");
                }
                Command::RemoveDrone(sysid) => {
                    let Some(i) = slot_of(sysid, &drones).filter(|&i| drones[i].present) else { continue };
                    let was_flying = drones[i].alive;
                    let d = &mut drones[i];
                    d.present = false;
                    d.alive = false;
                    d.sensor_tx = None;
                    d.velocity = Vec2::ZERO;
                    physics.remove_drone(i);
                    positions[i] = GONE;
                    // A wreck being cleared away isn't news to anyone; a flying drone leaving is.
                    if was_flying {
                        links.telemetry.drone_left(sysid, time_ms);
                    }
                    println!("t={t_s:6.1}s drone {sysid} removed");
                }
                Command::Lie(sysid) => {
                    let Some(i) = slot_of(sysid, &drones).filter(|&i| drones[i].flying() && !drones[i].liar) else { continue };
                    // Put the fake target well away from the real ones, so a swarm that
                    // believes it visibly flies the wrong way.
                    let mut fake_pos = random_point(&mut rng, half);
                    for _ in 0..50 {
                        if targets.iter().all(|t| (fake_pos - t.pos).length() >= 8.0) {
                            break;
                        }
                        fake_pos = random_point(&mut rng, half);
                    }
                    if drones[i].fault_tx.try_send(Fault::Lie { fake_pos }).is_ok() {
                        drones[i].liar = true;
                        drones[i].fake_target = Some(fake_pos);
                        links.telemetry.drone_compromised(sysid);
                        println!("t={t_s:6.1}s drone {sysid} compromised: reporting a fake target at {fake_pos:?}");
                    }
                }
                Command::Jam(at) => {
                    let at = at.map_or_else(|| random_point(&mut rng, half), |p| clamp_to_field(p, half));
                    jammers.push((at, JAMMER_RADIUS));
                    println!("t={t_s:6.1}s GPS jammer switched on at {at:?}, {JAMMER_RADIUS} m");
                }
                Command::ClearJamming => {
                    jammers.clear();
                    println!("t={t_s:6.1}s GPS jammers off");
                }
                Command::Spoof(sysid) => {
                    let Some(i) = slot_of(sysid, &drones).filter(|&i| drones[i].flying()) else { continue };
                    let d = &mut drones[i];
                    d.spoof = match d.spoof {
                        Some(_) => None,
                        None => Some((random_heading(&mut rng), t_s)),
                    };
                    println!("t={t_s:6.1}s drone {sysid} GPS spoofing {}", if d.spoof.is_some() { "on" } else { "off" });
                }
                Command::Wind(wind) => {
                    physics.wind = wind;
                    links.telemetry.wind_speed.record(wind.length() as f64, &[]);
                    println!("t={t_s:6.1}s wind set to {wind:?} ({:.1} m/s)", wind.length());
                }
                Command::Kill(_) | Command::KillRandom => {
                    let victim = match command {
                        Command::Kill(sysid) => slot_of(sysid, &drones).filter(|&i| drones[i].flying()),
                        _ => {
                            let candidates: Vec<usize> = (0..drones.len()).filter(|&i| drones[i].flying()).collect();
                            (!candidates.is_empty()).then(|| candidates[rng.random_range(0..candidates.len())])
                        }
                    };
                    let Some(i) = victim else { continue };
                    let d = &mut drones[i];
                    d.alive = false;
                    d.sensor_tx = None;
                    d.velocity = Vec2::ZERO;
                    d.target = None;
                    d.informed = false;
                    d.isolated = false;
                    physics.disarm(i);
                    let sysid = DroneSlot::sysid(i);
                    links.telemetry.drone_failed(sysid, time_ms);
                    println!("t={t_s:6.1}s drone {sysid} killed");
                }
            }
        }
        // The base launches the next queued drone from a free pad.
        let present = drones.iter().filter(|d| d.present).count();
        if launch_queue > 0 && t_s >= next_launch_s && present < MAX_DRONES && drones.len() < MAX_SLOTS {
            if let Some(pad) = free_pad(cfg.base, half, &positions, &drones, &hangar) {
                let i = launch(&mut drones, &mut positions, &mut physics, &links.fleet, pad, true);
                launch_queue -= 1;
                next_launch_s = t_s + cfg.launch_interval.as_secs_f32();
                println!("t={t_s:6.1}s drone {} took off from the base", DroneSlot::sysid(i));
            }
        }
        // A drone back on the pads to charge: the most charged airframe there takes off in its
        // place, as it (its system ID and keys), once one is charged enough.
        if t_s >= next_launch_s && links.dock.read().unwrap().rotation {
            let vacant = (0..drones.len()).find(|&i| drones[i].vacant);
            let ready = (0..hangar.len()).filter(|&k| hangar[k].battery >= READY_CHARGE).max_by(|&a, &b| hangar[a].battery.total_cmp(&hangar[b].battery));
            if let (Some(i), Some(k)) = (vacant, ready) {
                let parked = hangar.remove(k);
                physics.relaunch(i, parked.pad);
                positions[i] = parked.pad;
                drones[i] = DroneSlot::new(links.fleet.launch(i), parked.pad, parked.battery);
                next_launch_s = t_s + cfg.launch_interval.as_secs_f32();
                println!("t={t_s:6.1}s drone {} took off from the base again, on a charged airframe ({:.0}%)", DroneSlot::sysid(i), 100.0 * parked.battery);
            }
        }

        // Drones back from a return-to-launch, or brought down by their autopilot's failsafe:
        // once down, they leave the mission.
        for i in 0..drones.len() {
            if drones[i].landing && drones[i].present && physics.landed(i) {
                let d = &mut drones[i];
                d.present = false;
                d.alive = false;
                d.landing = false;
                d.sensor_tx = None;
                physics.remove_drone(i);
                positions[i] = GONE;
                let sysid = DroneSlot::sysid(i);
                links.telemetry.drone_left(sysid, time_ms);
                // On a pad, the airframe stays there, charging. Back to charge, its system ID
                // waits for a charged one; sent home, it's out of the mission.
                if let Some((pad, _)) = d.docking.take() {
                    hangar.push(Parked { pad, battery: d.battery });
                    d.vacant = d.to_charge;
                    if d.to_charge {
                        println!("t={t_s:6.1}s drone {sysid} landed on a pad to charge ({:.0}% left)", 100.0 * d.battery);
                    } else {
                        println!("t={t_s:6.1}s drone {sysid} landed on a pad at the base");
                    }
                } else if d.failsafe {
                    println!("t={t_s:6.1}s drone {sysid} landed by its autopilot, its brain silent");
                } else {
                    println!("t={t_s:6.1}s drone {sysid} landed at the base");
                }
            }
        }

        let flying_count = drones.iter().filter(|d| d.flying()).count();

        for t in targets.iter_mut() {
            if rng.random_bool(cfg.target_turn_chance) {
                t.heading = random_heading(&mut rng);
            }
            t.pos = bounce_within_field(t.pos + t.heading * cfg.target_speed * dt, &mut t.heading, half);
        }

        // Sense: each live drone gets its own position and velocity from its GPS, and, if close
        // enough, the targets — measured relative to itself (a camera or rangefinder knows where
        // a target is from the drone, not on the map), so its GPS error carries over to them.
        gps.advance(dt);
        // What each drone's GPS says, for its autopilot too (see `OFFBOARD_LOSS_S`).
        let mut fixes: Vec<Option<Vec2>> = vec![None; drones.len()];
        // try_send so a drone that stopped consuming readings can't stall the whole world.
        for (i, d) in drones.iter().enumerate() {
            let Some(tx) = &d.sensor_tx else { continue };
            let (mut fix, _) = gps.reading(i, positions[i], physics.velocity(i), t_s);
            // A spoofer drags the fix off slowly, so the drone doesn't notice a jump.
            if let Some((dir, since)) = d.spoof {
                fix = fix + dir * (SPOOF_DRAG_SPEED * (t_s - since)).min(SPOOF_MAX_OFFSET);
            }
            let jammed = jammers.iter().any(|&(at, r)| (positions[i] - at).length() <= r);
            fixes[i] = (!jammed).then_some(fix);
            // Velocity from the drone's own sensors, not GPS: there when jammed, a bit off.
            let own_velocity = physics.velocity(i) + d.flow_bias;
            let mut targets_seen = [None; MAX_SEEN];
            let in_range = targets.iter().map(|t| t.pos).filter(|&t| (t - positions[i]).length() <= cfg.sensor_range);
            for (slot, t) in targets_seen.iter_mut().zip(in_range) {
                *slot = Some(t - positions[i]);
            }
            // Relative to this drone, from its own sensors: no GPS in it.
            let nearby = cfg.proximity_range.map(|range| {
                let mut close: Vec<(f32, u8, Vec2, Vec2)> = drones
                    .iter()
                    .enumerate()
                    .filter(|&(j, d)| j != i && d.present)
                    .map(|(j, _)| (DroneSlot::sysid(j), positions[j] - positions[i], physics.velocity(j) - physics.velocity(i)))
                    .filter(|(_, rel, _)| rel.length() <= range)
                    .map(|(id, rel, vel)| (rel.length(), id, rel, vel))
                    .collect();
                close.sort_by(|a, b| a.0.total_cmp(&b.0));
                let mut out = [None; MAX_NEARBY];
                for (slot, (_, id, rel, vel)) in out.iter_mut().zip(close) {
                    *slot = Some((id, rel, vel));
                }
                out
            });
            let _ = tx.try_send(SensorReading {
                time_ms,
                own_pos: fix,
                gps_fix: !jammed,
                own_velocity,
                targets_seen,
                nearby,
                battery: if cfg.battery.is_some() { d.battery } else { f32::NAN },
            });
        }

        // Act: collect this tick's commands, giving brains half a tick to answer.
        let mut reported = vec![false; drones.len()];
        // Brains back over the base asking to land.
        let mut to_land = Vec::new();
        let mut got = 0;
        let deadline = tokio::time::sleep(cfg.tick / 2);
        tokio::pin!(deadline);
        while got < flying_count {
            tokio::select! {
                report = links.report_rx.recv() => {
                    let Some(r) = report else { break };
                    // Ignore stale reports, and late ones from a drone killed or removed mid-tick.
                    let Some(d) = drones.get_mut(r.index) else { continue };
                    if r.time_ms == time_ms && d.flying() && !reported[r.index] {
                        reported[r.index] = true;
                        // Checked here too, not only in the brain: a board's command comes in
                        // over a wire, and one NaN in the physics would wreck the whole world.
                        d.velocity = if r.velocity.is_finite() { r.velocity } else { Vec2::ZERO };
                        d.informed = r.informed;
                        d.known = r.known;
                        d.role = r.role;
                        d.isolated = r.isolated;
                        d.mode = r.mode;
                        d.target = r.target;
                        d.believed = r.position;
                        d.nav = r.nav;
                        d.to_charge = r.to_charge;
                        if r.land && !d.landing {
                            d.landing = true;
                            to_land.push(r.index);
                        }
                        got += 1;
                    }
                }
                _ = &mut deadline => break,
            }
        }
        // A live drone whose brain didn't answer in time is told to hold position rather than
        // keep its last command (its autopilot brakes it); a dead one's command doesn't matter.
        // Not a word for `OFFBOARD_LOSS_S`, and the autopilot flies it home.
        for (i, d) in drones.iter_mut().enumerate() {
            if reported[i] && !d.failsafe {
                d.silent_since_s = None;
                continue;
            }
            d.velocity = Vec2::ZERO;
            if !d.flying() || d.landing {
                continue;
            }
            let since = *d.silent_since_s.get_or_insert(t_s);
            if !d.failsafe && t_s - since >= OFFBOARD_LOSS_S {
                d.failsafe = true;
                links.telemetry.offboard_lost(DroneSlot::sysid(i));
                let action = if fixes[i].is_some() { "flying home" } else { "no GPS fix, landing where it is" };
                println!("t={t_s:6.1}s drone {}: no command from its brain for {OFFBOARD_LOSS_S} s, {action}", DroneSlot::sysid(i));
            }
            if d.failsafe {
                // Once it's given up on the brain, the autopilot sees it through, as PX4 does,
                // even if the brain speaks up again.
                match fixes[i] {
                    Some(at) if (cfg.base - at).length() > FAILSAFE_HOME_M => {
                        let to_base = cfg.base - at;
                        d.velocity = to_base * (FAILSAFE_SPEED.min(to_base.length()) / to_base.length());
                    }
                    _ => {
                        d.landing = true;
                        physics.land(i);
                    }
                }
            }
        }
        if got < flying_count {
            links.telemetry.world_missed_reports.add((flying_count - got) as u64, &[]);
        }
        // A drone landing by the base lands on the free pad nearest it: the base guides it over
        // the pad, then it descends (to charge, with batteries). Far off, where it is.
        for i in to_land {
            let pad = ((positions[i] - cfg.base).length() <= DOCK_ZONE_M).then(|| nearest_free_pad(cfg.base, half, positions[i], &positions, &drones, &hangar, i)).flatten();
            match pad {
                Some(pad) => drones[i].docking = Some((pad, false)),
                None => physics.land(i),
            }
        }
        for (i, d) in drones.iter_mut().enumerate() {
            let Some((pad, false)) = d.docking.filter(|_| d.flying()) else { continue };
            let to = pad - positions[i];
            if to.length() <= DOCK_ALIGN_M {
                d.velocity = Vec2::ZERO;
                d.docking = Some((pad, true));
                physics.land(i);
            } else {
                d.velocity = to * (FAILSAFE_SPEED.min(to.length()) / to.length());
            }
        }

        // The brains asked for velocities; the airframes get there as fast as physics allows.
        let setpoints: Vec<Vec2> = drones.iter().map(|d| d.velocity).collect();
        for (a, b) in physics.step(dt, &setpoints) {
            links.telemetry.collisions.add(1, &[]);
            println!(
                "t={t_s:6.1}s drones {} and {} collided at {:.1} m/s ({} and {})",
                DroneSlot::sysid(a),
                DroneSlot::sysid(b),
                physics.relative_speed(a, b),
                mode_name(drones[a].mode),
                mode_name(drones[b].mode)
            );
        }
        for (i, d) in drones.iter().enumerate() {
            positions[i] = if d.present { physics.position(i) } else { GONE };
        }
        links.positions.write().unwrap().clone_from(&positions);
        // Batteries: down in the air (faster the faster it flies), up on the pads. One that runs
        // out in the air falls.
        if let Some(battery) = cfg.battery {
            for (i, d) in drones.iter_mut().enumerate() {
                if !d.flying() || physics.altitude(i) < 0.3 {
                    continue;
                }
                let speed = physics.velocity(i).length();
                d.battery -= dt / battery.flight_s * (1.0 + 0.25 * (speed / 2.0) * (speed / 2.0));
                if d.battery <= 0.0 {
                    d.battery = 0.0;
                    d.alive = false;
                    d.sensor_tx = None;
                    d.velocity = Vec2::ZERO;
                    d.docking = None;
                    physics.disarm(i);
                    let sysid = DroneSlot::sysid(i);
                    links.telemetry.drone_failed(sysid, time_ms);
                    println!("t={t_s:6.1}s drone {sysid}: battery empty, it fell");
                }
            }
            for parked in hangar.iter_mut() {
                parked.battery = (parked.battery + dt / battery.charge_s).min(1.0);
            }
        }
        {
            let mut dock = links.dock.write().unwrap();
            dock.parked = hangar.len() as u32;
            dock.ready = hangar.iter().filter(|p| p.battery >= READY_CHARGE).count() as u32;
            dock.waiting = drones.iter().filter(|d| d.vacant).count() as u32;
            dock.lowest_charge = cfg.battery.and_then(|_| drones.iter().filter(|d| d.flying()).map(|d| d.battery).min_by(f32::total_cmp));
        }

        let flying = || drones.iter().enumerate().filter(|(_, d)| d.flying());
        let informed_count = flying().filter(|(_, d)| d.informed).count();
        let isolated_count = flying().filter(|(_, d)| d.isolated).count();
        // Per target: closest drone, and how many drones are near it.
        let mut nearest_overall = f32::INFINITY;
        let mut covered = 0;
        for target in &targets {
            let label = [opentelemetry::KeyValue::new("target", target.id as i64)];
            let distances = flying().map(|(i, _)| (positions[i] - target.pos).length());
            let nearest = distances.clone().fold(f32::INFINITY, f32::min);
            let near = distances.filter(|&d| d <= 4.0).count();
            if nearest.is_finite() {
                links.telemetry.target_nearest_agent_distance.record(nearest as f64, &label);
            }
            links.telemetry.target_drones_near.record(near as u64, &label);
            covered += (near > 0) as u64;
            nearest_overall = nearest_overall.min(nearest);
        }
        links.telemetry.targets_covered.record(covered, &[]);

        for (i, _) in flying() {
            for (c, at) in visited_at.iter_mut().enumerate() {
                let center = Vec2::new(-half.x + ((c / GRID) as f32 + 0.5) * cell_size.x, -half.y + ((c % GRID) as f32 + 0.5) * cell_size.y);
                if (center - positions[i]).length() <= cfg.sensor_range {
                    *at = t_s;
                }
            }
        }
        // Who can the ground station at the base reach, hop by hop?
        let mut reached = vec![false; drones.len()];
        let mut frontier = vec![cfg.base];
        while let Some(p) = frontier.pop() {
            for (i, _) in flying() {
                if !reached[i] && (positions[i] - p).length() <= cfg.radio_range {
                    reached[i] = true;
                    frontier.push(positions[i]);
                }
            }
        }
        if flying_count > 0 {
            let connected = reached.iter().filter(|&&r| r).count();
            links.telemetry.link_connected.record(connected as f64 / flying_count as f64, &[]);
        }
        let recent = visited_at.iter().filter(|&&at| t_s - at <= cfg.recent_coverage_s).count();
        links.telemetry.search_coverage.record(recent as f64 / visited_at.len() as f64, &[]);
        for target in targets.iter_mut().filter(|t| !t.found) {
            if flying().any(|(i, _)| (positions[i] - target.pos).length() <= cfg.sensor_range) {
                target.found = true;
                let took = t_s - target.added_at_s;
                links.telemetry.time_to_find.record(took as f64, &[]);
                println!("t={t_s:6.1}s target {} found after {took:.1}s", target.id);
            }
        }
        let errors: Vec<f32> = flying().filter_map(|(i, _)| gps.error(i, positions[i])).collect();
        if !errors.is_empty() {
            links.telemetry.gps_error.record(errors.iter().sum::<f32>() as f64 / errors.len() as f64, &[]);
        }
        // How wrong the drones are about where they are, whatever their position comes from.
        let nav_errors: Vec<f32> = flying().map(|(i, d)| (d.believed - positions[i]).length()).collect();
        if !nav_errors.is_empty() {
            links.telemetry.nav_error.record(nav_errors.iter().sum::<f32>() as f64 / nav_errors.len() as f64, &[]);
        }
        for nav in [Nav::Gps, Nav::Neighbors, Nav::DeadReckoning] {
            let count = flying().filter(|(_, d)| d.nav == nav).count();
            links.telemetry.agents_by_nav.record(count as u64, &[opentelemetry::KeyValue::new("nav", nav_name(nav))]);
        }
        let present_count = drones.iter().filter(|d| d.present).count();
        links.telemetry.agents.record(present_count as u64, &[]);
        links.telemetry.agents_alive.record(flying_count as u64, &[]);
        links.telemetry.agents_informed.record(informed_count as u64, &[]);
        links.telemetry.agents_isolated.record(isolated_count as u64, &[]);
        for (i, _) in flying() {
            let drone = opentelemetry::KeyValue::new("drone.sysid", DroneSlot::sysid(i) as i64);
            links.telemetry.tilt.record(physics.tilt_degrees(i) as f64, &[drone]);
        }
        for mode in [Mode::Searching, Mode::Investigating, Mode::Converging, Mode::Operator] {
            let count = flying().filter(|(_, d)| d.mode == mode).count();
            links
                .telemetry
                .agents_by_mode
                .record(count as u64, &[opentelemetry::KeyValue::new("mode", mode_name(mode))]);
        }

        let _ = links.ws_tx.send(SwarmState {
            seq,
            agents: drones
                .iter()
                .enumerate()
                .filter(|(_, d)| d.present)
                .map(|(i, d)| AgentState {
                    id: DroneSlot::sysid(i) as u32,
                    pos: positions[i],
                    informed: d.informed,
                    known: d.known.iter().flatten().map(|&(pos, confirmed)| KnownTarget { pos, confirmed }).collect(),
                    role: match d.role {
                        swarm_brain::Role::Relay => "relay",
                        swarm_brain::Role::Sweep => "sweep",
                        swarm_brain::Role::None => "",
                    },
                    alive: d.alive,
                    mode: mode_name(d.mode),
                    liar: d.liar,
                    alt: physics.altitude(i),
                    target_id: d.target,
                    believed: d.believed,
                    nav: nav_name(d.nav),
                    spoofed: d.spoof.is_some(),
                    battery: cfg.battery.map(|_| d.battery),
                })
                .collect(),
            parked: hangar.iter().map(|p| ParkedState { pos: p.pad, battery: p.battery }).collect(),
            simulation: *links.simulation.read().unwrap(),
            targets: targets.iter().map(|t| TargetState { id: t.id, pos: t.pos }).collect(),
            // A dead or departed liar no longer broadcasts its lie.
            fake_targets: drones.iter().filter(|d| d.flying()).filter_map(|d| d.fake_target).collect(),
            wind: physics.wind,
            base: cfg.base,
            field_half: half,
            radio_range: cfg.radio_range,
            jammers: jammers.iter().map(|&(pos, radius)| JammerState { pos, radius }).collect(),
            launch_queue,
            coverage_age: visited_at.iter().map(|&at| (t_s - at).min(60.0)).collect(),
        });

        if seq.is_multiple_of(40) {
            println!(
                "t={t_s:6.1}s alive={flying_count}/{present_count} informed={informed_count} isolated={isolated_count} nearest={nearest_overall:.1}m"
            );
        }
        seq = seq.wrapping_add(1);
    }
}

/// Starts a drone over `at` in the next slot (and so with the next system ID): on the ground
/// about to take off, or already hovering at cruise altitude.
fn launch(drones: &mut Vec<DroneSlot>, positions: &mut Vec<Vec2>, physics: &mut Physics, fleet: &Fleet, at: Vec2, on_ground: bool) -> usize {
    let i = drones.len();
    let launched = fleet.launch(i);
    let body = physics.add_drone(at, on_ground);
    debug_assert_eq!(body, i, "physics and world slots must line up");
    positions.push(at);
    drones.push(DroneSlot::new(launched, at, 1.0));
    i
}

impl DroneSlot {
    /// A drone just launched over `at`, its battery charged this far.
    fn new(launched: crate::fleet::LaunchedDrone, at: Vec2, battery: f32) -> Self {
        Self {
            sensor_tx: Some(launched.sensor_tx),
            fault_tx: launched.fault_tx,
            present: true,
            alive: true,
            velocity: Vec2::ZERO,
            informed: false,
            known: [None; swarm_brain::MAX_TRACKS],
            role: swarm_brain::Role::None,
            isolated: false,
            mode: Mode::Searching,
            liar: false,
            fake_target: None,
            target: None,
            landing: false,
            silent_since_s: None,
            failsafe: false,
            believed: at,
            nav: Nav::Gps,
            flow_bias: random_heading(&mut rand::rng()) * 0.08,
            spoof: None,
            battery,
            docking: None,
            to_charge: false,
            vacant: false,
        }
    }
}

/// How far apart the base's pads are, and how clear of other drones one must be for a launch:
/// closer, a drone taking off passes within a meter of the one just leaving, and launches are
/// where a big swarm is most crowded.
const PAD_SPACING_M: f32 = 2.5;

/// The base's pads: rows along the field's edge by the base, outside the field, as many to a
/// row as fit along it (up to `PADS_A_ROW`), each row a pad spacing further out. Inside, the
/// grid reached up to 40 m into the field: airframes charging where the mice run, and drones
/// taking off and landing across the lanes being searched.
fn pads(base: Vec2, half: Vec2) -> impl Iterator<Item = Vec2> {
    let inward = Vec2::new(-base.x.signum(), -base.y.signum());
    let a_row = ((2.0 * half.x / PAD_SPACING_M) as usize).clamp(1, PADS_A_ROW);
    let edge = -inward.y * half.y;
    (0..PADS).map(move |k| Vec2::new(base.x + inward.x * PAD_SPACING_M * (k % a_row) as f32, edge - inward.y * PAD_SPACING_M * (k / a_row + 1) as f32))
}

/// Whether no drone but `me` is sitting on `pad` or hovering over it, none is being guided onto
/// it, and no airframe is charging on it.
fn pad_free(pad: Vec2, positions: &[Vec2], drones: &[DroneSlot], hangar: &[Parked], me: Option<usize>) -> bool {
    let clear = |p: Vec2| (p - pad).length() >= PAD_SPACING_M;
    positions.iter().zip(drones).enumerate().all(|(j, (&p, d))| Some(j) == me || !d.present || clear(p))
        && drones.iter().all(|d| d.docking.is_none_or(|(at, _)| clear(at)))
        && hangar.iter().all(|parked| clear(parked.pad))
}

/// Puts charged airframes on the pads, or takes the least charged off, so there are `n`.
fn set_spares(hangar: &mut Vec<Parked>, n: u32, base: Vec2, half: Vec2, positions: &[Vec2], drones: &[DroneSlot]) {
    while hangar.len() > n as usize {
        let least = (0..hangar.len()).min_by(|&a, &b| hangar[a].battery.total_cmp(&hangar[b].battery)).expect("not empty");
        hangar.remove(least);
    }
    while hangar.len() < n as usize {
        let Some(pad) = free_pad(base, half, positions, drones, hangar) else { break };
        hangar.push(Parked { pad, battery: 1.0 });
    }
}

/// The first free pad, to launch from or park a spare on.
fn free_pad(base: Vec2, half: Vec2, positions: &[Vec2], drones: &[DroneSlot], hangar: &[Parked]) -> Option<Vec2> {
    pads(base, half).find(|&pad| pad_free(pad, positions, drones, hangar, None))
}

/// The free pad nearest `to`, for drone `me` to land on.
fn nearest_free_pad(base: Vec2, half: Vec2, to: Vec2, positions: &[Vec2], drones: &[DroneSlot], hangar: &[Parked], me: usize) -> Option<Vec2> {
    pads(base, half).filter(|&pad| pad_free(pad, positions, drones, hangar, Some(me))).min_by(|a, b| (*a - to).length().total_cmp(&(*b - to).length()))
}

/// The base's pads: room for the spares and drones back to charge as well as launches.
const PADS: usize = 64;
const PADS_A_ROW: usize = 16;

fn slot_of(sysid: u8, drones: &[DroneSlot]) -> Option<usize> {
    (sysid as usize).checked_sub(1).filter(|&i| i < drones.len())
}

fn clamp_to_field(p: Vec2, half: Vec2) -> Vec2 {
    Vec2::new(p.x.clamp(-half.x, half.x), p.y.clamp(-half.y, half.y))
}

/// Field sizes (each side, m) a mission can have.
pub const MIN_FIELD: f32 = 10.0;
pub const MAX_FIELD: f32 = 600.0;

/// A target in the field: ground truth only, never shown to the drones.
struct Target {
    id: u32,
    pos: Vec2,
    heading: Vec2,
    added_at_s: f32,
    /// Some drone has had it within sensor range at least once.
    found: bool,
}

impl Target {
    fn new(next_id: &mut u32, pos: Vec2, rng: &mut impl Rng, added_at_s: f32) -> Self {
        let id = *next_id;
        *next_id += 1;
        Self { id, pos, heading: random_heading(rng), added_at_s, found: false }
    }
}

/// How far a GPS jammer reaches.
const JAMMER_RADIUS: f32 = 7.0;
/// How fast a spoofer drags a drone's fix away, and how far at most.
const SPOOF_DRAG_SPEED: f32 = 0.3;
const SPOOF_MAX_OFFSET: f32 = 40.0;

fn nav_name(nav: Nav) -> &'static str {
    match nav {
        Nav::Gps => "gps",
        Nav::Neighbors => "neighbors",
        Nav::DeadReckoning => "dead_reckoning",
    }
}

fn mode_name(mode: Mode) -> &'static str {
    match mode {
        Mode::Searching => "searching",
        Mode::Investigating => "investigating",
        Mode::Converging => "converging",
        Mode::Operator => "operator",
    }
}

fn random_point(rng: &mut impl Rng, half: Vec2) -> Vec2 {
    Vec2::new(rng.random_range(-half.x..half.x), rng.random_range(-half.y..half.y))
}

fn random_heading(rng: &mut impl Rng) -> Vec2 {
    let angle = rng.random_range(0.0..std::f32::consts::TAU);
    Vec2::new(angle.cos(), angle.sin())
}

/// Reflects `pos` and flips the matching component of `heading` if `pos` has gone past the
/// field boundary, so the target bounces back in instead of leaving the field.
fn bounce_within_field(mut pos: Vec2, heading: &mut Vec2, half: Vec2) -> Vec2 {
    if pos.x.abs() > half.x {
        heading.x = -heading.x;
        pos.x = pos.x.clamp(-half.x, half.x);
    }
    if pos.y.abs() > half.y {
        heading.y = -heading.y;
        pos.y = pos.y.clamp(-half.y, half.y);
    }
    pos
}
