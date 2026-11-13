mod agent;
mod base;
mod body;
mod fleet;
mod gps;
mod hil;
mod netem;
mod observability;
mod physics;
mod world;
mod ws;

use std::sync::{Arc, RwLock};
use std::time::Duration;

use tokio::net::UdpSocket;

use fleet::Fleet;
use gps::GpsConfig;
use netem::NetemConfig;
use observability::Telemetry;
use swarm_brain::{BrainConfig, SearchStrategy, SwarmMode, SweepPattern, Vec2};
use world::{WorldConfig, WorldLinks};
use ws::Simulation;

const NUM_AGENTS: usize = 12;
const SENSOR_RANGE: f32 = 2.5;
/// How far a drone's onboard sensing (cameras, rangefinders, UWB) picks up other drones:
/// enough to see a drone closing at twice top speed a second ahead.
const PROXIMITY_RANGE: f32 = 5.0;
/// Mice in the field. With `responders` (2) drones per target, 12 drones cover all three and
/// keep six searching.
const NUM_TARGETS: usize = 3;
// Radio reaches well beyond what a drone can see, so news can travel through the swarm: with a
// radio of a few meters, drones on a 30 m field would seldom be close enough to relay anything.
const RADIO_RANGE: f32 = 8.0;
const TICK: Duration = Duration::from_millis(50); // 20 Hz

/// The board flies as this drone when `--hil` is given.
const HIL_SYSID: u8 = 1;
/// Must match firmware/src/main.rs BAUD. 100 KB/s each way: a crowd of drones around the board
/// offers more than 460800 baud (46 KB/s) carries. Divides exactly from the board's 54 MHz APB1
/// clock.
pub const HIL_BAUD: u32 = 1_000_000;

const NETEM_ADDR: &str = "127.0.0.1:14600";
/// The base link the operator's ground station uses to launch drones (see base.rs).
const BASE_ADDR: &str = "127.0.0.1:14650";
const VIEWER_ADDR: &str = "127.0.0.1:9002";

/// Swarm search-and-converge simulator. Each drone is its own task with its own UDP socket,
/// talking to the others only through MAVLink v2 frames routed by an emulated radio channel.
/// See world.rs for ground truth, netem.rs for the radio, agent.rs for a drone's brain.
///
/// `--hil <serial port>` flies drone 1 on the STM32 board instead (see hil.rs), e.g.
/// `cargo run -- --hil /dev/tty.usbmodem1103`. Any mavlink-core address works too, e.g.
/// `--hil tcpin:127.0.0.1:5760` for the host-side fake board (examples/fake_board.rs).
///
/// `--search random` makes the simulated drones search by random walk instead of the shared
/// coverage map, to compare the two (the board keeps its built-in default).
///
/// `--drones N` sets how many drones take off from the base at mission start (default 12; 0
/// to launch them all from the ground station instead).
///
/// `--gps on` gives drones GNSS fixes with realistic error instead of their true positions (off
/// by default).
///
/// `--proximity off` takes away the drones' onboard sensing of each other, so they avoid each
/// other by broadcast (GPS) positions alone.
///
/// `--mode connected`: the swarm always in contact with the base by relay chains, sweeping the
/// field in rays out from it (slower). By default it's `autonomous`: off the leash, sweeping the
/// field in formation, the quickest search, in contact with the base only near it. The ground
/// station switches it over the radio (see `SwarmMode`).
///
/// `--sweep formation|cells|radar`: how free drones look over the field, overriding the mode's
/// own (formation when autonomous, radar when connected), to compare them.
///
/// `--field 40x30`: a field 40 m wide and 30 m high (30x30 by default), the base in its lower
/// left corner. The viewer or the ground station can change it, for the next mission.
///
/// `--battery-min 20` (12 by default): drones have batteries a full charge of which flies them 20
/// minutes (less flying fast). Each goes back to the base to charge in time, by its own
/// reckoning, and lands on a free pad; once an airframe on the pads is charged, it takes off in
/// its place, flying as it. `--battery-min off`: drones never run down.
/// `--charge-min 40` (2 by default): how long a pad takes to charge one from empty.
/// `--spares 8`: charged airframes waiting on the pads besides those launched (none by default).
///
/// `--gps`, `--proximity`, `--battery-min`, `--charge-min` and `--search` set how the first
/// mission is simulated; the viewer's Simulation panel changes it for the next ones.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let hil_port = match args.iter().position(|a| a == "--hil") {
        Some(i) => Some(args.get(i + 1).ok_or("--hil needs a serial port, e.g. /dev/tty.usbmodem1103")?.clone()),
        None => None,
    };
    let search = match args.iter().position(|a| a == "--search").map(|i| args.get(i + 1).map(String::as_str)) {
        None | Some(Some("coverage")) => SearchStrategy::Coverage,
        Some(Some("random")) => SearchStrategy::RandomWalk,
        _ => return Err("--search takes `coverage` or `random`".into()),
    };
    let mode = match args.iter().position(|a| a == "--mode").map(|i| args.get(i + 1).map(String::as_str)) {
        None | Some(Some("autonomous")) => SwarmMode::Autonomous,
        Some(Some("connected")) => SwarmMode::Connected,
        _ => return Err("--mode takes `autonomous` or `connected`".into()),
    };
    let field = match args.iter().position(|a| a == "--field") {
        Some(i) => args
            .get(i + 1)
            .and_then(|f| f.split_once('x'))
            .and_then(|(w, h)| Some(Vec2::new(w.parse().ok()?, h.parse().ok()?)))
            .ok_or("--field takes width x height in meters, e.g. 40x30")?,
        None => Vec2::new(30.0, 30.0),
    };
    let field_half = clamp_field(field) * 0.5;
    let mut brain_cfg = BrainConfig { search, field_half, home: base_of(field_half), ..BrainConfig::SWARM }.with_mode(mode);
    // Overrides the mode's own sweep pattern, to compare them (a mode change from the ground
    // station sets the mode's again).
    match args.iter().position(|a| a == "--sweep").map(|i| args.get(i + 1).map(String::as_str)) {
        None => {}
        Some(Some("formation")) => brain_cfg.sweep_pattern = SweepPattern::Formation,
        Some(Some("cells")) => brain_cfg.sweep_pattern = SweepPattern::Cells,
        Some(Some("radar")) => brain_cfg.sweep_pattern = SweepPattern::Radar,
        _ => return Err("--sweep takes `formation`, `cells` or `radar`".into()),
    }
    let gps = match args.iter().position(|a| a == "--gps").map(|i| args.get(i + 1).map(String::as_str)) {
        Some(Some("on")) => true,
        None | Some(Some("off")) => false,
        _ => return Err("--gps takes `on` or `off`".into()),
    };
    let proximity = match args.iter().position(|a| a == "--proximity").map(|i| args.get(i + 1).map(String::as_str)) {
        None | Some(Some("on")) => true,
        Some(Some("off")) => false,
        _ => return Err("--proximity takes `on` or `off`".into()),
    };
    println!("mode: {mode:?} (the ground station can switch it)");
    let num_agents = match args.iter().position(|a| a == "--drones") {
        Some(i) => args.get(i + 1).and_then(|n| n.parse().ok()).ok_or("--drones takes a number")?,
        None => NUM_AGENTS,
    };
    let minutes = |flag: &str| -> Result<Option<f32>, Box<dyn std::error::Error>> {
        match args.iter().position(|a| a == flag) {
            Some(i) => Ok(Some(args.get(i + 1).and_then(|n| n.parse::<f32>().ok()).filter(|m| *m > 0.0).ok_or(format!("{flag} takes minutes, e.g. 20"))?)),
            None => Ok(None),
        }
    };
    let battery_min = match args.iter().position(|a| a == "--battery-min").map(|i| args.get(i + 1).map(String::as_str)) {
        Some(Some("off")) => None,
        _ => Some(minutes("--battery-min")?.unwrap_or(12.0)),
    };
    let spares = match args.iter().position(|a| a == "--spares") {
        Some(i) => args.get(i + 1).and_then(|n| n.parse().ok()).ok_or("--spares takes a number")?,
        None => 0,
    };
    // The first mission's: the viewer can change it for the next ones.
    let simulation = Simulation {
        gps,
        proximity,
        battery_min,
        charge_min: minutes("--charge-min")?.unwrap_or(2.0),
        random_search: search == SearchStrategy::RandomWalk,
    };

    // Telemetry must be set up before the runtime starts (see observability::init).
    let (telemetry_guard, telemetry) = observability::init()?;

    let runtime = tokio::runtime::Runtime::new()?;
    let result = runtime.block_on(run(Arc::new(telemetry), hil_port, brain_cfg, num_agents, simulation, spares));
    // Drop the runtime first so in-flight tasks stop recording, then flush what's buffered.
    drop(runtime);
    telemetry_guard.shutdown();
    result
}

/// Where the base is on a field of this half-size: in its lower left corner, 2 m in.
fn base_of(field_half: Vec2) -> Vec2 {
    Vec2::new(2.0 - field_half.x, 2.0 - field_half.y)
}

fn clamp_field(size: Vec2) -> Vec2 {
    Vec2::new(size.x.clamp(world::MIN_FIELD, world::MAX_FIELD), size.y.clamp(world::MIN_FIELD, world::MAX_FIELD))
}

async fn run(
    telemetry: Arc<Telemetry>,
    hil_port: Option<String>,
    brain_cfg: BrainConfig,
    num_agents: usize,
    simulation: Simulation,
    spares: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    let (ws_tx, command_tx, command_rx) = ws::spawn_server(VIEWER_ADDR, 64);
    // The launch base's certificate authority, new every run (see `swarm_brain::auth`).
    let authority = Arc::new(swarm_brain::auth::Authority::from_secret(rand::random()));
    let mission_state = Arc::new(RwLock::new(base::Mission::default()));
    // The base's pads: the operator sets them over the base link, the world keeps what's on them.
    let dock = Arc::new(RwLock::new(world::Dock::new(spares as u32)));
    let base_socket = UdpSocket::bind(BASE_ADDR).await?;
    tokio::spawn(base::run(base_socket, command_tx, authority.clone(), mission_state.clone(), dock.clone()));
    // How the world is simulated: the viewer sets it for the next mission.
    let simulation = Arc::new(RwLock::new(simulation));
    println!("base: launch link on udp://{BASE_ADDR}");
    let positions = Arc::new(RwLock::new(vec![Vec2::ZERO; NUM_AGENTS]));
    telemetry.set_ground_truth(positions.clone(), RADIO_RANGE);

    let netem_socket = UdpSocket::bind(NETEM_ADDR).await?;
    let netem_addr = netem_socket.local_addr()?;
    let netem_cfg = NetemConfig {
        range: RADIO_RANGE,
        base_latency: Duration::from_millis(30),
        jitter: Duration::from_millis(70),
        drop_chance: 0.1,
    };
    // The ground station's radio is at the base.
    // Shared: a new field moves the base (and its radio) to the new field's corner.
    let stations = Arc::new(RwLock::new(vec![(swarm_brain::GCS_SYSID, brain_cfg.home), (swarm_brain::BASE_SYSID, brain_cfg.home)]));
    // Counts missions, for the radio emulator to forget the last one's radios, and for the
    // drones' keys: each mission's are good for that mission only.
    let mission = Arc::new(std::sync::atomic::AtomicU64::new(0));
    tokio::spawn(netem::run(netem_socket, netem_cfg, positions.clone(), stations.clone(), mission.clone(), telemetry.clone()));
    tokio::spawn(base::beacon(UdpSocket::bind("127.0.0.1:0").await?, netem_addr, authority.clone(), mission_state.clone()));
    println!("netem: radio channel on udp://{netem_addr}");

    let world_cfg = WorldConfig {
        num_agents,
        base: brain_cfg.home,
        launch_interval: Duration::from_secs(1),
        field_half: brain_cfg.field_half,
        sensor_range: SENSOR_RANGE,
        radio_range: RADIO_RANGE,
        gps: GpsConfig::CONSUMER,
        proximity_range: None,
        recent_coverage_s: 30.0,
        num_targets: NUM_TARGETS,
        target_speed: 0.8,
        target_turn_chance: 0.02,
        tick: TICK,
        battery: None,
    };

    // One iteration per mission. A reset from the viewer ends the world, which drops every
    // drone's sensor feed and so ends every drone's task; the next iteration starts a fresh
    // swarm. The viewer connection, the radio emulator and telemetry carry on throughout.
    let mut command_rx = command_rx;
    let (mut brain_cfg, mut world_cfg) = (brain_cfg, world_cfg);
    // The viewer can switch it at any time; it carries over to the next mission.
    loop {
        // The board flies as HIL_SYSID; a bare device path means its serial port.
        let hil = hil_port.as_deref().map(|port| {
            let address = if port.starts_with('/') { format!("serial:{port}:{HIL_BAUD}") } else { port.to_string() };
            (address, HIL_SYSID)
        });
        let mission_number = mission.load(std::sync::atomic::Ordering::Relaxed) as u32;
        // The world as the viewer last set it (the flags, to begin with).
        let sim = *simulation.read().unwrap();
        world_cfg.gps = GpsConfig { enabled: sim.gps, ..GpsConfig::CONSUMER };
        world_cfg.proximity_range = sim.proximity.then_some(PROXIMITY_RANGE);
        world_cfg.battery = sim.battery_min.map(|flight| world::Battery { flight_s: flight * 60.0, charge_s: sim.charge_min * 60.0 });
        brain_cfg.search = if sim.random_search { SearchStrategy::RandomWalk } else { SearchStrategy::Coverage };
        println!("simulation: {sim:?}");
        if let Some(b) = world_cfg.battery {
            // Kept up for good, each drone in the air needs charge_s / flight_s more charging on
            // the pads, besides the one flying (a little more: drones come back with some left).
            println!("to keep {} flying for good, about {:.0} spares", world_cfg.num_agents, (world_cfg.num_agents as f32 * b.charge_s / b.flight_s).ceil());
        }
        // The drones answer for mice in home stretches split by how many there are (see the
        // brain's `home_stretch`): the ones launched at the start.
        brain_cfg.fleet = world_cfg.num_agents as u16;
        let (fleet, report_rx) = Fleet::new(telemetry.clone(), netem_addr, brain_cfg, hil, authority.clone(), mission_number);
        let started = std::time::Instant::now();
        *mission_state.write().unwrap() = base::Mission { number: mission_number, started, revoked: Vec::new() };
        let world_links = WorldLinks {
            fleet,
            report_rx,
            positions: positions.clone(),
            ws_tx: ws_tx.clone(),
            command_rx,
            telemetry: telemetry.clone(),
            dock: dock.clone(),
            simulation: simulation.clone(),
            started,
        };

        // Run until Ctrl+C, so main can flush telemetry on the way out.
        tokio::select! {
            ended = world::run(world_cfg, world_links) => match ended {
                Some((rx, ended)) => {
                    command_rx = rx;
                    telemetry.reset();
                    mission.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    // A reset flies the same fleet again. (A new field from the ground
                    // station launches nobody: without this, resetting after one restarted
                    // with no drones at all.)
                    if let world::Ended::Reset(fleet) = ended {
                        if fleet > 0 {
                            world_cfg.num_agents = fleet as usize;
                        }
                    }
                    if let world::Ended::NewField(size, launch) = ended {
                        if let Some(n) = launch {
                            world_cfg.num_agents = n as usize;
                        }
                        // The base stays in the field's corner, and so does its radio.
                        let half = clamp_field(size) * 0.5;
                        brain_cfg.field_half = half;
                        brain_cfg.home = base_of(half);
                        world_cfg.field_half = half;
                        world_cfg.base = brain_cfg.home;
                        for station in stations.write().unwrap().iter_mut() {
                            station.1 = brain_cfg.home;
                        }
                    }
                    println!("reset: starting a new mission");
                }
                None => break,
            },
            _ = tokio::signal::ctrl_c() => {
                println!("shutting down");
                break;
            }
        }
    }
    Ok(())
}
