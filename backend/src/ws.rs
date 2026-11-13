//! WebSocket server for the viewer, in both directions:
//!  - outbound: the world pushes one `SwarmState` per tick into a `broadcast` channel, fanned
//!    out as JSON to every connected viewer.
//!  - inbound: a viewer can send `Command`s as JSON (e.g. `{"kill":3}` or `"kill_random"`) to
//!    inject faults or change the weather, forwarded to the world over an mpsc channel.
//!
//! Like the rest of the viewer side, none of this is visible to the drones themselves.

use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::{broadcast, mpsc};

use swarm_brain::Vec2;

#[derive(Clone, Serialize)]
pub struct AgentState {
    pub id: u32,
    pub pos: Vec2,
    /// Whether this agent currently has *any* information about the target (directly seen or
    /// relayed), so the viewer can color "informed" vs. "still searching" agents differently.
    pub informed: bool,
    /// The targets it knows of: where it thinks each is, and whether it counts it confirmed.
    pub known: Vec<KnownTarget>,
    /// "relay" (in a chain out to a ring), "sweep" (on the sweep), or "".
    pub role: &'static str,
    pub alive: bool,
    /// "searching" | "investigating" | "converging" — see `agent::Mode`.
    pub mode: &'static str,
    /// Its radio was compromised and is broadcasting a fake target.
    pub liar: bool,
    /// Swarm-wide ID of the target it's responding to, if any.
    pub target_id: Option<u16>,
    /// Meters above the ground.
    pub alt: f32,
    /// Where its brain thinks it is (`pos` is where it really is), and where that came from:
    /// "gps" | "neighbors" | "dead_reckoning".
    pub believed: Vec2,
    pub nav: &'static str,
    /// Its GPS is being spoofed.
    pub spoofed: bool,
    /// Its battery's charge, 0 to 1 (None: the simulation runs without batteries).
    pub battery: Option<f32>,
}

/// An airframe on one of the base's pads, charging.
#[derive(Clone, Serialize)]
pub struct ParkedState {
    pub pos: Vec2,
    pub battery: f32,
}

#[derive(Clone, Serialize)]
pub struct KnownTarget {
    pub pos: Vec2,
    pub confirmed: bool,
}

#[derive(Clone, Serialize)]
pub struct JammerState {
    pub pos: Vec2,
    pub radius: f32,
}

#[derive(Clone, Serialize)]
pub struct TargetState {
    /// Stable for the target's lifetime, for `Command::RemoveTarget`.
    pub id: u32,
    pub pos: Vec2,
}

#[derive(Clone, Serialize)]
pub struct SwarmState {
    pub seq: u32,
    pub agents: Vec<AgentState>,
    /// Every target, by its world ID (not the swarm's own target IDs, which the drones assign).
    pub targets: Vec<TargetState>,
    /// Where liars claim the target is, so the viewer can show what the swarm is being told.
    pub fake_targets: Vec<Vec2>,
    /// m/s, in field coordinates.
    pub wind: Vec2,
    /// The launch base.
    pub base: Vec2,
    /// Half the field's width (x) and height (y); it's centered on the origin.
    pub field_half: Vec2,
    /// How far the drones' radio reaches, m.
    pub radio_range: f32,
    /// GPS jammers: drones inside get no fix.
    pub jammers: Vec<JammerState>,
    /// Drones waiting on the base to take off.
    pub launch_queue: u32,
    /// Seconds since some drone last flew over each cell of the field (capped at 60), as
    /// `GRID` x `GRID` cells, x-major: cell (x, y) is `x * GRID + y`.
    pub coverage_age: Vec<f32>,
    /// Airframes charging on the base's pads.
    pub parked: Vec<ParkedState>,
    /// How the next mission will be simulated (see `Simulation`).
    pub simulation: Simulation,
}

/// Fault injection requested from the viewer.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Command {
    /// Kill the drone with this MAVLink system ID.
    Kill(u8),
    /// Kill one randomly chosen live drone.
    KillRandom,
    /// Compromise the drone with this sysid: it starts broadcasting a fake target.
    Lie(u8),
    /// Set the wind over the field, m/s, e.g. `{"wind":{"x":3.0,"y":0.0}}`.
    Wind(Vec2),
    /// Start a new mission: fresh drones, positions, targets and weather (`"reset"`).
    Reset,
    /// Put a new target in the field, at this spot or a random one:
    /// `{"add_target":{"x":3.0,"y":-2.0}}` or `{"add_target":null}`.
    AddTarget(Option<Vec2>),
    /// Take the target with this world ID out of the field: `{"remove_target":4}`.
    RemoveTarget(u32),
    /// Launch a new drone over this spot or a random one: `{"add_drone":{"x":0,"y":0}}` or
    /// `{"add_drone":null}`. It gets the next unused system ID.
    AddDrone(Option<Vec2>),
    /// Take the drone with this sysid out of the mission (it leaves; unlike `Kill`, it doesn't
    /// crash): `{"remove_drone":5}`.
    RemoveDrone(u8),
    /// Jam GPS around this spot, or a random one: `{"jam":{"x":3.0,"y":-2.0}}` or `{"jam":null}`.
    Jam(Option<Vec2>),
    /// Switch off every jammer: `"clear_jamming"`.
    ClearJamming,
    /// Spoof this drone's GPS (dragging its fix slowly off), or stop if it already is:
    /// `{"spoof":5}`.
    Spoof(u8),
    /// Launch this many more drones from the base, one after another: `{"launch":4}`. The
    /// operator's ground station sends the same thing over the base link (base.rs).
    Launch(u32),
    /// Start a new mission on a field this wide (x) and high (y), in meters, launching this
    /// many drones (None: as many as the backend was started with):
    /// `{"set_field":[{"x":40.0,"y":30.0},0]}`. The base stays in its corner.
    SetField(Vec2, Option<u32>),
    /// Keep this many spare airframes on the base's pads: `{"set_spares":8}`. The ground
    /// station sends the same over the base link.
    SetSpares(u32),
    /// Whether a charged airframe takes off in the place of each drone down to charge:
    /// `{"set_rotation":true}`.
    SetRotation(bool),
    /// How to simulate the world from the next mission on (see `Simulation`):
    /// `{"simulation":{"gps":true,"proximity":true,"battery_min":20.0,"charge_min":60.0,"random_search":false}}`.
    Simulation(Simulation),
}

/// How the world is simulated: what the drones' sensors and batteries are like, and how they
/// search. Not what an operator would choose: things about the drones and the world to try the
/// swarm against. Set from the viewer for the next mission (the backend's flags set it to begin
/// with).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Simulation {
    /// GNSS fixes with realistic error (off: the true positions).
    pub gps: bool,
    /// The drones' onboard sensing of each other, within 5 m (off: they keep clear of each other
    /// by the positions they broadcast alone).
    pub proximity: bool,
    /// How many minutes a full battery flies a drone (None: batteries never run down), and how
    /// many a pad takes to charge one from empty.
    pub battery_min: Option<f32>,
    pub charge_min: f32,
    /// Free drones search by random walk instead of the shared coverage map, to compare.
    pub random_search: bool,
}

/// Returns the state broadcaster, plus both ends of the command channel: the sender so other
/// sources of commands (the base link) can feed the same world.
pub fn spawn_server(addr: &str, capacity: usize) -> (broadcast::Sender<SwarmState>, mpsc::Sender<Command>, mpsc::Receiver<Command>) {
    let (state_tx, _rx) = broadcast::channel(capacity);
    let (command_tx, command_rx) = mpsc::channel(16);
    let state_tx_for_server = state_tx.clone();
    let command_tx_for_others = command_tx.clone();
    let addr = addr.to_string();

    tokio::spawn(async move {
        let listener = match TcpListener::bind(&addr).await {
            Ok(l) => l,
            Err(e) => {
                eprintln!("ws: failed to bind {addr}: {e}");
                return;
            }
        };
        println!("ws: listening on ws://{addr}");

        loop {
            let (stream, peer) = match listener.accept().await {
                Ok(pair) => pair,
                Err(e) => {
                    eprintln!("ws: accept failed: {e}");
                    continue;
                }
            };
            let rx = state_tx_for_server.subscribe();
            tokio::spawn(handle_client(stream, peer.to_string(), rx, command_tx.clone()));
        }
    });

    (state_tx, command_tx_for_others, command_rx)
}

async fn handle_client(
    stream: tokio::net::TcpStream,
    peer: String,
    mut rx: broadcast::Receiver<SwarmState>,
    command_tx: mpsc::Sender<Command>,
) {
    let ws_stream = match tokio_tungstenite::accept_async(stream).await {
        Ok(s) => s,
        Err(e) => {
            eprintln!("ws: handshake with {peer} failed: {e}");
            return;
        }
    };
    println!("ws: viewer connected ({peer})");

    let (mut write, mut read) = ws_stream.split();
    // Commands are read on their own task: if this viewer is slow to take its state updates,
    // writing them blocks, and with one loop for both its commands would wait behind them.
    let reader_peer = peer.clone();
    let mut reader = tokio::spawn(async move {
        while let Some(msg) = read.next().await {
            match msg {
                Ok(tokio_tungstenite::tungstenite::Message::Text(text)) => match serde_json::from_str::<Command>(&text) {
                    Ok(command) => {
                        if command_tx.send(command).await.is_err() {
                            return;
                        }
                    }
                    Err(e) => eprintln!("ws: ignoring bad command from {reader_peer}: {e}"),
                },
                Ok(tokio_tungstenite::tungstenite::Message::Close(_)) | Err(_) => return,
                Ok(_) => {}
            }
        }
    });
    loop {
        tokio::select! {
            state = rx.recv() => match state {
                Ok(state) => {
                    let json = serde_json::to_string(&state).unwrap();
                    if write.send(tokio_tungstenite::tungstenite::Message::Text(json.into())).await.is_err() {
                        break;
                    }
                }
                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                Err(broadcast::error::RecvError::Closed) => break,
            },
            // The viewer hung up (or sent Close).
            _ = &mut reader => break,
        }
    }
    reader.abort();
    println!("ws: viewer disconnected ({peer})");
}
