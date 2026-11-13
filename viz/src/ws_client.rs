//! Bridges the async WebSocket world to Bevy's synchronous per-frame `World`, same pattern as
//! the drone HIL project's viz/src/ws_client.rs. Two plain channels cross the boundary:
//!  - inbound (`Receiver<SwarmState>`): live swarm state, drained by a Bevy system each frame.
//!  - outbound (`Sender<String>`): JSON fault-injection commands from the Faults panel.

use std::sync::mpsc::{Receiver, Sender};

use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use tokio_tungstenite::tungstenite::Message;

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

#[derive(Clone, Debug, Deserialize)]
pub struct AgentState {
    pub id: u32,
    pub pos: Vec2,
    pub informed: bool,
    pub alive: bool,
    /// "searching" | "investigating" | "converging".
    #[serde(default)]
    pub mode: String,
    /// Compromised: its radio broadcasts a fake target.
    #[serde(default)]
    pub liar: bool,
    /// Swarm-wide ID of the target it's responding to, if any.
    #[serde(default)]
    pub target_id: Option<u16>,
    /// Where its brain thinks it is, where that comes from, and whether its GPS is spoofed.
    #[serde(default = "origin")]
    pub believed: Vec2,
    #[serde(default)]
    pub nav: String,
    #[serde(default)]
    pub spoofed: bool,
    /// Meters above the ground.
    #[serde(default)]
    pub alt: f32,
    /// "relay" (in a chain out to a ring), "sweep" (on the sweep), or "".
    #[serde(default)]
    pub role: String,
    /// Its battery's charge, 0 to 1 (None: the swarm runs without batteries).
    #[serde(default)]
    pub battery: Option<f32>,
}

/// How the backend simulates the world from the next mission on (backend/src/ws.rs).
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, serde::Serialize)]
pub struct Simulation {
    pub gps: bool,
    pub proximity: bool,
    pub battery_min: Option<f32>,
    pub charge_min: f32,
    pub random_search: bool,
}

/// An airframe on one of the base's pads, charging.
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct ParkedState {
    pub pos: Vec2,
    pub battery: f32,
}

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct TargetState {
    pub id: u32,
    pub pos: Vec2,
}

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct JammerState {
    pub pos: Vec2,
    pub radius: f32,
}

#[derive(Clone, Debug, Deserialize)]
pub struct SwarmState {
    pub seq: u32,
    pub agents: Vec<AgentState>,
    /// Every target, by its world ID.
    #[serde(default)]
    pub targets: Vec<TargetState>,
    /// Where liars claim the target is.
    #[serde(default)]
    pub fake_targets: Vec<Vec2>,
    /// The launch base.
    #[serde(default = "origin")]
    pub base: Vec2,
    /// Half the field's width (x) and height (y); it's centered on the origin.
    #[serde(default = "default_field_half")]
    pub field_half: Vec2,
    /// How far the drones' radio reaches, m (0: not sent, no links drawn).
    #[serde(default)]
    pub radio_range: f32,
    /// GPS jammers.
    #[serde(default)]
    pub jammers: Vec<JammerState>,
    /// Drones waiting on the base to take off.
    #[serde(default)]
    pub launch_queue: u32,
    /// Seconds since some drone flew over each cell (x-major, GRID x GRID), capped at 60.
    #[serde(default)]
    pub coverage_age: Vec<f32>,
    /// Airframes charging on the base's pads.
    #[serde(default)]
    pub parked: Vec<ParkedState>,
    /// How the next mission will be simulated (None: an older backend).
    #[serde(default)]
    pub simulation: Option<Simulation>,
}

fn default_field_half() -> Vec2 {
    Vec2 { x: 15.0, y: 15.0 }
}

fn origin() -> Vec2 {
    Vec2 { x: 0.0, y: 0.0 }
}

pub fn spawn_client(url: &str) -> (Receiver<SwarmState>, Sender<String>) {
    let (state_tx, state_rx) = std::sync::mpsc::channel();
    let (command_tx, command_rx) = std::sync::mpsc::channel();
    let url = url.to_string();

    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to build tokio runtime for ws client thread");
        rt.block_on(connect_loop(url, state_tx, command_rx));
    });

    (state_rx, command_tx)
}

/// Keeps retrying the connection forever: the backend may start after the viewer, or be
/// restarted mid-session, so a dropped connection is a normal, recoverable event here.
async fn connect_loop(url: String, state_tx: Sender<SwarmState>, command_rx: Receiver<String>) {
    loop {
        match tokio_tungstenite::connect_async(&url).await {
            Ok((ws_stream, _)) => {
                println!("ws_client: connected to {url}");
                let (mut write, mut read) = ws_stream.split();
                loop {
                    // Commands are rare (button clicks), so draining them non-blockingly before
                    // each read is responsive enough without a second task.
                    while let Ok(command) = command_rx.try_recv() {
                        if write.send(Message::Text(command.into())).await.is_err() {
                            break;
                        }
                    }
                    let next = tokio::time::timeout(std::time::Duration::from_millis(50), read.next()).await;
                    match next {
                        Ok(Some(Ok(Message::Text(text)))) => {
                            if let Ok(state) = serde_json::from_str::<SwarmState>(&text) {
                                if state_tx.send(state).is_err() {
                                    return;
                                }
                            }
                        }
                        Ok(Some(Ok(Message::Close(_)))) | Ok(Some(Err(_))) | Ok(None) => break,
                        Ok(Some(Ok(_))) | Err(_) => {}
                    }
                }
                println!("ws_client: disconnected, retrying...");
            }
            Err(e) => {
                eprintln!("ws_client: connect failed ({e}), retrying...");
            }
        }
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}
