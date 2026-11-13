//! Launches drones: one task per drone with a fresh brain, or the HIL bridge for the drone the
//! board flies. The world calls it at mission start and whenever the viewer adds a drone.

use std::net::SocketAddr;
use std::sync::Arc;

use rand::RngExt;
use swarm_brain::auth::Authority;
use swarm_brain::{Brain, BrainConfig, SensorReading};
use tokio::sync::mpsc;

use crate::agent::{self, AgentLinks};
use crate::body::{AgentReport, Fault};
use crate::hil;
use crate::observability::Telemetry;

/// The world's ends of one drone's links. Dropping `sensor_tx` ends the drone's task.
pub struct LaunchedDrone {
    pub sensor_tx: mpsc::Sender<SensorReading>,
    pub fault_tx: mpsc::Sender<Fault>,
}

pub struct Fleet {
    telemetry: Arc<Telemetry>,
    netem_addr: SocketAddr,
    brain_cfg: BrainConfig,
    /// Where the board's brain is (a mavlink-core address), and which sysid it flies as.
    hil: Option<(String, u8)>,
    report_tx: mpsc::Sender<AgentReport>,
    /// The launch base's certificate authority, and this mission's number: every drone gets
    /// fresh keys for the mission (see `swarm_brain::auth`).
    authority: Arc<Authority>,
    mission: u32,
    /// Each drone's keys, once issued: an airframe flying as one that went to charge is that
    /// drone to the swarm, keys and all (the swarm learns a drone's certificate once).
    issued: std::sync::Mutex<std::collections::HashMap<u8, swarm_brain::auth::Keys>>,
}

impl Fleet {
    /// Also returns the receiver every launched drone reports to.
    pub fn new(
        telemetry: Arc<Telemetry>,
        netem_addr: SocketAddr,
        brain_cfg: BrainConfig,
        hil: Option<(String, u8)>,
        authority: Arc<Authority>,
        mission: u32,
    ) -> (Self, mpsc::Receiver<AgentReport>) {
        // Plenty for a few dozen drones' reports per tick.
        let (report_tx, report_rx) = mpsc::channel(256);
        (Self { telemetry, netem_addr, brain_cfg, hil, report_tx, authority, mission, issued: Default::default() }, report_rx)
    }

    /// Starts the drone in world slot `index`, which flies as MAVLink system ID `index + 1`
    /// (0 means "broadcast").
    pub fn launch(&self, index: usize) -> LaunchedDrone {
        let (sensor_tx, sensor_rx) = mpsc::channel(4);
        let (fault_tx, fault_rx) = mpsc::channel(4);
        let sysid = index as u8 + 1;
        let links = AgentLinks {
            index,
            sensor_rx,
            report_tx: self.report_tx.clone(),
            netem_addr: self.netem_addr,
            fault_rx,
            telemetry: self.telemetry.clone(),
        };
        let beacon_period_ms = self.brain_cfg.beacon_period_ms;
        let keys = *self.issued.lock().unwrap().entry(sysid).or_insert_with(|| self.authority.issue(self.mission, sysid, rand::rng().random()));
        match &self.hil {
            Some((address, hil_sysid)) if *hil_sysid == sysid => {
                let address = address.clone();
                let mission = swarm_brain::hil::Mission::new(keys, &self.brain_cfg);
                tokio::spawn(async move {
                    if let Err(e) = hil::run(mission, links, &address, beacon_period_ms).await {
                        eprintln!("drone {sysid} (board): stopped: {e}");
                    }
                });
            }
            _ => {
                let brain = Brain::new(sysid, self.brain_cfg, rand::rng().random(), keys);
                tokio::spawn(async move {
                    if let Err(e) = agent::run(brain, links).await {
                        eprintln!("drone {sysid}: stopped: {e}");
                    }
                });
            }
        }
        LaunchedDrone { sensor_tx, fault_tx }
    }
}
