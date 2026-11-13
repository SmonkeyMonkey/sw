//! The operator's ground control station (GCS). Unlike the viewer, which shows the simulator's
//! ground truth, this only knows what the swarm tells it over MAVLink: drone statuses relayed
//! hop by hop, and target sightings. From here the operator can:
//!  - launch drones from the base,
//!  - give one drone an order (go to a spot, hold, return to base) or hand it back to autonomy,
//!  - cancel a target, so the swarm stops responding to it.
//!
//! Commands are COMMAND_LONGs, repeated until acknowledged, the way MAVLink ground stations do.

use std::collections::{BTreeMap, VecDeque};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use bevy::prelude::*;
use bevy::window::WindowResolution;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use swarm_gcs::link::{self, Heard, Link, Send};

mod remote;
use mavlink_swarm::{MavCmd, MavMessage, MavResult, COMMAND_LONG_DATA};

const RADIO_ADDR: &str = "127.0.0.1:14600";
const BASE_ADDR: &str = "127.0.0.1:14650";
/// The launcher's MAVLink system ID (backend/src/base.rs).
const BASE_SYSID: u8 = 250;
/// The field the swarm starts on (width, height, in meters; the backend's default), and the
/// sizes the operator can pick (backend/src/world.rs MIN_FIELD, MAX_FIELD).
const DEFAULT_FIELD: (f32, f32) = (30.0, 30.0);
const FIELD_RANGE: std::ops::RangeInclusive<f32> = 10.0..=600.0;

/// A command is repeated this often until acknowledged...
const RETRY_EVERY: Duration = Duration::from_millis(1_200);
/// ...at most this many times.
const MAX_ATTEMPTS: u8 = 6;
/// A drone or target not heard from for this long is shown as stale: three of a drone's
/// statuses missed (it sends one every 2 s).
pub const STALE: Duration = Duration::from_secs(6);

fn main() {
    let (heard, to_send) = link::spawn(RADIO_ADDR.parse().unwrap(), BASE_ADDR.parse().unwrap(), swarm_gcs::qgc::QGC_ADDR.parse().unwrap());
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "swarm ground station".into(),
                resolution: WindowResolution::new(1100, 720),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(Links { heard: Mutex::new(heard), to_send: Mutex::new(to_send) })
        .init_resource::<Gcs>()
        .init_resource::<remote::Remote>()
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn(Camera2d);
        })
        .add_systems(Update, (listen, retry, watch_losses, remote::transmit))
        .add_systems(EguiPrimaryContextPass, (ui, remote::keys, remote::panel))
        .run();
}

/// `Receiver`/`Sender` aren't `Sync`; a Mutex makes them Bevy resources.
#[derive(Resource)]
pub struct Links {
    heard: Mutex<Receiver<Heard>>,
    to_send: Mutex<Sender<Send>>,
}

impl Links {
    pub fn send(&self, link: Link, msg: MavMessage) {
        let _ = self.to_send.lock().unwrap().send(Send { link, msg });
    }
}

/// A drone as the ground station knows it.
pub struct DroneView {
    pos: egui::Pos2,
    /// Where its position comes from: 0 GPS, 1 neighbors, 2 dead reckoning.
    nav: u8,
    mode: u8,
    target_id: u16,
    order: u8,
    hops: u8,
    /// When we heard about it, minus how old the news already was.
    seen_at: Instant,
}

/// A target as the swarm reports it.
struct TargetView {
    pos: egui::Pos2,
    cancelled: bool,
    responders: u8,
    seen_at: Instant,
}

/// A command waiting for its acknowledgement.
struct Pending {
    link: Link,
    cmd: COMMAND_LONG_DATA,
    label: String,
    sent_at: Instant,
    attempts: u8,
}

#[derive(Resource)]
pub struct Gcs {
    drones: BTreeMap<u8, DroneView>,
    targets: BTreeMap<u16, TargetView>,
    selected: Option<u8>,
    pending: Vec<Pending>,
    log: VecDeque<String>,
    launch_count: u32,
    started: Instant,
    /// The field the swarm is on (width, height, m), and the one being set up on the map.
    field: egui::Vec2,
    field_draft: egui::Vec2,
    /// Dragging the field's corner on the map.
    dragging_corner: bool,
    /// Drones waiting at the base, as the operator says: each launch takes from it, and a new
    /// mission brings the drones in the field back to it.
    at_base: u32,
    /// Launch a drone from the base for each one lost (the operator's switch), and which are
    /// lost.
    replace_lost: bool,
    losses: swarm_gcs::fleet::LossWatch,
    /// What the base last said is on its pads, and when.
    base: Option<(mavlink_swarm::SWARM_BASE_STATUS_DATA, Instant)>,
    /// How many spares the operator wants on the pads (sent with "Put on the pads").
    spares_draft: u32,
}

impl Default for Gcs {
    fn default() -> Self {
        Self {
            drones: BTreeMap::new(),
            targets: BTreeMap::new(),
            selected: None,
            pending: Vec::new(),
            log: VecDeque::new(),
            launch_count: 4,
            started: Instant::now(),
            field: egui::vec2(DEFAULT_FIELD.0, DEFAULT_FIELD.1),
            field_draft: egui::vec2(DEFAULT_FIELD.0, DEFAULT_FIELD.1),
            dragging_corner: false,
            at_base: 12,
            replace_lost: false,
            losses: Default::default(),
            base: None,
            spares_draft: 0,
        }
    }
}

impl Gcs {
    fn note(&mut self, line: String) {
        let t = self.started.elapsed().as_secs_f32();
        self.log.push_front(format!("{t:6.1}s  {line}"));
        self.log.truncate(14);
    }

    /// Sends a command now and keeps repeating it until it's acknowledged.
    fn command(&mut self, links: &Links, link: Link, target_system: u8, command: MavCmd, params: [f32; 7], label: String) {
        // A newer order for the same drone replaces one still waiting.
        self.pending.retain(|p| p.cmd.target_system != target_system || target_system == 0);
        let cmd = COMMAND_LONG_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
            command,
            target_system,
            target_component: 0,
            confirmation: 0,
        };
        links.send(link, MavMessage::COMMAND_LONG(cmd.clone()));
        self.note(format!("→ {label}"));
        self.pending.push(Pending { link, cmd, label, sent_at: Instant::now(), attempts: 1 });
    }

    /// Launches `n` drones from the base, out of those waiting there.
    fn launch(&mut self, links: &Links, n: u32) {
        let n = n.min(self.at_base);
        if n == 0 {
            return;
        }
        self.at_base -= n;
        self.command(links, Link::Base, BASE_SYSID, MavCmd::MAV_CMD_SWARM_LAUNCH, [n as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], format!("launch {n}"));
    }

    fn acknowledged(&mut self, from: u8, command: u16, result: u8) {
        let Some(k) = self.pending.iter().position(|p| p.cmd.target_system == from && p.cmd.command as u16 == command) else {
            return;
        };
        let p = self.pending.remove(k);
        let outcome = if result == MavResult::MAV_RESULT_ACCEPTED as u8 { "done".to_string() } else { format!("refused ({result})") };
        self.note(format!("✓ {}: {outcome} after {} tries", p.label, p.attempts));
    }
}

/// The base's pads: what's on them, how many spares it keeps there, and whether charged
/// airframes take over from drones down to charge, so the swarm keeps on by itself.
fn charging_panel(ui: &mut egui::Ui, links: &Links, gcs: &mut Gcs, now: Instant) {
    ui.separator();
    ui.heading("Charging");
    let Some(base) = gcs.base.clone().filter(|(_, at)| now.duration_since(*at) <= STALE).map(|(base, _)| base) else {
        ui.label(egui::RichText::new("no word from the base").small().weak());
        return;
    };
    ui.label(format!("On the pads: {} airframes, {} charged.", base.parked, base.ready));
    if base.batteries == 1 {
        if base.lowest_charge <= 100 {
            ui.label(format!("Lowest charge in the air: {}%.", base.lowest_charge));
        }
        if base.waiting > 0 {
            ui.colored_label(egui::Color32::from_rgb(240, 170, 60), format!("{} down to charge, waiting for a charged airframe to take over.", base.waiting));
        }
    } else {
        ui.label(egui::RichText::new("The drones' batteries don't run down here (start the backend with --battery-min).").small().weak());
    }
    ui.horizontal(|ui| {
        ui.label("Spares:");
        ui.add(egui::DragValue::new(&mut gcs.spares_draft).range(0..=64));
        let changed = gcs.spares_draft != base.spares as u32;
        if ui.add_enabled(changed, egui::Button::new("Put on the pads")).on_hover_text("Charged airframes waiting on the pads, to take over from drones that come back to charge.").clicked() {
            let n = gcs.spares_draft;
            gcs.command(links, Link::Base, BASE_SYSID, MavCmd::MAV_CMD_SWARM_SET_SPARES, [n as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], format!("{n} spares on the pads"));
        }
    });
    let mut rotation = base.rotation == 1;
    if ui
        .checkbox(&mut rotation, "Swap in charged drones")
        .on_hover_text("Each drone that comes back low on charge is replaced by a charged airframe from the pads, flying as it: as many stay in the air as were launched, for as long as there are charged ones. A drone you send home isn't replaced.")
        .changed()
    {
        gcs.command(links, Link::Base, BASE_SYSID, MavCmd::MAV_CMD_SWARM_SET_ROTATION, [rotation as u8 as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], format!("swap in charged drones: {}", if rotation { "on" } else { "off" }));
        // Shown as asked for until the base says otherwise.
        if let Some((st, _)) = gcs.base.as_mut() {
            st.rotation = rotation as u8;
        }
    }
}

/// Folds everything heard on the links into the ground station's picture of the swarm.
fn listen(links: Res<Links>, mut gcs: ResMut<Gcs>) {
    let heard: Vec<Heard> = links.heard.lock().unwrap().try_iter().collect();
    let now = Instant::now();
    for Heard { link, header, msg } in heard {
        match (link, msg) {
            (Link::Radio, MavMessage::SWARM_DRONE_STATUS(st)) => {
                let seen_at = now - Duration::from_millis(st.age_ms as u64);
                // Several relays can deliver the same status; keep the freshest.
                if gcs.drones.get(&st.sysid).is_some_and(|d| d.seen_at > seen_at) {
                    continue;
                }
                gcs.drones.insert(
                    st.sysid,
                    DroneView { pos: egui::pos2(st.x, st.y), nav: st.nav, mode: st.mode, target_id: st.target_id, order: st.order, hops: st.hops, seen_at },
                );
                if st.ack_command != 0 {
                    gcs.acknowledged(st.sysid, st.ack_command, st.ack_result);
                }
            }
            (Link::Radio, MavMessage::COMMAND_ACK(ack)) => {
                // Heard directly: the header names the drone.
                gcs.acknowledged(header.system_id, ack.command as u16, ack.result as u8);
            }
            (Link::Radio, MavMessage::SWARM_TARGET_SIGHTING(s)) => {
                let seen_at = now - Duration::from_millis(s.age_ms as u64);
                if gcs.targets.get(&s.target_id).is_some_and(|t| t.seen_at > seen_at && !(s.cancelled == 1 && !t.cancelled)) {
                    continue;
                }
                let was_cancelled = gcs.targets.get(&s.target_id).is_some_and(|t| t.cancelled);
                gcs.targets.insert(
                    s.target_id,
                    TargetView { pos: egui::pos2(s.x, s.y), cancelled: s.cancelled == 1 || was_cancelled, responders: s.responders, seen_at },
                );
                // A cancel is broadcast, so nobody acknowledges it; the swarm reporting the
                // target as cancelled is the confirmation.
                if s.cancelled == 1 {
                    let id = s.target_id as f32;
                    if let Some(k) = gcs.pending.iter().position(|p| p.cmd.command == MavCmd::MAV_CMD_SWARM_CANCEL_TARGET && p.cmd.param1 == id) {
                        let p = gcs.pending.remove(k);
                        gcs.note(format!("✓ {}: the swarm reports it cancelled", p.label));
                    }
                }
            }
            (Link::Base, MavMessage::COMMAND_ACK(ack)) => gcs.acknowledged(BASE_SYSID, ack.command as u16, ack.result as u8),
            (Link::Base, MavMessage::SWARM_BASE_STATUS(st)) => {
                // The first word from the base: start from what it has.
                if gcs.base.is_none() {
                    gcs.spares_draft = st.spares as u32;
                }
                gcs.base = Some((st, now));
            }
            _ => {}
        }
    }
    // Targets the swarm stopped reporting are forgotten after a while.
    gcs.targets.retain(|_, t| now.duration_since(t.seen_at) < Duration::from_secs(90));
}

/// Repeats unacknowledged commands, the standard MAVLink way: same command, `confirmation`
/// counting up.
fn retry(links: Res<Links>, mut gcs: ResMut<Gcs>) {
    let now = Instant::now();
    let mut gave_up = Vec::new();
    for p in gcs.pending.iter_mut() {
        if now.duration_since(p.sent_at) < RETRY_EVERY {
            continue;
        }
        if p.attempts >= MAX_ATTEMPTS {
            gave_up.push(p.label.clone());
            continue;
        }
        p.attempts += 1;
        p.cmd.confirmation = p.cmd.confirmation.wrapping_add(1);
        p.sent_at = now;
        links.send(p.link, MavMessage::COMMAND_LONG(p.cmd.clone()));
    }
    if !gave_up.is_empty() {
        gcs.pending.retain(|p| p.attempts < MAX_ATTEMPTS || now.duration_since(p.sent_at) < RETRY_EVERY);
        for label in gave_up {
            gcs.note(format!("✗ {label}: no acknowledgement"));
        }
    }
}

/// Notices drones lost from the mission (see `swarm_gcs::fleet`), and with the operator's switch
/// on, launches a replacement for each from the base.
fn watch_losses(links: Res<Links>, mut gcs: ResMut<Gcs>) {
    let now = Instant::now();
    let drones: Vec<(u8, Duration, u8)> = gcs.drones.iter().map(|(&id, d)| (id, now.duration_since(d.seen_at), d.order)).collect();
    for event in gcs.losses.update(drones) {
        match event {
            swarm_gcs::fleet::FleetEvent::Lost(id) if gcs.replace_lost && gcs.at_base > 0 => {
                gcs.note(format!("drone {id} lost: launching a replacement"));
                gcs.launch(&links, 1);
            }
            swarm_gcs::fleet::FleetEvent::Lost(id) if gcs.replace_lost => gcs.note(format!("drone {id} lost: none left at the base to replace it")),
            swarm_gcs::fleet::FleetEvent::Lost(id) => gcs.note(format!("drone {id} lost")),
            swarm_gcs::fleet::FleetEvent::Back(id) => gcs.note(format!("drone {id} is back")),
        }
    }
}

fn mode_name(mode: u8) -> &'static str {
    match mode {
        0 => "searching",
        1 => "investigating",
        2 => "converging",
        3 => "operator",
        _ => "?",
    }
}

fn order_name(order: u8) -> &'static str {
    match order {
        1 => "go to",
        2 => "hold",
        3 => "return to base",
        4 => "manual",
        5 => "manual, sticks lost",
        _ => "auto",
    }
}

fn mode_color(mode: u8) -> egui::Color32 {
    match mode {
        1 => egui::Color32::from_rgb(50, 200, 200),
        2 => egui::Color32::from_rgb(230, 150, 25),
        3 => egui::Color32::from_rgb(75, 215, 90),
        _ => egui::Color32::from_rgb(75, 125, 230),
    }
}

/// The drones' configuration on a field this size.
fn field_config(field: egui::Vec2, responders: u8, mode: swarm_brain::SwarmMode) -> swarm_brain::BrainConfig {
    let half = swarm_brain::Vec2::new(field.x / 2.0, field.y / 2.0);
    swarm_brain::BrainConfig {
        field_half: half,
        home: swarm_brain::Vec2::new(2.0 - half.x, 2.0 - half.y),
        responders,
        ..swarm_brain::BrainConfig::SWARM
    }
    .with_mode(mode)
}

/// How many drones the field needs (see `BrainConfig::plan`), for `responders` per target.
fn fleet_plan(field: egui::Vec2, responders: u8, mode: swarm_brain::SwarmMode, targets: usize) -> swarm_brain::FleetPlan {
    field_config(field, responders, mode).plan(targets)
}

/// The sweep's fleet sizes worth flying on a field (see `BrainConfig::sweep_options`), as
/// "drones (seconds a pass)".
fn sweep_options(field: egui::Vec2, mode: swarm_brain::SwarmMode) -> String {
    let options: Vec<String> = field_config(field, 4, mode)
        .sweep_options(128)
        .filter(|l| l.drones >= 3)
        .map(|l| format!("{} ({:.0} s)", l.drones, l.pass_s))
        .collect();
    options.join(", ")
}

fn ui(mut contexts: EguiContexts, links: Res<Links>, mut gcs: ResMut<Gcs>, mut remote: ResMut<remote::Remote>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let now = Instant::now();
    // egui 0.36: panels go inside a root Ui covering the window.
    let mut root = egui::Ui::new(
        ctx.clone(),
        "root".into(),
        egui::UiBuilder::new().layer_id(egui::LayerId::background()).max_rect(ctx.viewport_rect()),
    );

    egui::Panel::right("controls").min_size(330.0).show(&mut root, |ui| {
        ui.heading("Field");
        // Set up before launching: the drones are told the field as they take off, so a new one
        // starts a new mission. Drag the field's top right corner on the map, or:
        ui.add(egui::Slider::new(&mut gcs.field_draft.x, FIELD_RANGE).step_by(1.0).suffix(" m").text("wide"));
        ui.add(egui::Slider::new(&mut gcs.field_draft.y, FIELD_RANGE).step_by(1.0).suffix(" m").text("high"));
        ui.horizontal(|ui| {
            let changed = gcs.field_draft != gcs.field;
            if ui.add_enabled(changed, egui::Button::new("Apply (new mission)")).clicked() {
                let f = gcs.field_draft;
                gcs.field = f;
                // The drones flying land back at the base for the new mission.
                let back = gcs.drones.values().filter(|d| now.duration_since(d.seen_at) <= STALE).count() as u32;
                gcs.at_base += back;
                // The swarm's picture of the old field is gone with it.
                gcs.drones.clear();
                gcs.targets.clear();
                gcs.losses.clear();
                // Nothing launched on it yet: that's the Fleet panel's.
                gcs.command(&links, Link::Base, BASE_SYSID, MavCmd::MAV_CMD_SWARM_SET_FIELD, [f.x, f.y, 0.0, 0.0, 0.0, 0.0, 0.0], format!("field {:.0} x {:.0} m", f.x, f.y));
            }
            if ui.add_enabled(changed, egui::Button::new("Cancel")).clicked() {
                gcs.field_draft = gcs.field;
            }
        });
        ui.label(egui::RichText::new("The base is in the lower left corner. Drag the field's top right corner on the map.").small().weak());

        ui.separator();
        ui.heading("Fleet");
        // For the sweep and the first mouse: the drones adapt the rest as mice are found.
        ui.horizontal(|ui| {
            ui.label("Mode:");
            let mut mode = remote.mode;
            ui.radio_value(&mut mode, Some(swarm_brain::SwarmMode::Autonomous), "autonomous")
                .on_hover_text("An autonomous swarm: off the leash, the free drones sweep the field side by side in lanes, the quickest search, and rings follow their mice wherever they run. In contact with the base only near it.");
            ui.radio_value(&mut mode, Some(swarm_brain::SwarmMode::Connected), "connected")
                .on_hover_text("Always in contact with the base by relay chains: the free drones sweep the field in rays out from it, slower, and rings let a mouse go rather than leave the chain.");
            if mode != remote.mode {
                remote.mode = mode;
                if let Some(mode) = mode {
                    remote::send_mode(&links, mode, 0);
                    gcs.note(format!("→ mode: {mode:?}"));
                }
            }
        });
        // Planned for the mode picked, or the default one (autonomous) if none is yet.
        let plan = fleet_plan(gcs.field_draft, remote.responders, remote.mode.unwrap_or(swarm_brain::SwarmMode::Autonomous), 1);
        let in_field = gcs.drones.values().filter(|d| now.duration_since(d.seen_at) <= STALE).count() as u32;
        ui.horizontal(|ui| {
            ui.label("At the base:");
            ui.add(egui::DragValue::new(&mut gcs.at_base).range(0..=200));
            ui.label(format!("in the field: {in_field}"));
        });
        ui.label(if plan.relays > 0 {
            format!("This field needs {}: {} to sweep it, and {} for a mouse with ~{} relays out to it.", plan.total, plan.sweep, plan.rings, plan.relays)
        } else {
            format!("This field needs {}: {} to sweep it, and {} for a mouse.", plan.total, plan.sweep, plan.rings)
        });
        ui.label(
            egui::RichText::new(format!("Each mouse after that takes up to {} more (fewer with the remote's \"adapt\").", plan.rings + plan.relays))
                .small()
                .weak(),
        );
        ui.label(egui::RichText::new(format!("The sweep crosses the field in about {:.0} s.", plan.pass_s)).small().weak());
        if let Some(l) = plan.layout {
            ui.label(
                egui::RichText::new(format!(
                    "{} front(s) of {} drones abreast, {} lane(s) each, one radio network; {:.0}% of their flying sweeps new ground.",
                    l.rows,
                    l.per_row,
                    l.lanes,
                    l.efficiency * 100.0
                ))
                .small()
                .weak(),
            );
            let mode = remote.mode.unwrap_or(swarm_brain::SwarmMode::Autonomous);
            ui.label(egui::RichText::new(format!("Sweep fleets worth flying here: {}.", sweep_options(gcs.field_draft, mode))).small().weak())
                .on_hover_text("Each looks over the field sooner than any smaller fleet. Drones beyond one of these and short of the next only crowd its fronts.");
        }
        let have = gcs.at_base + in_field;
        if have < plan.total as u32 {
            ui.colored_label(
                egui::Color32::from_rgb(240, 170, 60),
                format!("{} short: while drones are on mice, the sweep won't reach the far side.", plan.total as u32 - have),
            );
        }
        ui.checkbox(&mut gcs.replace_lost, "Replace lost drones").on_hover_text(format!(
            "A drone silent for {} s (crashed, or out of contact for good) is replaced by one from the base, while there are any. A drone sent home isn't lost.",
            swarm_gcs::fleet::LOST_AFTER.as_secs()
        ));
        let short = (plan.total as u32).saturating_sub(in_field);
        let wanted = short.min(gcs.at_base);
        ui.horizontal(|ui| {
            if ui.add_enabled(wanted > 0, egui::Button::new(format!("Launch what's needed ({wanted})"))).clicked() {
                gcs.launch(&links, wanted);
            }
            if short == 0 {
                ui.label(egui::RichText::new("enough in the field").small().weak());
            } else if gcs.at_base == 0 {
                ui.colored_label(egui::Color32::from_rgb(240, 170, 60), "none at the base: set how many there are");
            }
        });
        ui.horizontal(|ui| {
            let most = gcs.at_base.max(1);
            gcs.launch_count = gcs.launch_count.clamp(1, most);
            ui.add_enabled(gcs.at_base > 0, egui::Slider::new(&mut gcs.launch_count, 1..=most).text("drones"));
            if ui.add_enabled(gcs.at_base > 0, egui::Button::new("Launch")).clicked() {
                let n = gcs.launch_count;
                gcs.launch(&links, n);
            }
        });
        charging_panel(ui, &links, &mut gcs, now);

        ui.separator();
        ui.heading("Drones");
        egui::ScrollArea::vertical().id_salt("drones").max_height(220.0).show(ui, |ui| {
            let rows: Vec<(u8, String, bool)> = gcs
                .drones
                .iter()
                .map(|(&id, d)| {
                    let age = now.duration_since(d.seen_at).as_secs_f32();
                    let target = if d.target_id != 0 { format!(" on {:04x}", d.target_id) } else { String::new() };
                    // Only worth a mention when it isn't plain GPS.
                    let nav = match d.nav { 1 => " nav:nbrs", 2 => " nav:DR", _ => "" };
                    (id, format!("{id:>3}  {}{target}{nav}  [{}]  {} hops, {age:.0}s ago", mode_name(d.mode), order_name(d.order), d.hops), age > STALE.as_secs_f32())
                })
                .collect();
            for (id, text, stale) in rows {
                let text = if stale { egui::RichText::new(text).weak() } else { egui::RichText::new(text) };
                if ui.selectable_label(gcs.selected == Some(id), text).clicked() {
                    gcs.selected = Some(id);
                }
            }
        });

        if let Some(id) = gcs.selected {
            ui.separator();
            ui.heading(format!("Drone {id}"));
            ui.label("Click the map to send it there.");
            ui.horizontal(|ui| {
                if ui.button("Hold").clicked() {
                    gcs.command(&links, Link::Radio, id, MavCmd::MAV_CMD_NAV_LOITER_UNLIM, [0.0; 7], format!("drone {id}: hold"));
                }
                if ui.button("Return to base").clicked() {
                    gcs.command(&links, Link::Radio, id, MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7], format!("drone {id}: return to base"));
                }
                if ui.button("Resume autonomy").clicked() {
                    gcs.command(&links, Link::Radio, id, MavCmd::MAV_CMD_MISSION_START, [0.0; 7], format!("drone {id}: resume"));
                }
            });
            // Captured, or acting like it: the base revokes its key, so the swarm stops
            // listening to it, and tells it to come home.
            if ui.button("Revoke (captured)").on_hover_text("The base revokes its key for the rest of the mission").clicked() {
                gcs.command(&links, Link::Base, BASE_SYSID, MavCmd::MAV_CMD_SWARM_REVOKE, [id as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], format!("drone {id}: revoke"));
            }
        }

        ui.separator();
        ui.heading("Targets");
        let rows: Vec<(u16, String, bool)> = gcs
            .targets
            .iter()
            .map(|(&id, t)| {
                let age = now.duration_since(t.seen_at).as_secs_f32();
                let state = if t.cancelled { "cancelled".to_string() } else { format!("{} responding", t.responders) };
                (id, format!("{id:04x}  ({:+.1}, {:+.1})  {state}, {age:.0}s ago", t.pos.x, t.pos.y), t.cancelled)
            })
            .collect();
        for (id, text, cancelled) in rows {
            ui.horizontal(|ui| {
                ui.label(text);
                if !cancelled && ui.button("Cancel").clicked() {
                    gcs.command(&links, Link::Radio, 0, MavCmd::MAV_CMD_SWARM_CANCEL_TARGET, [id as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], format!("cancel target {id:04x}"));
                }
            });
        }

        ui.separator();
        ui.heading("Log");
        for line in &gcs.log {
            ui.monospace(line);
        }
    });

    egui::CentralPanel::default().show(&mut root, |ui| {
        let size = ui.available_size().min_elem();
        let (response, painter) = ui.allocate_painter(egui::vec2(size, size), egui::Sense::click_and_drag());
        let rect = response.rect;
        // Room for the bigger of the two fields (on, and being set up), and a margin to drag in.
        let shown = gcs.field.max(gcs.field_draft).max_elem() * 1.15;
        let scale = size / shown;
        // The field is centered on the origin; field y points up, screen y points down.
        let to_screen = |p: egui::Pos2| egui::pos2(rect.center().x + p.x * scale, rect.center().y - p.y * scale);
        let to_field = |s: egui::Pos2| egui::pos2((s.x - rect.center().x) / scale, (rect.center().y - s.y) / scale);

        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 18, 17));
        let field_rect = |f: egui::Vec2| egui::Rect::from_two_pos(to_screen(egui::pos2(-f.x / 2.0, -f.y / 2.0)), to_screen(egui::pos2(f.x / 2.0, f.y / 2.0)));
        painter.rect_filled(field_rect(gcs.field), 0.0, egui::Color32::from_rgb(28, 32, 30));
        if gcs.field_draft != gcs.field {
            // The field being set up, not applied yet.
            painter.rect_stroke(field_rect(gcs.field_draft), 0.0, egui::Stroke::new(2.0, egui::Color32::from_rgb(240, 200, 60)), egui::StrokeKind::Middle);
        }
        // Its top right corner: drag it to resize.
        let corner = to_screen(egui::pos2(gcs.field_draft.x / 2.0, gcs.field_draft.y / 2.0));
        painter.circle_filled(corner, 6.0, egui::Color32::from_rgb(240, 200, 60));
        if response.drag_started() && response.interact_pointer_pos().is_some_and(|p| p.distance(corner) < 14.0) {
            gcs.dragging_corner = true;
        }
        if gcs.dragging_corner {
            if let Some(p) = response.interact_pointer_pos() {
                let at = to_field(p);
                let clamp = |v: f32| (2.0 * v).round().clamp(*FIELD_RANGE.start(), *FIELD_RANGE.end());
                gcs.field_draft = egui::vec2(clamp(at.x), clamp(at.y));
            }
            if response.drag_stopped() {
                gcs.dragging_corner = false;
            }
        }
        // The base, in its corner (2 m in), of the field being set up.
        let base = to_screen(egui::pos2(2.0 - gcs.field_draft.x / 2.0, 2.0 - gcs.field_draft.y / 2.0));
        painter.rect_stroke(egui::Rect::from_center_size(base, egui::vec2(28.0, 28.0)), 0.0, egui::Stroke::new(1.5, egui::Color32::LIGHT_GRAY), egui::StrokeKind::Middle);

        for (&id, t) in &gcs.targets {
            let p = to_screen(t.pos);
            let color = if t.cancelled { egui::Color32::GRAY } else { egui::Color32::from_rgb(220, 50, 50) };
            painter.circle_stroke(p, 7.0, egui::Stroke::new(2.0, color));
            painter.text(p + egui::vec2(9.0, -9.0), egui::Align2::LEFT_BOTTOM, format!("{id:04x}"), egui::FontId::monospace(11.0), color);
        }
        for (&id, d) in &gcs.drones {
            let p = to_screen(d.pos);
            let stale = now.duration_since(d.seen_at) > STALE;
            let color = if stale { egui::Color32::DARK_GRAY } else { mode_color(d.mode) };
            painter.circle_filled(p, 5.0, color);
            if gcs.selected == Some(id) {
                painter.circle_stroke(p, 9.0, egui::Stroke::new(2.0, egui::Color32::WHITE));
            }
            painter.text(p + egui::vec2(7.0, 7.0), egui::Align2::LEFT_TOP, id.to_string(), egui::FontId::monospace(10.0), egui::Color32::LIGHT_GRAY);
        }

        // Click: send the selected drone there.
        if let (Some(id), Some(click)) = (gcs.selected, response.interact_pointer_pos().filter(|_| response.clicked())) {
            let at = to_field(click);
            // DO_REPOSITION takes latitude/longitude in params 5/6; this field has no GPS, so
            // they carry local x/y in meters instead.
            gcs.command(&links, Link::Radio, id, MavCmd::MAV_CMD_DO_REPOSITION, [-1.0, 0.0, 0.0, f32::NAN, at.x, at.y, 5.0], format!("drone {id}: go to ({:+.1}, {:+.1})", at.x, at.y));
        }
    });
    Ok(())
}
