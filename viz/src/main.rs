mod ws_client;

use std::sync::{
    mpsc::{Receiver, Sender},
    Mutex,
};

use bevy::prelude::*;
use bevy::window::WindowResolution;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use ws_client::{AgentState, SwarmState};

/// Wraps the mpsc::Receiver as a Bevy resource so systems can poll it each frame. `Receiver`
/// is `Send` but not `Sync`, and Bevy resources must be both — `Mutex` is the cheapest way to
/// add `Sync` back since only one system ever touches this at a time anyway.
#[derive(Resource)]
struct StateReceiver(Mutex<Receiver<SwarmState>>);

/// Outbound fault-injection commands (JSON), sent to the backend by the ws client thread.
#[derive(Resource)]
struct CommandSender(Mutex<Sender<String>>);

#[derive(Resource, Default)]
struct LatestState(Option<SwarmState>);

/// Marker on each agent's sprite entity, keyed by the backend's agent id so updates can find
/// the right entity without re-spawning everything every frame.
#[derive(Component)]
struct AgentMarker(u32);

/// One per coverage-map cell, by its index in `SwarmState::coverage_age`.
#[derive(Component)]
struct CoverageCell(usize);

/// One per target, by its world ID.
#[derive(Component)]
struct TargetMarker(u32);

#[derive(Component)]
struct StatusText;

/// World units (meters) to world pixels. The camera zooms so the whole field fits the window
/// (see `fit_camera`), whatever its size: this is just the scale at a 30 m field.
const WORLD_TO_PIXELS: f32 = 22.0;

const CONVERGING_COLOR: Color = Color::srgb(0.9, 0.6, 0.1); // amber: target confirmed, holding the ring
const INVESTIGATING_COLOR: Color = Color::srgb(0.2, 0.8, 0.8); // cyan: checking an unconfirmed report
const OPERATOR_COLOR: Color = Color::srgb(0.3, 0.85, 0.35); // green: following an operator order
const SEARCHING_COLOR: Color = Color::srgb(0.3, 0.5, 0.9); // blue: no usable news, wandering
const SWEEP_COLOR: Color = Color::srgb(0.55, 0.8, 1.0); // light blue: on the sweep, searching
const RELAY_COLOR: Color = Color::srgb(0.92, 0.92, 0.98); // white: relaying in a chain out to a ring
const DEAD_COLOR: Color = Color::srgb(0.35, 0.35, 0.35); // grey: killed, lying where it fell
const TARGET_COLOR: Color = Color::srgb(0.85, 0.15, 0.15);
const LIAR_COLOR: Color = Color::srgb(0.9, 0.2, 0.9); // magenta: compromised drone and its fake target

fn agent_color(agent: &AgentState) -> Color {
    if !agent.alive {
        return DEAD_COLOR;
    }
    match agent.mode.as_str() {
        "converging" => CONVERGING_COLOR,
        "investigating" => INVESTIGATING_COLOR,
        "operator" => OPERATOR_COLOR,
        _ => match agent.role.as_str() {
            "relay" => RELAY_COLOR,
            "sweep" => SWEEP_COLOR,
            _ => SEARCHING_COLOR,
        },
    }
}

fn main() {
    let (state_rx, command_tx) = ws_client::spawn_client("ws://127.0.0.1:9002");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "swarm viz".into(),
                resolution: WindowResolution::new(700, 700),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(StateReceiver(Mutex::new(state_rx)))
        .insert_resource(CommandSender(Mutex::new(command_tx)))
        .init_resource::<LatestState>()
        .init_resource::<SimulationDraft>()
        .add_systems(Startup, setup)
        .add_systems(Update, (receive_state, fit_camera, sync_coverage, sync_agents, update_targets, draw_overlays, update_status_text))
        .add_systems(EguiPrimaryContextPass, (faults_panel, simulation_panel, place_on_field))
        .run();
}

/// Fault injection. A killed drone goes silent on the radio and the rest of the swarm has to
/// notice on its own; a liar keeps flying but broadcasts a fake target, which the swarm has to
/// see through by consensus.
fn faults_panel(
    mut contexts: EguiContexts,
    latest: Res<LatestState>,
    sender: Res<CommandSender>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    let send = |command: String| {
        let _ = sender.0.lock().unwrap().send(command);
    };
    egui::Window::new("Faults").default_pos([520.0, 10.0]).show(ctx, |ui| {
        // A whole new mission without restarting anything: new drones, positions and targets.
        if ui.button("Reset mission").clicked() {
            send("\"reset\"".to_string());
        }
        ui.separator();
        if ui.button("Kill random drone").clicked() {
            send("\"kill_random\"".to_string());
        }
        ui.separator();
        ui.label("Kill a specific drone:");
        let Some(state) = &latest.0 else { return };
        ui.horizontal_wrapped(|ui| {
            for agent in state.agents.iter().filter(|a| a.alive) {
                if ui.button(agent.id.to_string()).clicked() {
                    send(format!("{{\"kill\":{}}}", agent.id));
                }
            }
        });
        ui.separator();
        ui.label("Drones (Shift+right-click the field to drop one in there):");
        ui.horizontal_wrapped(|ui| {
            if ui.button("Launch one from base").clicked() {
                send("{\"add_drone\":null}".to_string());
            }
            for agent in &state.agents {
                if ui.button(format!("Remove {}", agent.id)).clicked() {
                    send(format!("{{\"remove_drone\":{}}}", agent.id));
                }
            }
        });
        ui.separator();
        ui.label("GPS (Alt+right-click the field to put a jammer there):");
        ui.horizontal_wrapped(|ui| {
            if ui.button("Jam somewhere").clicked() {
                send("{\"jam\":null}".to_string());
            }
            if ui.button("Clear jamming").clicked() {
                send("\"clear_jamming\"".to_string());
            }
        });
        ui.label("Spoof (or stop spoofing) a drone's GPS:");
        ui.horizontal_wrapped(|ui| {
            for agent in state.agents.iter().filter(|a| a.alive) {
                let label = if agent.spoofed { format!("{}*", agent.id) } else { agent.id.to_string() };
                if ui.button(label).clicked() {
                    send(format!("{{\"spoof\":{}}}", agent.id));
                }
            }
        });
        ui.separator();
        ui.label("Mice (right-click the field to place one):");
        ui.horizontal_wrapped(|ui| {
            if ui.button("Add mouse").clicked() {
                send("{\"add_target\":null}".to_string());
            }
            for t in &state.targets {
                if ui.button(format!("Remove {}", t.id)).clicked() {
                    send(format!("{{\"remove_target\":{}}}", t.id));
                }
            }
        });
        ui.separator();
        ui.label("Make a drone lie (fake target):");
        ui.horizontal_wrapped(|ui| {
            for agent in state.agents.iter().filter(|a| a.alive && !a.liar) {
                if ui.button(agent.id.to_string()).clicked() {
                    send(format!("{{\"lie\":{}}}", agent.id));
                }
            }
        });
    });
    Ok(())
}

/// The Simulation panel's settings as being edited, until they're applied.
#[derive(Resource, Default)]
struct SimulationDraft(Option<ws_client::Simulation>);

/// How the world is simulated from the next mission on: the drones' sensors and batteries, and
/// a random-walk search to compare against. Things about the drones and the world to try the
/// swarm against, not an operator's choices (those are the ground station's).
fn simulation_panel(mut contexts: EguiContexts, latest: Res<LatestState>, sender: Res<CommandSender>, mut draft: ResMut<SimulationDraft>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let Some(next) = latest.0.as_ref().and_then(|s| s.simulation) else { return Ok(()) };
    let sim = draft.0.get_or_insert(next);
    let send = |command: String| {
        let _ = sender.0.lock().unwrap().send(command);
    };
    egui::Window::new("Simulation").default_pos([520.0, 420.0]).default_open(false).show(ctx, |ui| {
        ui.checkbox(&mut sim.gps, "GPS error").on_hover_text("Consumer-grade GNSS fixes, a meter or two off (off: the true positions).");
        ui.checkbox(&mut sim.proximity, "Drones sense each other (5 m)").on_hover_text("Onboard ranging of nearby drones (off: they keep clear of each other by the positions they broadcast alone).");
        let mut batteries = sim.battery_min.is_some();
        ui.checkbox(&mut batteries, "Batteries run down").on_hover_text("Drones go back to the base to charge in time, and charged airframes on the pads take over from them (the ground station's Charging panel).");
        if batteries {
            let mut flight = sim.battery_min.unwrap_or(12.0);
            ui.horizontal(|ui| {
                ui.label("a charge flies");
                ui.add(egui::DragValue::new(&mut flight).range(1.0..=120.0).speed(0.5).suffix(" min"));
            });
            ui.horizontal(|ui| {
                ui.label("charging takes");
                ui.add(egui::DragValue::new(&mut sim.charge_min).range(1.0..=240.0).speed(0.5).suffix(" min"));
            });
            sim.battery_min = Some(flight);
        } else {
            sim.battery_min = None;
        }
        ui.checkbox(&mut sim.random_search, "Random-walk search (to compare)").on_hover_text("Free drones wander instead of sharing a map of what's been looked over.");
        ui.separator();
        let changed = *sim != next;
        ui.horizontal(|ui| {
            if ui.add_enabled(changed, egui::Button::new("Apply to the next mission")).clicked() {
                send(format!("{{\"simulation\":{}}}", serde_json::to_string(&*sim).expect("plain data")));
            }
            if ui.button("Apply and start a new mission").clicked() {
                send(format!("{{\"simulation\":{}}}", serde_json::to_string(&*sim).expect("plain data")));
                send("\"reset\"".to_string());
            }
        });
        if changed {
            ui.label(egui::RichText::new("Not applied yet.").small().weak());
        }
    });
    Ok(())
}

/// Zooms the camera so the whole field fits the window, with a little room around it.
fn fit_camera(latest: Res<LatestState>, windows: Query<&Window>, mut cameras: Query<&mut Projection, With<Camera2d>>) {
    let (Some(state), Ok(window)) = (&latest.0, windows.single()) else { return };
    // The field, and the base's pads and drones outside it (the pads are along its edge).
    let mut half = Vec2::new(state.field_half.x, state.field_half.y);
    for p in state.parked.iter().map(|p| p.pos).chain(state.agents.iter().map(|a| a.pos)) {
        half = half.max(Vec2::new(p.x.abs(), p.y.abs()) + Vec2::splat(2.0));
    }
    let field = half * 2.0 * WORLD_TO_PIXELS;
    let fit = (field.x / window.width()).max(field.y / window.height()) * 1.08;
    for mut projection in cameras.iter_mut() {
        if let Projection::Orthographic(ortho) = &mut *projection {
            if (ortho.scale - fit).abs() > 1e-3 {
                ortho.scale = fit;
            }
        }
    }
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);

    commands.spawn((
        StatusText,
        Text::new("waiting for data..."),
        TextFont {
            font_size: 16.0.into(),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));
}

fn receive_state(receiver: Res<StateReceiver>, mut latest: ResMut<LatestState>) {
    let rx = receiver.0.lock().unwrap();
    while let Ok(state) = rx.try_recv() {
        latest.0 = Some(state);
    }
}

/// Spawns one sprite per new agent id the first time it's seen, and otherwise just moves and
/// recolors existing ones — avoids despawning/respawning every agent every frame, which would
/// otherwise make each agent's sprite "pop" instead of moving smoothly.
fn sync_agents(
    latest: Res<LatestState>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut query: Query<(Entity, &AgentMarker, &mut Transform, &MeshMaterial2d<ColorMaterial>)>,
) {
    let Some(state) = &latest.0 else { return };

    let mut existing_ids: Vec<u32> = Vec::new();
    for (entity, marker, mut transform, material_handle) in query.iter_mut() {
        existing_ids.push(marker.0);
        let Some(agent) = state.agents.iter().find(|a| a.id == marker.0) else {
            // Removed from the mission (or a reset brought fewer drones back).
            commands.entity(entity).despawn();
            continue;
        };
        {
            transform.translation.x = agent.pos.x * WORLD_TO_PIXELS;
            transform.translation.y = agent.pos.y * WORLD_TO_PIXELS;
            // Top-down view: a drone looks smaller the lower it is, so a killed one visibly falls.
            transform.scale = Vec3::splat(altitude_scale(agent.alt));
            if let Some(mut material) = materials.get_mut(&material_handle.0) {
                material.color = agent_color(agent);
            }
        }
    }

    for agent in &state.agents {
        if !existing_ids.contains(&agent.id) {
            let mesh = meshes.add(Circle::new(6.0));
            let material = materials.add(agent_color(agent));
            commands.spawn((
                AgentMarker(agent.id),
                Mesh2d(mesh),
                MeshMaterial2d(material),
                Transform::from_xyz(agent.pos.x * WORLD_TO_PIXELS, agent.pos.y * WORLD_TO_PIXELS, 2.0),
            ));
        }
    }
}

/// 1.0 at the 5 m cruise altitude, shrinking to 0.4 on the ground.
fn altitude_scale(alt: f32) -> f32 {
    0.4 + 0.6 * (alt / 5.0).clamp(0.0, 1.0)
}

/// The field's search coverage underneath everything: a cell is clear right after a drone
/// flew over it and darkens over the next minute. Spawned on the first state, since the grid
/// size comes from the backend.
fn sync_coverage(
    latest: Res<LatestState>,
    mut commands: Commands,
    mut cells: Query<(Entity, &CoverageCell, &mut Sprite)>,
    mut drawn_for: Local<Option<(f32, f32)>>,
) {
    let Some(state) = &latest.0 else { return };
    let ages = &state.coverage_age;
    if ages.is_empty() {
        return;
    }
    let shade = |age: f32| Color::srgba(0.25, 0.1, 0.1, 0.45 * (age / 60.0).clamp(0.0, 1.0));
    // A new field (a new mission on it): new cells.
    let field_half = (state.field_half.x, state.field_half.y);
    if *drawn_for != Some(field_half) {
        for (entity, _, _) in cells.iter() {
            commands.entity(entity).despawn();
        }
        *drawn_for = Some(field_half);
        let grid = (ages.len() as f32).sqrt().round() as usize;
        let field = Vec2::new(2.0 * field_half.0, 2.0 * field_half.1) * WORLD_TO_PIXELS;
        let size = field / grid as f32;
        for (i, &age) in ages.iter().enumerate() {
            let (x, y) = ((i / grid) as f32, (i % grid) as f32);
            commands.spawn((
                CoverageCell(i),
                Sprite::from_color(shade(age), size - Vec2::splat(1.0)),
                Transform::from_xyz(-field.x / 2.0 + (x + 0.5) * size.x, -field.y / 2.0 + (y + 0.5) * size.y, 0.0),
            ));
        }
        return;
    }
    for (_, cell, mut sprite) in cells.iter_mut() {
        if let Some(&age) = ages.get(cell.0) {
            sprite.color = shade(age);
        }
    }
}

/// Keeps one sprite per target: moves existing ones, spawns new targets, despawns removed ones.
fn update_targets(
    latest: Res<LatestState>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut query: Query<(Entity, &TargetMarker, &mut Transform)>,
) {
    let Some(state) = &latest.0 else { return };
    let mut shown = Vec::new();
    for (entity, marker, mut transform) in query.iter_mut() {
        match state.targets.iter().find(|t| t.id == marker.0) {
            Some(t) => {
                transform.translation.x = t.pos.x * WORLD_TO_PIXELS;
                transform.translation.y = t.pos.y * WORLD_TO_PIXELS;
                shown.push(marker.0);
            }
            None => commands.entity(entity).despawn(),
        }
    }
    for t in state.targets.iter().filter(|t| !shown.contains(&t.id)) {
        commands.spawn((
            TargetMarker(t.id),
            Mesh2d(meshes.add(Circle::new(8.0))),
            MeshMaterial2d(materials.add(TARGET_COLOR)),
            Transform::from_xyz(t.pos.x * WORLD_TO_PIXELS, t.pos.y * WORLD_TO_PIXELS, 1.0),
        ));
    }
}

/// Right-click on the field (not on a panel) puts a new target there; Shift+right-click
/// launches a new drone there instead, and Alt+right-click switches on a GPS jammer there.
fn place_on_field(
    mut contexts: EguiContexts,
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    sender: Res<CommandSender>,
) -> Result {
    if !buttons.just_pressed(MouseButton::Right) || contexts.ctx_mut()?.egui_wants_pointer_input() {
        return Ok(());
    }
    let (Ok(window), Ok((camera, camera_transform))) = (windows.single(), cameras.single()) else {
        return Ok(());
    };
    let Some(cursor) = window.cursor_position() else { return Ok(()) };
    let Ok(world) = camera.viewport_to_world_2d(camera_transform, cursor) else { return Ok(()) };
    let at = world / WORLD_TO_PIXELS;
    let what = if keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) {
        "add_drone"
    } else if keys.pressed(KeyCode::AltLeft) || keys.pressed(KeyCode::AltRight) {
        "jam"
    } else {
        "add_target"
    };
    let _ = sender.0.lock().unwrap().send(format!("{{\"{what}\":{{\"x\":{},\"y\":{}}}}}", at.x, at.y));
    Ok(())
}

/// A color per swarm target ID, so drones on the same target look alike.
fn target_color(id: u16) -> Color {
    let hue = (id as f32 * 137.508) % 360.0; // golden angle: neighbors in ID get far-apart hues
    Color::hsl(hue, 0.85, 0.6)
}

/// A thin line from every drone responding to a target to the mouse it's on, in that target's
/// color. Mice close together put two rings side by side, which otherwise looks like one crowd
/// of drones all going for one mouse. The viewer only knows the drones' target IDs (the swarm's
/// own names), not which mouse each names, so it matches each ID to the mouse nearest the
/// middle of the drones using it.
fn draw_assignments(state: &SwarmState, gizmos: &mut Gizmos) {
    let mut by_id: std::collections::BTreeMap<u16, Vec<Vec2>> = Default::default();
    for agent in state.agents.iter().filter(|a| a.alive) {
        if let Some(id) = agent.target_id.filter(|&id| id != 0xFFFF) {
            by_id.entry(id).or_default().push(Vec2::new(agent.pos.x, agent.pos.y));
        }
    }
    for (id, drones) in by_id {
        let middle = drones.iter().copied().sum::<Vec2>() / drones.len() as f32;
        let Some(mouse) = state
            .targets
            .iter()
            .map(|t| Vec2::new(t.pos.x, t.pos.y))
            .min_by(|a, b| (*a - middle).length().total_cmp(&(*b - middle).length()))
        else {
            continue;
        };
        let color = target_color(id);
        for d in drones {
            gizmos.line_2d(d * WORLD_TO_PIXELS, mouse * WORLD_TO_PIXELS, color.with_alpha(0.55));
        }
        gizmos.circle_2d(mouse * WORLD_TO_PIXELS, 12.0, color);
    }
}

/// Line overlays: a magenta ring on each liar, and a magenta cross on each fake
/// target they broadcast — so you can watch whether anyone is fooled into flying there.
fn draw_overlays(latest: Res<LatestState>, mut gizmos: Gizmos) {
    let Some(state) = &latest.0 else { return };
    // The radio network: a faint line between every two drones in the air within radio range.
    let flying: Vec<Vec2> = state.agents.iter().filter(|a| a.alive && a.alt > 1.0).map(|a| Vec2::new(a.pos.x, a.pos.y)).collect();
    for (i, a) in flying.iter().enumerate() {
        for b in &flying[i + 1..] {
            if (*a - *b).length() <= state.radio_range {
                gizmos.line_2d(*a * WORLD_TO_PIXELS, *b * WORLD_TO_PIXELS, Color::srgba(0.8, 0.85, 1.0, 0.12));
            }
        }
    }
    // And the base's radio.
    let base = Vec2::new(state.base.x, state.base.y);
    for a in &flying {
        if (*a - base).length() <= state.radio_range {
            gizmos.line_2d(*a * WORLD_TO_PIXELS, base * WORLD_TO_PIXELS, Color::srgba(0.8, 0.85, 1.0, 0.12));
        }
    }
    draw_assignments(state, &mut gizmos);
    // GPS jammers: orange circles.
    for j in &state.jammers {
        gizmos.circle_2d(Vec2::new(j.pos.x, j.pos.y) * WORLD_TO_PIXELS, j.radius * WORLD_TO_PIXELS, Color::srgb(1.0, 0.55, 0.1));
    }
    for agent in state.agents.iter().filter(|a| a.alive) {
        let truth = Vec2::new(agent.pos.x, agent.pos.y) * WORLD_TO_PIXELS;
        // A spoofed drone: yellow ring.
        if agent.spoofed {
            gizmos.circle_2d(truth, 13.0, Color::srgb(1.0, 0.9, 0.2));
        }
        // Where a drone thinks it is, when that's well off where it really is: a hollow "ghost"
        // joined to it. Grey: it has no usable GPS and knows it's unsure. Red: it trusts its
        // GPS and is wrong by far more than ordinary GPS error (about 2 m) — a spoofed drone
        // nobody has caught yet. Ordinary GPS error alone draws nothing.
        let believed = Vec2::new(agent.believed.x, agent.believed.y) * WORLD_TO_PIXELS;
        let off = (believed - truth).length() / WORLD_TO_PIXELS;
        let ghost = if agent.nav == "gps" { (off > 4.0).then_some(Color::srgb(0.95, 0.25, 0.25)) } else { (off > 1.5).then_some(Color::srgb(0.7, 0.7, 0.7)) };
        if let Some(color) = ghost {
            gizmos.line_2d(truth, believed, color.with_alpha(0.7));
            gizmos.circle_2d(believed, 6.0, color);
        }
    }
    // The field's edge.
    let field = Vec2::new(state.field_half.x, state.field_half.y) * 2.0 * WORLD_TO_PIXELS;
    gizmos.rect_2d(Vec2::ZERO, field, Color::srgba(0.8, 0.8, 0.8, 0.35));
    // The launch base: a square on its corner of the field.
    let base = Vec2::new(state.base.x, state.base.y) * WORLD_TO_PIXELS;
    gizmos.rect_2d(base, Vec2::splat(28.0), Color::srgb(0.85, 0.85, 0.85));
    for agent in state.agents.iter().filter(|a| a.alive && a.liar) {
        let pos = Vec2::new(agent.pos.x, agent.pos.y) * WORLD_TO_PIXELS;
        gizmos.circle_2d(pos, 10.0, LIAR_COLOR);
    }
    // Batteries: a bar under each drone, as long as its charge (red when low), and each
    // airframe charging on the base's pads a small square with its own bar.
    for agent in state.agents.iter().filter(|a| a.alive) {
        if let Some(charge) = agent.battery {
            battery_bar(&mut gizmos, Vec2::new(agent.pos.x, agent.pos.y) * WORLD_TO_PIXELS + Vec2::new(0.0, -14.0), charge);
        }
    }
    for parked in &state.parked {
        let pad = Vec2::new(parked.pos.x, parked.pos.y) * WORLD_TO_PIXELS;
        gizmos.rect_2d(pad, Vec2::splat(10.0), Color::srgb(0.6, 0.6, 0.65));
        battery_bar(&mut gizmos, pad + Vec2::new(0.0, -9.0), parked.battery);
    }
    for fake in &state.fake_targets {
        let pos = Vec2::new(fake.x, fake.y) * WORLD_TO_PIXELS;
        gizmos.line_2d(pos + Vec2::new(-8.0, -8.0), pos + Vec2::new(8.0, 8.0), LIAR_COLOR);
        gizmos.line_2d(pos + Vec2::new(-8.0, 8.0), pos + Vec2::new(8.0, -8.0), LIAR_COLOR);
    }
}

/// A battery gauge centered on `at`: full width at a full charge, green, going red as it runs
/// down past a third.
fn battery_bar(gizmos: &mut Gizmos, at: Vec2, charge: f32) {
    const WIDTH: f32 = 16.0;
    let charge = charge.clamp(0.0, 1.0);
    let color = if charge > 0.5 {
        Color::srgb(0.3, 0.85, 0.35)
    } else if charge > 0.3 {
        Color::srgb(0.95, 0.8, 0.2)
    } else {
        Color::srgb(0.95, 0.3, 0.25)
    };
    let left = at - Vec2::new(WIDTH / 2.0, 0.0);
    gizmos.line_2d(left, left + Vec2::new(WIDTH, 0.0), Color::srgba(0.5, 0.5, 0.5, 0.4));
    gizmos.line_2d(left, left + Vec2::new(WIDTH * charge, 0.0), color);
}

fn update_status_text(latest: Res<LatestState>, mut query: Query<&mut Text, With<StatusText>>) {
    let Some(state) = &latest.0 else { return };
    let alive_count = state.agents.iter().filter(|a| a.alive).count();
    let informed_count = state.agents.iter().filter(|a| a.alive && a.informed).count();
    let in_mode = |mode: &str| state.agents.iter().filter(|a| a.alive && a.mode == mode).count();
    let liars = state.agents.iter().filter(|a| a.alive && a.liar).count();
    let in_role = |role: &str| state.agents.iter().filter(|a| a.alive && a.role == role).count();
    // Which drones are on which target, by the swarm's own target IDs (hex: first spotter's
    // sysid, then its counter) — the names an operator would use.
    let mut assignments: std::collections::BTreeMap<u16, Vec<u32>> = Default::default();
    for agent in state.agents.iter().filter(|a| a.alive) {
        if let Some(id) = agent.target_id {
            assignments.entry(id).or_default().push(agent.id);
        }
    }
    let assignments: String = assignments
        .iter()
        .map(|(id, drones)| format!("\ntarget {id:04x}: drones {drones:?}"))
        .collect();
    // With batteries: the lowest charge in the air, and the airframes on the pads.
    let batteries = state.agents.iter().filter(|a| a.alive).filter_map(|a| a.battery).fold(None, |low: Option<f32>, b| Some(low.map_or(b, |l| l.min(b))));
    let charging = match batteries {
        Some(lowest) => format!(
            "\nlowest battery in the air: {:.0}%\ncharging on the pads: {} ({} ready)",
            100.0 * lowest,
            state.parked.len(),
            state.parked.iter().filter(|p| p.battery >= 0.95).count()
        ),
        None => String::new(),
    };
    if let Ok(mut text) = query.single_mut() {
        text.0 = format!(
            "seq={}\nalive: {}/{}\ninformed: {}/{}\nconverging: {}  investigating: {}  searching: {}\nrelaying (white): {}  sweeping (light blue): {}\nliars: {}\nlaunch queue: {}\ntargets: {}{}",
            state.seq,
            alive_count,
            state.agents.len(),
            informed_count,
            alive_count,
            in_mode("converging"),
            in_mode("investigating"),
            in_mode("searching"),
            in_role("relay"),
            in_role("sweep"),
            liars,
            state.launch_queue,
            state.targets.len(),
            charging + &assignments
        );
    }
}
