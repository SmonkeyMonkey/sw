//! The remote controller, like a DJI Mavic's: a row of lights, one per drone, to see them all at
//! a glance and pick one; a stick to fly the picked drone by hand; and how many drones the swarm
//! sends to each target.
//!
//! Flying by hand is standard MAVLink: `MANUAL_CONTROL` ("the sticks") ten times a second to the
//! drone, relayed by the swarm if it's out of the ground station's own range. Centered sticks
//! mean hover. If the sticks' signal stops reaching it for a second, the drone hovers where it
//! is (its light blinks red) until it comes back or it's released back to the swarm.
//!
//! If they stay lost for 5 s, it rejoins the swarm (out of contact, the swarm's own rule takes
//! it back toward the base), and the remote lets go of it.
//!
//! Keys: 1-9 and 0 pick drone 1-10, Enter takes control of it, arrows or WASD fly it, Space
//! hovers, Esc hands it (or the picked drone) back to the swarm.

use std::time::{Duration, Instant};

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use mavlink_swarm::{MavCmd, MavMessage, COMMAND_LONG_DATA, MANUAL_CONTROL_DATA};
use swarm_gcs::link::Link;

use crate::{Gcs, Links};

/// How often the sticks are sent while flying a drone by hand.
const STICK_EVERY: Duration = Duration::from_millis(100);
/// How often the drones-per-target setting is sent again, for drones that missed it or joined
/// since.
const RESPONDERS_EVERY: Duration = Duration::from_secs(5);

#[derive(Resource)]
pub struct Remote {
    /// The drone being flown by hand, if any, and whether it has said so yet (its status shows
    /// manual control).
    controlling: Option<u8>,
    confirmed: bool,
    /// Where the stick is pushed: x east, y north, each -1..1.
    stick: egui::Vec2,
    next_stick: Instant,
    /// Numbers each stick frame (sent in `buttons`), so drones can tell new input from old
    /// copies still echoing round the relays.
    stick_seq: u16,
    /// Drones per target, and when it was last sent (with its repeat counter).
    pub(crate) responders: u8,
    /// Let the drones adapt how many go to each mouse to how many mice there are, up to
    /// `responders` (see `BrainConfig::adaptive_responders`).
    pub(crate) adaptive: bool,
    /// The swarm's mode (see `swarm_brain::SwarmMode`), once the operator has picked one: until
    /// then the ground station leaves the drones in the mode they started in.
    pub(crate) mode: Option<swarm_brain::SwarmMode>,
    responders_sent: Option<(Instant, u8)>,
}

impl Default for Remote {
    fn default() -> Self {
        Self { controlling: None, confirmed: false, stick: egui::Vec2::ZERO, next_stick: Instant::now(), stick_seq: 0, responders: 2, adaptive: false, mode: None, responders_sent: None }
    }
}

/// Keyboard: picking a drone, taking and releasing control, the stick.
pub fn keys(mut contexts: EguiContexts, keys: Res<ButtonInput<KeyCode>>, mut remote: ResMut<Remote>, mut gcs: ResMut<Gcs>, links: Res<Links>) -> Result {
    // Typing into a text field isn't flying.
    if contexts.ctx_mut()?.egui_wants_keyboard_input() {
        return Ok(());
    }
    let digits = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Digit7,
        KeyCode::Digit8,
        KeyCode::Digit9,
        KeyCode::Digit0,
    ];
    for (k, key) in digits.iter().enumerate() {
        if keys.just_pressed(*key) {
            gcs.selected = Some(k as u8 + 1);
        }
    }
    if keys.just_pressed(KeyCode::Enter) {
        take_control(&mut remote, &mut gcs);
    }
    if keys.just_pressed(KeyCode::Escape) {
        release(&mut remote, &mut gcs, &links);
    }
    let pressed = |a: KeyCode, b: KeyCode| keys.pressed(a) || keys.pressed(b);
    let x = pressed(KeyCode::ArrowRight, KeyCode::KeyD) as i32 - pressed(KeyCode::ArrowLeft, KeyCode::KeyA) as i32;
    let y = pressed(KeyCode::ArrowUp, KeyCode::KeyW) as i32 - pressed(KeyCode::ArrowDown, KeyCode::KeyS) as i32;
    remote.stick = egui::vec2(x as f32, y as f32);
    if keys.pressed(KeyCode::Space) {
        remote.stick = egui::Vec2::ZERO;
    }
    Ok(())
}

fn take_control(remote: &mut Remote, gcs: &mut Gcs) {
    if let Some(id) = gcs.selected {
        remote.controlling = Some(id);
        remote.confirmed = false;
        // Sticks centered: it starts by hovering where it is.
        remote.stick = egui::Vec2::ZERO;
        gcs.note(format!("→ drone {id}: manual control"));
    }
}

/// Hands the drone being flown back to the swarm — or, if none, the picked one: one left
/// hovering where the release didn't reach it (out of contact at the time) can be released
/// again once it's back.
fn release(remote: &mut Remote, gcs: &mut Gcs, links: &Links) {
    if let Some(id) = remote.controlling.take().or(gcs.selected) {
        gcs.command(links, Link::Radio, id, MavCmd::MAV_CMD_MISSION_START, [0.0; 7], format!("drone {id}: back to the swarm"));
    }
}

/// Sends the sticks while flying a drone, and the drones-per-target setting now and then.
pub fn transmit(mut remote: ResMut<Remote>, mut gcs: ResMut<Gcs>, links: Res<Links>) {
    let now = Instant::now();
    if let Some(order) = remote.controlling.and_then(|id| gcs.drones.get(&id)).map(|d| d.order) {
        if order == 4 || order == 5 {
            remote.confirmed = true;
        } else if remote.confirmed {
            // It gave up on the sticks (lost for too long) and rejoined the swarm, or someone
            // gave it another order: it's not ours to fly any more.
            let id = remote.controlling.take().unwrap_or_default();
            gcs.note(format!("drone {id} left manual control"));
        }
    }
    if let Some(id) = remote.controlling {
        if now >= remote.next_stick {
            remote.next_stick = now + STICK_EVERY;
            remote.stick_seq = remote.stick_seq.wrapping_add(1);
            let s = remote.stick.normalized() * remote.stick.length().min(1.0);
            links.send(
                Link::Radio,
                MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
                    // Forward is north here: the drones have no heading to be relative to.
                    x: (s.y * 1000.0) as i16,
                    y: (s.x * 1000.0) as i16,
                    z: 500,
                    r: 0,
                    buttons: remote.stick_seq,
                    target: id,
                }),
            );
        }
    }
    let due = remote.responders_sent.is_none_or(|(at, _)| now.duration_since(at) >= RESPONDERS_EVERY);
    if due {
        let repeat = remote.responders_sent.map_or(0, |(_, n)| n.wrapping_add(1));
        remote.responders_sent = Some((now, repeat));
        send_responders(&links, remote.responders, remote.adaptive, repeat);
        if let Some(mode) = remote.mode {
            send_mode(&links, mode, repeat);
        }
        // Cancels too, while the swarm still reports the target: only the command itself
        // cancels one (a drone won't take another drone's word for it), and a drone that joined
        // since, or missed it, hears it this way.
        let cancelled: Vec<u16> = gcs.targets.iter().filter(|(_, t)| t.cancelled).map(|(&id, _)| id).collect();
        for id in cancelled {
            send_cancel(&links, id, repeat);
        }
    }
}

fn send_cancel(links: &Links, id: u16, repeat: u8) {
    links.send(
        Link::Radio,
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: id as f32,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MavCmd::MAV_CMD_SWARM_CANCEL_TARGET,
            target_system: 0,
            target_component: 0,
            confirmation: repeat,
        }),
    );
}

fn send_responders(links: &Links, responders: u8, adaptive: bool, repeat: u8) {
    links.send(
        Link::Radio,
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: responders as f32,
            param2: if adaptive { 1.0 } else { 0.0 },
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MavCmd::MAV_CMD_SWARM_SET_RESPONDERS,
            target_system: 0,
            target_component: 0,
            // A new value each time, so drones that relayed the last one relay this one too.
            confirmation: repeat,
        }),
    );
}

/// The swarm's mode, to all drones; repeated with the drones per target, for drones that missed
/// it or joined since.
pub(crate) fn send_mode(links: &Links, mode: swarm_brain::SwarmMode, repeat: u8) {
    links.send(
        Link::Radio,
        MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: mode.to_u8() as f32,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MavCmd::MAV_CMD_SWARM_SET_MODE,
            target_system: 0,
            target_component: 0,
            confirmation: repeat,
        }),
    );
}

/// What a drone's light shows.
fn light(order: u8, mode: u8, stale: bool, blink_on: bool) -> egui::Color32 {
    if stale {
        egui::Color32::from_rgb(70, 70, 70)
    } else if order == 5 {
        // Flown by hand, sticks' signal lost: hovering. Blinks.
        if blink_on { egui::Color32::from_rgb(235, 50, 50) } else { egui::Color32::from_rgb(90, 20, 20) }
    } else if order == 4 {
        egui::Color32::from_rgb(60, 140, 255)
    } else if order != 0 {
        // Some other operator order (go to, hold, return).
        egui::Color32::from_rgb(170, 110, 255)
    } else if mode == 1 || mode == 2 {
        egui::Color32::from_rgb(240, 160, 30)
    } else {
        egui::Color32::from_rgb(70, 210, 90)
    }
}

pub fn panel(mut contexts: EguiContexts, mut remote: ResMut<Remote>, mut gcs: ResMut<Gcs>, links: Res<Links>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let now = Instant::now();
    let blink_on = gcs.started.elapsed().as_millis() % 1000 < 500;
    egui::Window::new("Remote").default_pos([20.0, 470.0]).resizable(false).show(ctx, |ui| {
        // The lights, one per drone: click (or press its number) to pick it.
        ui.horizontal_wrapped(|ui| {
            let ids: Vec<u8> = gcs.drones.keys().copied().collect();
            for id in ids {
                let d = &gcs.drones[&id];
                let stale = now.duration_since(d.seen_at) > crate::STALE;
                let color = light(d.order, d.mode, stale, blink_on);
                let (rect, response) = ui.allocate_exact_size(egui::vec2(30.0, 38.0), egui::Sense::click());
                let center = rect.center_top() + egui::vec2(0.0, 13.0);
                let painter = ui.painter();
                painter.circle_filled(center, 9.0, color);
                if gcs.selected == Some(id) {
                    painter.circle_stroke(center, 12.5, egui::Stroke::new(2.5, egui::Color32::WHITE));
                }
                let label_color = if remote.controlling == Some(id) { egui::Color32::from_rgb(60, 140, 255) } else { egui::Color32::LIGHT_GRAY };
                painter.text(rect.center_bottom(), egui::Align2::CENTER_BOTTOM, id.to_string(), egui::FontId::monospace(11.0), label_color);
                if response.clicked() {
                    gcs.selected = Some(id);
                }
            }
        });
        ui.label(
            egui::RichText::new("● searching  ● on a target  ● yours (manual)  ● other order  ● sticks lost  ● no contact")
                .small()
                .weak(),
        );
        ui.separator();

        ui.horizontal(|ui| {
            // The stick: where it's pushed.
            let (rect, _) = ui.allocate_exact_size(egui::vec2(84.0, 84.0), egui::Sense::hover());
            let painter = ui.painter();
            painter.circle_stroke(rect.center(), 38.0, egui::Stroke::new(1.5, egui::Color32::GRAY));
            painter.line_segment([rect.center() - egui::vec2(38.0, 0.0), rect.center() + egui::vec2(38.0, 0.0)], egui::Stroke::new(0.5, egui::Color32::DARK_GRAY));
            painter.line_segment([rect.center() - egui::vec2(0.0, 38.0), rect.center() + egui::vec2(0.0, 38.0)], egui::Stroke::new(0.5, egui::Color32::DARK_GRAY));
            let s = remote.stick.normalized() * remote.stick.length().min(1.0);
            painter.circle_filled(rect.center() + egui::vec2(s.x, -s.y) * 30.0, 8.0, egui::Color32::from_rgb(60, 140, 255));

            ui.vertical(|ui| {
                match (gcs.selected, remote.controlling) {
                    (_, Some(id)) => ui.label(egui::RichText::new(format!("Flying drone {id}")).strong()),
                    (Some(id), None) => ui.label(format!("Drone {id} picked")),
                    (None, None) => ui.label("Pick a drone (click a light, or 1-9, 0)"),
                };
                ui.horizontal(|ui| {
                    if ui.add_enabled(gcs.selected.is_some(), egui::Button::new("Take control (Enter)")).clicked() {
                        take_control(&mut remote, &mut gcs);
                    }
                    if ui.add_enabled(remote.controlling.is_some() || gcs.selected.is_some(), egui::Button::new("Release (Esc)")).clicked() {
                        release(&mut remote, &mut gcs, &links);
                    }
                });
                ui.label(egui::RichText::new("Arrows / WASD fly, Space hovers").small().weak());
            });
        });
        ui.separator();

        let mut responders = remote.responders;
        let mut adaptive = remote.adaptive;
        let slider = ui.add(egui::Slider::new(&mut responders, 1..=8).text(if adaptive { "drones per mouse, at most" } else { "drones per mouse" }));
        let check = ui.checkbox(&mut adaptive, "Adapt to the number of mice").on_hover_text(
            "The drones share themselves out over the mice they know of (and one not found yet), down to one per mouse: with as many mice as drones, each drone takes one.",
        );
        if slider.changed() || check.changed() {
            remote.responders = responders;
            remote.adaptive = adaptive;
            // Now, not at the next repeat.
            let repeat = remote.responders_sent.map_or(0, |(_, n)| n.wrapping_add(1));
            remote.responders_sent = Some((now, repeat));
            send_responders(&links, responders, adaptive, repeat);
            gcs.note(if adaptive { format!("→ up to {responders} drones per mouse, adapting") } else { format!("→ {responders} drones per mouse") });
        }
    });
    Ok(())
}
