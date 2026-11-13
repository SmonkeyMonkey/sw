//! The battery: how fast it's going down, and when to go home to charge.

use super::*;

/// Going home to charge (see `Brain::battery_low`): how often it works out how fast its battery
/// is going down (ms); how much charge it lands with at least; how many times over what the
/// flight home takes it keeps; and when it goes home before it knows the rate.
pub(crate) const BATTERY_SAMPLE_MS: u32 = 5_000;
pub(crate) const BATTERY_RESERVE: f32 = 0.10;
pub(crate) const BATTERY_MARGIN: f32 = 1.5;
pub(crate) const BATTERY_UNKNOWN_RATE: f32 = 0.30;
/// The flight home: at this share of top speed (around others, into the wind), and this long
/// (s) to wait for a pad and land on it.
pub(crate) const HOMEWARD_SPEED_SHARE: f32 = 0.8;
pub(crate) const LANDING_S: f32 = 30.0;
/// Up to how much more charge some keep (see `Brain::stagger`), in how many steps.
pub(crate) const BATTERY_STAGGER: f32 = 0.2;
pub(crate) const BATTERY_STAGGER_GROUPS: u8 = 4;

/// Landed to charge, this charged it's flying again (see `Brain::decide`).
pub(crate) const RELAUNCH_CHARGE: f32 = 0.9;

impl Brain {
    /// Takes in this tick's battery reading, and every `BATTERY_SAMPLE_MS` how fast it went
    /// down since the last sample, smoothed: by what it measures, not by a model of the battery,
    /// so a heavier load, wind or an older battery are all in it. (Going up, on a charger, it
    /// isn't drained.)
    pub(crate) fn update_battery(&mut self, level: f32, now: u32) {
        if !level.is_finite() {
            self.battery = None;
            return;
        }
        self.battery = Some(level);
        match self.battery_sample {
            Some((was, at)) if now.saturating_sub(at) >= BATTERY_SAMPLE_MS => {
                let rate = (was - level) / (now.saturating_sub(at) as f32 / 1000.0);
                if rate > 0.0 {
                    self.drain_per_s = Some(self.drain_per_s.map_or(rate, |r| r + (rate - r) * 0.3));
                }
                self.battery_sample = Some((level, now));
            }
            None => self.battery_sample = Some((level, now)),
            _ => {}
        }
    }

    /// Whether it's time to go back to the base to charge: what's left is what the flight home
    /// takes, at the rate it's been using charge, `BATTERY_MARGIN` times over, and
    /// `BATTERY_RESERVE` besides. Before it knows the rate, at `BATTERY_UNKNOWN_RATE`. Never, if
    /// it doesn't know its charge.
    pub(crate) fn battery_low(&self, pos: Vec2) -> bool {
        let Some(level) = self.battery else { return false };
        let need = match self.drain_per_s {
            Some(rate) => {
                let home_s = (self.cfg.home - pos).length() / (HOMEWARD_SPEED_SHARE * self.cfg.max_speed) + LANDING_S;
                BATTERY_RESERVE + self.stagger() + rate * home_s * BATTERY_MARGIN
            }
            None => BATTERY_UNKNOWN_RATE,
        };
        level <= need
    }

    /// Charge it keeps besides, by system ID: drones launched together with full batteries
    /// would otherwise all come back to charge together, again and again, and leave the field
    /// to a few (8 drones with 4 spares: down to 4 in the air at once). Flights a little longer
    /// or shorter by system ID drift apart.
    pub(crate) fn stagger(&self) -> f32 {
        BATTERY_STAGGER * (self.sysid % BATTERY_STAGGER_GROUPS) as f32 / BATTERY_STAGGER_GROUPS as f32
    }

    /// The battery's charge, as the drone last read it (None if it doesn't know it).
    pub fn battery(&self) -> Option<f32> {
        self.battery
    }

    /// Whether it's going back to the base to charge.
    pub fn recharging(&self) -> bool {
        self.recharging
    }
}
