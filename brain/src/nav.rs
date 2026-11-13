//! Where the drone is: GPS checked against its neighbors, or worked out without it.

use super::*;

/// Where a drone's idea of its own position comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Nav {
    /// Its GPS fix, agreeing with its neighbors (or with no neighbors to check against).
    Gps,
    /// No usable GPS (jammed, or rejected as spoofed): placed by neighbors it can range, from
    /// where they say they are.
    Neighbors,
    /// No usable GPS and no one to range: carried forward on its own velocity, drifting.
    DeadReckoning,
}

impl Nav {
    /// Wire encoding used by SWARM_DRONE_STATUS.nav and SWARM_HIL_COMMAND.nav.
    pub fn to_u8(self) -> u8 {
        match self {
            Nav::Gps => 0,
            Nav::Neighbors => 1,
            Nav::DeadReckoning => 2,
        }
    }

    pub fn from_u8(value: u8) -> Nav {
        match value {
            1 => Nav::Neighbors,
            2 => Nav::DeadReckoning,
            _ => Nav::Gps,
        }
    }
}

/// How sure a drone with a GPS fix is of its position, m.
pub(crate) const GPS_ACCURACY_M: f32 = 2.0;

impl Brain {
    /// Where are we? GPS if it has a fix that agrees with the neighbors; else placed by the
    /// neighbors we can range; else carried forward on our own velocity (dead reckoning),
    /// growing less sure by `dead_reckoning_drift` every second.
    ///
    /// The check against the neighbors is what catches spoofing: a spoofed receiver drags the
    /// drone's idea of where it is away slowly, and on its own nothing looks wrong, but the
    /// neighbors it can range — each sure of its own position and seen at a measured offset
    /// from us — say otherwise. A drone with no one to check against can't tell.
    pub(crate) fn update_position(&mut self, r: &SensorReading, now: u32) -> Vec2 {
        let dt = self.last_tick_ms.map_or(0.0, |t| now.saturating_sub(t) as f32 / 1000.0);
        self.last_tick_ms = Some(now);
        let predicted = self.estimate.map(|e| e + r.own_velocity * dt);
        let from_neighbors = self.position_from_neighbors(now);
        let gps = r.gps_fix.then_some(r.own_pos);
        if let (Some(g), Some((n, accuracy, refs))) = (gps, from_neighbors) {
            if refs >= 2 && (g - n).length() > self.cfg.gps_disagreement_m + accuracy {
                self.gps_rejected_until = now.saturating_add(self.cfg.gps_reject_ms);
            }
        }
        let gps = gps.filter(|_| now >= self.gps_rejected_until);
        let (estimate, accuracy, nav) = match (gps, from_neighbors, predicted) {
            (Some(g), _, _) => (g, GPS_ACCURACY_M, Nav::Gps),
            (None, Some((n, accuracy, _)), _) => (n, accuracy, Nav::Neighbors),
            (None, None, Some(p)) => (p, self.pos_accuracy + self.cfg.dead_reckoning_drift * dt, Nav::DeadReckoning),
            // Never had a fix: we were launched from the base.
            (None, None, None) => (self.cfg.home, GPS_ACCURACY_M, Nav::DeadReckoning),
        };
        self.estimate = Some(estimate);
        self.pos_accuracy = accuracy;
        self.nav = nav;
        estimate
    }

    /// Our position as the drones we can range put it: each one's broadcast position minus the
    /// offset we measure to it, averaged weighting the surer ones more. Only neighbors sure of
    /// their own position to `reference_accuracy_m` count. Returns (position, how sure, how many
    /// neighbors it rests on).
    pub(crate) fn position_from_neighbors(&self, now: u32) -> Option<(Vec2, f32, usize)> {
        let nearby = self.nearby?;
        let (mut sum, mut weight, mut count) = (Vec2::ZERO, 0.0f32, 0usize);
        for (sysid, rel, _) in nearby.into_iter().flatten() {
            let Some(n) = self.neighbors[sysid as usize] else { continue };
            let (Some((pos, vel, at)), Some(accuracy)) = (n.position, n.pos_accuracy) else { continue };
            let age_ms = now.saturating_sub(at);
            if age_ms > (self.beacon_now_ms + 150).max(500) || accuracy > self.cfg.reference_accuracy_m {
                continue;
            }
            let theirs = pos + vel * (age_ms as f32 / 1000.0);
            let w = 1.0 / (accuracy * accuracy).max(0.01);
            sum = sum + (theirs - rel) * w;
            weight += w;
            count += 1;
        }
        (count > 0).then(|| (sum * (1.0 / weight), libm::sqrtf(1.0 / weight) + 0.3, count))
    }
}
