//! Keeping clear of other drones.

use super::*;

/// Most drones collision avoidance looks at at once (radio neighbors past this are ignored;
/// in a 12-drone swarm, never more than a handful are anywhere near).
const MAX_OBSTACLES: usize = 16;

impl Brain {
    /// The drones collision avoidance has to keep clear of, as (offset from them to us, their
    /// velocity): from the proximity sensing when the drone has it — relative measurements, so
    /// GPS error doesn't make two drones think they're further apart than they are — else from
    /// the positions neighbors broadcast. Those are up to a beacon period old, so each is carried
    /// forward along its reported velocity to now: a drone dodges where its neighbors are, not
    /// where they were.
    pub(crate) fn obstacles(&self, me: Vec2, now: u32) -> [Option<(Vec2, Vec2)>; MAX_OBSTACLES] {
        let mut out = [None; MAX_OBSTACLES];
        if let Some(nearby) = self.nearby {
            for (slot, (_, rel_pos, rel_vel)) in out.iter_mut().zip(nearby.into_iter().flatten()) {
                *slot = Some((Vec2::ZERO - rel_pos, self.last_velocity + rel_vel));
            }
            return out;
        }
        let radio = self.neighbors.iter().flatten().filter_map(|n| n.position).filter_map(|(pos, vel, heard_at_ms)| {
            let age_ms = now.saturating_sub(heard_at_ms);
            (age_ms <= self.cfg.neighbor_timeout_ms).then(|| (me - (pos + vel * (age_ms as f32 / 1000.0)), vel))
        });
        for (slot, o) in out.iter_mut().zip(radio) {
            *slot = Some(o);
        }
        out
    }

    /// Collision avoidance: sum of pushes away from every drone (see `obstacles`) that is
    /// predicted to come within the separation radius in the next `avoidance_horizon_ms`.
    /// Each push grows from 0 at the edge of that radius to 1 at contact, and is weaker the
    /// further ahead the close approach is. Prediction assumes both keep their current velocity
    /// (ours as commanded, theirs as sensed or last reported).
    pub(crate) fn separation(&self, me: Vec2, now: u32) -> Vec2 {
        let r = self.cfg.separation_radius;
        let horizon = self.cfg.avoidance_horizon_ms as f32 / 1000.0;
        let mut push = Vec2::ZERO;
        for (offset, vel) in self.obstacles(me, now).into_iter().flatten() {
            let closing = self.last_velocity - vel;
            // Time of closest approach, limited to [now, horizon].
            let closing_sq = closing.x * closing.x + closing.y * closing.y;
            let t = if closing_sq > 1e-6 {
                (-(offset.x * closing.x + offset.y * closing.y) / closing_sq).clamp(0.0, horizon)
            } else {
                0.0
            };
            let closest = offset + closing * t;
            let distance = closest.length();
            if distance >= r {
                continue;
            }
            // Dead-on courses (or two drones at the same point) give no "away" direction; dodge
            // sideways instead, to a side picked by sysid so the pair doesn't mirror each other.
            let dir = if distance > 1e-3 {
                closest * (1.0 / distance)
            } else {
                let side = if self.sysid.is_multiple_of(2) { 1.0 } else { -1.0 };
                let lateral = Vec2::new(-closing.y, closing.x).normalized();
                if lateral == Vec2::ZERO {
                    Vec2::new(side, 0.0)
                } else {
                    lateral * side
                }
            };
            let urgency = 1.0 - 0.5 * t / horizon.max(1e-3);
            push = push + dir * ((r - distance) / r * urgency);
        }
        push
    }

    /// The hard part of collision avoidance: never close in on a neighbor faster than this
    /// drone could brake before reaching it. `separation` steers around neighbors, but a real
    /// airframe needs room to stop, so the closing speed toward each neighbor is capped by the
    /// remaining gap. Both drones of a pair apply the same rule, so each removes only half of
    /// the excess, like reciprocal collision avoidance (ORCA).
    pub(crate) fn limit_closing_speed(&self, me: Vec2, mut velocity: Vec2, now: u32) -> Vec2 {
        // Half a separation radius is roughly where two airframes' rotor guards touch.
        let min_gap = 0.5 * self.cfg.separation_radius;
        // Time an airframe needs to stop from a given speed, roughly its velocity loop's response.
        let brake_time = 0.5 * self.cfg.avoidance_horizon_ms as f32 / 1000.0;
        for (offset, vel) in self.obstacles(me, now).into_iter().flatten() {
            let distance = offset.length();
            if distance < 1e-3 {
                continue; // no direction to brake along; `separation` splits coincident drones
            }
            let away = offset * (1.0 / distance);
            let closing = -((velocity - vel).x * away.x + (velocity - vel).y * away.y);
            let allowed = (distance - min_gap).max(0.0) / brake_time;
            if closing > allowed {
                velocity = velocity + away * (0.5 * (closing - allowed));
            }
        }
        velocity
    }
}
