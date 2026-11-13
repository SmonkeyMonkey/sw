//! Searching without a formation: cells, the radar sweep, rallying, wandering.

use super::*;

/// Coverage map resolution: the field as GRID x GRID cells (3 m cells on the 30 m field, about
/// one sensor footprint each).
pub const GRID: usize = 10;

/// Mesh nodes keep this far (m) inside the field's edge.
pub(crate) const SWEEP_EDGE_MARGIN: f32 = 1.0;

/// How fast (m/s) the outermost node of a radar sweep moves (see `Brain::sweep_nodes`): well
/// within a drone's top speed, so it keeps up.
pub(crate) const SWEEP_SPEED: f32 = 1.0;

/// Radar sweep nodes worked out at most (see `Brain::sweep_nodes`).
pub(crate) const MAX_SWEEP_NODES: usize = 128;

impl Brain {
    /// True if this drone hasn't heard any other drone for `isolation_timeout_ms`.
    pub fn isolated(&self, now: u32) -> bool {
        let last_heard = self
            .neighbors
            .iter()
            .flatten()
            .map(Neighbor::last_heard_ms)
            .max();
        match last_heard.or(self.last_contact.map(|(_, _, t)| t)) {
            Some(t) => now.saturating_sub(t) > self.cfg.isolation_timeout_ms,
            None => now > self.cfg.isolation_timeout_ms,
        }
    }

    /// Searching, with no target to respond to. News only spreads over connected radio links,
    /// and once informed drones converge on a target they can leave a straggler out of range of
    /// everyone — it would then search alone and might never hear about anything. So a drone
    /// that has lost contact flies back toward the swarm instead, and resumes searching once
    /// it's back in range (or reaches the rally point and still hears no one).
    pub(crate) fn search(&mut self, pos: Vec2, now: u32) -> Vec2 {
        // Not one with a place in a front: the place is where its front is, by the same clock
        // for all of them, so that's the way back to them, not where it last heard them.
        if let Some(rally) = self.rally_point(now).filter(|_| self.formation_spot(now).is_none()) {
            let offset = rally - pos;
            let distance = offset.length();
            if distance > 1.0 {
                return offset * (self.cfg.max_speed / distance);
            }
        }
        match self.cfg.search {
            SearchStrategy::RandomWalk => self.wander(pos),
            SearchStrategy::Coverage => self.cover(pos, now),
        }
    }

    pub(crate) fn cell_of(&self, p: Vec2) -> Option<(usize, usize)> {
        let h = self.cfg.field_half;
        let (cx, cy) = ((p.x + h.x) / (2.0 * h.x / GRID as f32), (p.y + h.y) / (2.0 * h.y / GRID as f32));
        let in_field = (0.0..GRID as f32).contains(&cx) && (0.0..GRID as f32).contains(&cy);
        in_field.then_some((cx as usize, cy as usize))
    }

    pub(crate) fn cell_center(&self, (x, y): (usize, usize)) -> Vec2 {
        let h = self.cfg.field_half;
        Vec2::new(-h.x + (x as f32 + 0.5) * (2.0 * h.x / GRID as f32), -h.y + (y as f32 + 0.5) * (2.0 * h.y / GRID as f32))
    }

    /// Everything within `scan_radius` of `p` has just been looked at.
    pub(crate) fn mark_searched(&mut self, p: Vec2, now: u32) {
        let Some((x0, y0)) = self.cell_of(p) else { return };
        let cell = 2.0 * self.cfg.field_half.x.min(self.cfg.field_half.y) / GRID as f32;
        let reach = 1 + (self.cfg.scan_radius / cell) as usize;
        for x in x0.saturating_sub(reach)..(x0 + reach + 1).min(GRID) {
            for y in y0.saturating_sub(reach)..(y0 + reach + 1).min(GRID) {
                if (self.cell_center((x, y)) - p).length() <= self.cfg.scan_radius {
                    self.searched_at[x][y] = now as LocalMs;
                }
            }
        }
    }

    /// Whether a free neighbor (one not busy with a target, or relaying in a chain) is closer
    /// to `spot` than we are — then it's that neighbor's to search, or to relay for, not ours.
    /// Relays don't count: a relay is busy where it is, and counting it as free would keep the
    /// drones nearer the base from answering a chain's call for one more.
    pub(crate) fn claimed_by_closer_neighbor(&self, spot: Vec2, me: Vec2, now: u32) -> bool {
        let mine = (spot - me).length();
        let neighbor = self.neighbors.iter().flatten().any(|n| {
            n.assigned.is_none()
                && !(self.cfg.relay_chains && n.chain)
                && n.position.is_some_and(|(pos, _, heard_at_ms)| {
                    now.saturating_sub(heard_at_ms) <= self.cfg.neighbor_timeout_ms && (spot - pos).length() < mine
                })
        });
        neighbor || (self.cfg.relay_chains && self.free_drone_beyond_earshot_nearer(spot, mine, now))
    }

    /// With relay chains: a free drone we don't hear ourselves, but know of by its status from
    /// across the swarm, that's nearer `spot` than `mine`. Each free drone then searches its
    /// own part of the whole field (the ground nearer to it than to any other free drone), not
    /// just of the ground it shares with the neighbors it hears, so no two drones cover one part
    /// of the field while none covers another.
    pub(crate) fn free_drone_beyond_earshot_nearer(&self, spot: Vec2, mine: f32, now: u32) -> bool {
        (1..MAX_SYSIDS).filter(|&k| k != self.sysid as usize).any(|k| {
            let heard = self.neighbors[k].as_ref().is_some_and(|n| now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms);
            !heard
                && self.swarm_heard[k].is_some_and(|(at, free, pos)| free && now.saturating_sub(at) <= FREE_DRONE_FRESH_MS && (spot - pos).length() < mine)
        })
    }

    /// With relay chains: where the free drones left over fly — a sweep, like a radar's beam.
    /// They line up in rays out from the base, `link_range - 0.5` apart (so each is in radio
    /// range of the next, and the innermost of the base), and the rays swing back and forth
    /// across the field around the base, each over its own sector: the whole field is looked
    /// over, and the swarm stays one network all the while. As many nodes as `count`, innermost
    /// first (any number taken that way are connected); one ray if that's all there are drones
    /// for, more (in narrower sectors) as there are more. On every other pass the nodes shift
    /// out by half a spacing, to cover the strips between their circles on the way out.
    ///
    /// Every drone works this out from the time, so they need a common clock: the simulator's
    /// here, GPS time in the field.
    pub(crate) fn sweep_nodes(&self, count: usize, now: u32) -> [Option<Vec2>; MAX_SWEEP_NODES] {
        let mut nodes = [None; MAX_SWEEP_NODES];
        let count = count.min(MAX_SWEEP_NODES);
        if count == 0 {
            return nodes;
        }
        let SweepGeometry { spacing, half, from, span, reach, per_ray } = self.cfg.sweep_geometry();
        let home = self.cfg.home;
        let inside = |p: Vec2| p.x.abs() <= half.x && p.y.abs() <= half.y;
        // Only as many rays as there are drones for full ones: a ray cut short doesn't reach the
        // far side of the field. The drones a ray has no node for (toward the field's edges it
        // holds fewer) search the usual way, by the coverage map: they're what fills in what the
        // rays leave. (Rounding the number of rays up left both of two rays short of the far
        // side; a second row on a ray put those drones on its inner nodes, by the base.)
        let rays = (count / per_ray).max(1);
        let sector = span / rays as f32;
        // Back and forth over its sector, the outermost node at `SWEEP_SPEED`.
        let pass_s = (sector * reach / SWEEP_SPEED).max(1.0);
        let t = now as f32 / 1000.0 / pass_s;
        let pass = libm::floorf(t) as u32;
        let along = t - pass as f32;
        let tri = if pass.is_multiple_of(2) { along } else { 1.0 - along };
        let shift = if pass % 2 == 1 { spacing / 2.0 } else { 0.0 };
        let mut n = 0;
        // Innermost first, ray by ray: cut short anywhere, what's left is still connected.
        for k in 1..=per_ray + 1 {
            for r in 0..rays {
                let angle = from + sector * (r as f32 + tri);
                let dir = Vec2::new(libm::cosf(angle), libm::sinf(angle));
                let dist = k as f32 * spacing - shift;
                let p = home + dir * dist;
                if dist < 1.0 || !inside(p) || n == count {
                    continue;
                }
                nodes[n] = Some(p);
                n += 1;
            }
        }
        nodes
    }

    /// Coverage search: keep heading for the current goal cell until it's been searched (by us
    /// or anyone else) or a free neighbor is closer to it; then pick the cell with the best
    /// staleness-minus-travel score among those no free neighbor is closer to.
    pub(crate) fn cover(&mut self, pos: Vec2, now: u32) -> Vec2 {
        let relay = self.relay_duty(pos, now);
        self.relaying = relay.is_some();
        if let Some(relay) = relay {
            return relay;
        }
        // The formation needs nothing else; the radar's rays hang off the base, with the relay
        // chains.
        let spot = match self.cfg.sweep_pattern {
            SweepPattern::Formation => self.formation_spot(now),
            SweepPattern::Radar if self.cfg.relay_chains => match self.spot_for(pos, now) {
                Some(Spot::Sweep(node, moving)) => Some((node, moving)),
                _ => None,
            },
            SweepPattern::Radar | SweepPattern::Cells => None,
        };
        if let Some((slot, moving)) = spot {
            self.on_sweep = true;
            return self.follow_node(slot, moving, pos);
        }
        if let Some(goal) = self.search_goal {
            let center = self.cell_center(goal);
            let fresh = age_at(now, self.searched_at[goal.0][goal.1]) < 2_000;
            if fresh || !self.within_leash(center, self.cfg.scan_radius, now) || self.claimed_by_closer_neighbor(center, pos, now) {
                self.search_goal = None;
            }
        }
        if self.search_goal.is_none() {
            let mut best: Option<((usize, usize), f32)> = None;
            for x in 0..GRID {
                for y in 0..GRID {
                    let center = self.cell_center((x, y));
                    if !self.within_leash(center, self.cfg.scan_radius, now) || self.claimed_by_closer_neighbor(center, pos, now) {
                        continue;
                    }
                    let staleness_s = age_at(now, self.searched_at[x][y]) as f32 / 1000.0;
                    let score = staleness_s - self.cfg.travel_cost_s_per_m * (center - pos).length();
                    if best.is_none_or(|(_, b)| score > b) {
                        best = Some(((x, y), score));
                    }
                }
            }
            self.search_goal = best.map(|(cell, _)| cell);
        }
        // Everything claimed by someone closer (we're boxed in): keep moving anyway.
        let Some(goal) = self.search_goal else { return self.wander(pos) };
        let offset = self.cell_center(goal) - pos;
        let distance = offset.length();
        if distance < 1e-3 {
            return Vec2::ZERO;
        }
        offset * (self.cfg.max_speed / distance)
    }

    /// Where to go to regain contact, or `None` if we're not isolated. That's the last position
    /// any neighbor reported, carried forward along its reported velocity for a few seconds —
    /// converging drones are heading toward the target, so "where they were going" beats
    /// "where they were". A drone that has never heard anyone heads for the field center.
    pub(crate) fn rally_point(&self, now: u32) -> Option<Vec2> {
        if !self.isolated(now) {
            return None;
        }
        let timeout = self.cfg.isolation_timeout_ms;
        let rally = match self.last_contact {
            // Cap the extrapolation: past a few seconds the neighbor has likely stopped or
            // turned, and projecting further just sends us off the field.
            Some((pos, vel, heard_at_ms)) => {
                pos + vel * (now.saturating_sub(heard_at_ms).min(timeout) as f32 / 1000.0)
            }
            None => Vec2::ZERO,
        };
        let h = self.cfg.field_half;
        Some(Vec2::new(rally.x.clamp(-h.x, h.x), rally.y.clamp(-h.y, h.y)))
    }

    pub(crate) fn wander(&mut self, pos: Vec2) -> Vec2 {
        if self.rng.next_f32() < self.cfg.wander_turn_chance {
            self.wander_heading = self.rng.random_unit_vector();
        }
        let h = self.cfg.field_half;
        if (pos.x > h.x && self.wander_heading.x > 0.0) || (pos.x < -h.x && self.wander_heading.x < 0.0)
        {
            self.wander_heading.x = -self.wander_heading.x;
        }
        if (pos.y > h.y && self.wander_heading.y > 0.0) || (pos.y < -h.y && self.wander_heading.y < 0.0)
        {
            self.wander_heading.y = -self.wander_heading.y;
        }
        self.wander_heading * self.cfg.max_speed
    }
}
