//! Relay chains back to the base: routes, the leash, relay spots.

use super::*;

/// What a drone gets when the relay and sweep spots are shared out (`Brain::spot_for`).
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum Spot {
    /// A relay spot on a chain out to a ring (`Brain::relay_spots`).
    Relay(Vec2),
    /// A node of the radar sweep (`Brain::sweep_nodes`), and how fast it's moving.
    Sweep(Vec2, Vec2),
}

/// A drone within this distance of a relay spot holds it, over a drone up to this much nearer
/// to it from elsewhere (m).
pub(crate) const RELAY_SPOT_STICKY: f32 = 1.5;

/// How much nearer the base (m) a neighbor must be to anchor us without fresher news of it.
pub(crate) const NEARER_BASE_MARGIN: f32 = 0.5;

/// How much farther (m) a drone on a ring counts as from a relay spot when relay spots are
/// shared out: farther than any free drone can be, so free drones (the formation's) take them
/// all first, and rings give up drones only when there are none left.
pub(crate) const RING_DONOR_PENALTY: f32 = 1_000.0;

/// How much closer than `link_range` (m) relays in a chain are spaced: each holds its spot by
/// its own GPS, and the drones' GPS errors differ by up to a few meters, which a chain spaced at
/// the radio's range wouldn't survive.
pub(crate) const RELAY_MARGIN_M: f32 = 2.0;

/// Relay spots worked out at most (see `relay_spots`).
pub(crate) const MAX_RELAY_SPOTS: usize = 8;

/// A drone wanting to move slower than this (m/s) isn't held back by its leash, whatever it
/// takes off.
pub(crate) const STRETCHED_MIN_SPEED: f32 = 0.1;

/// News of the ground station older than this means we've lost it.
/// With heartbeats at the beacon rate, news ages about a quarter to a third of a second per
/// hop, and a relay chain out to the far corner of a 30 m field is 7 hops. As much as the
/// announcement can carry (`ANNOUNCE_NO_LINK`: tenths of a second, up to 3.0).
pub(crate) const LINK_TIMEOUT_MS: u32 = 3_000;

impl Brain {
    /// Link age if we went through neighbor `n` (its age when it announced, plus the time
    /// since, plus the hop), unless its route goes through us ("split horizon").
    pub(crate) fn link_age_via(&self, n: &Neighbor, now: u32) -> Option<u32> {
        let age = n.link_age_ms?;
        if n.route_parent == self.sysid {
            return None;
        }
        let age = age.saturating_add(now.saturating_sub(n.heartbeat_at_ms)).saturating_add(self.cfg.hop_latency_ms);
        (age <= LINK_TIMEOUT_MS).then_some(age)
    }

    /// Our route to the ground station: how fresh the freshest news of it is that we have,
    /// heard ourselves or from a neighbor — like a sighting's age, it only grows as it's passed
    /// on and as time passes. That's what keeps routing loops from lasting: hop counts can
    /// circle around a loop forever ("count to infinity"), but news circling a loop only gets
    /// older until it's too old to count.
    pub(crate) fn update_route(&mut self, now: u32) {
        let direct = self.gcs_heard_at.map(|at| now.saturating_sub(at)).filter(|&age| age <= LINK_TIMEOUT_MS);
        let mut best = direct.map(|age| (age, 0u8));
        for (sysid, n) in self.neighbors.iter().enumerate() {
            let Some(n) = n else { continue };
            if let Some(age) = self.link_age_via(n, now) {
                if best.is_none_or(|(b, _)| age < b) {
                    best = Some((age, sysid as u8));
                }
            }
        }
        self.route = best;
        if best.is_some() {
            self.route_at = Some(now);
        }
    }

    /// The spots that keep us in contact: the base if we hear the ground station, and every
    /// neighbor whose news of the ground station is fresher than ours. Staying within
    /// `link_range` of any one of them is enough — which is what lets the swarm reach deep
    /// into the field: a drone can move from near one connected neighbor to near another one
    /// further out. "Fresher than ours" makes the anchors point toward the base: two drones
    /// can never be each other's anchor and drift off together.
    pub(crate) fn anchors(&self, now: u32) -> impl Iterator<Item = Vec2> + '_ {
        self.anchors_from(self.last_pos, now)
    }

    /// `anchors`, for a drone at `from`: with relay chains, which neighbors count depends on
    /// where it is (see below) — and to judge a spot to go to, it's where it would be.
    pub(crate) fn anchors_from(&self, from: Vec2, now: u32) -> impl Iterator<Item = Vec2> + '_ {
        let hears_base = self.gcs_heard_at.is_some_and(|at| now.saturating_sub(at) <= LINK_TIMEOUT_MS);
        let base = hears_base.then_some(self.cfg.home);
        let mine = self.route.map_or(0, |(age, _)| age);
        let neighbors = self.neighbors.iter().flatten().filter_map(move |n| {
            let (pos, vel, at) = n.position?;
            let theirs = self.link_age_via(n, now)?.saturating_sub(self.cfg.hop_latency_ms);
            // With relay chains, also any neighbor in contact that's nearer the base, however
            // fresh its news. Drones that hear the base themselves are all equally fresh, so
            // with strictly fresher only, none could anchor another farther out, and neither the
            // search nor a chain of relays could spread past the base's reach. Nearer the base
            // keeps it one way — two drones can't anchor each other and drift off together — and
            // every chain of anchors ends at the base.
            //
            // Not for a drone on a target, though: drones on rings all move with their mice, and
            // could take turns anchoring each other and walk off together out of contact. A
            // responder stays within reach of drones with fresher news only, a chain that really
            // leads to the base.
            let nearer_base = (pos - self.cfg.home).length() + NEARER_BASE_MARGIN < (from - self.cfg.home).length();
            if theirs >= mine && !(self.cfg.relay_chains && nearer_base && self.assigned.is_none()) {
                return None;
            }
            // Only a recent position: anchoring to where a neighbor was two seconds ago is how
            // a drone drifts out of range while still believing it's fine. Recent for a crowd,
            // where beacons are fewer, is a bit longer: two of them.
            let recent = (2 * self.beacon_now_ms).max(1_000);
            (now.saturating_sub(at) <= recent).then(|| pos + vel * (now.saturating_sub(at) as f32 / 1000.0))
        });
        base.into_iter().chain(neighbors)
    }

    /// Whether the leash applies right now: keeping the link, connected, no operator order
    /// (the operator may fly a drone wherever they like), and not chasing a target out of reach.
    /// Without relay chains a target beats the link: a responder stays on the leash only while
    /// its target is within reach of its anchors — on a leash it couldn't follow a target
    /// running deeper into the field, and would let it go. The free drones — searchers, and the
    /// relays between the base and the responders — always keep the link. With relay chains,
    /// responders keep it too (see `BrainConfig::relay_chains`).
    pub(crate) fn leashed(&self) -> bool {
        // The formation keeps itself together (its drones are in radio range of each other) as it
        // sweeps the field, out of the base's reach and back.
        self.cfg.keep_link
            && self.order.is_none()
            && self.route.is_some()
            && !(self.on_sweep && self.cfg.sweep_pattern == SweepPattern::Formation)
            && (self.assigned.is_none() || self.target_in_reach || self.cfg.relay_chains)
    }

    /// Part of a relay chain out to a target: relaying, or a responder its leash holds back.
    pub(crate) fn in_chain(&self) -> bool {
        // On the radar sweep too: its rays reach out from the base, and the drones in them are
        // links like the relays (the formation keeps itself together instead).
        self.relaying || (self.on_sweep && self.cfg.sweep_pattern == SweepPattern::Radar) || (self.assigned.is_some() && self.stretched)
    }

    /// Lost our route to the base (after having one), recently enough to still go looking for
    /// it: then the only job is to get back in contact.
    pub(crate) fn link_lost(&self, now: u32) -> bool {
        // With relay chains, a free drone out of contact is in the formation, which sweeps out
        // of the base's reach and back on its own: it doesn't head home.
        self.cfg.keep_link
            && !(self.cfg.relay_chains && self.cfg.sweep_pattern == SweepPattern::Formation)
            && self.route.is_none()
            // A responder stays on its target; it comes back once it lets the target go.
            && self.assigned.is_none()
            && self.route_at.is_some_and(|at| now.saturating_sub(at) <= self.cfg.link_lost_timeout_ms)
    }

    /// Within `link_range` (plus `slack`) of some anchor: what the leash allows us to aim for.
    pub(crate) fn within_leash(&self, spot: Vec2, slack: f32, now: u32) -> bool {
        !self.leashed() || self.anchors_from(spot, now).any(|a| (spot - a).length() <= self.cfg.link_range + slack)
    }

    /// The leash: whatever the drone wants to do, it doesn't fly out of `link_range` of all its
    /// anchors. If where it's heading (half a second on) stays within reach of one, fine;
    /// otherwise moving away from the nearest anchor is slowed over the last couple of meters
    /// and stopped at the edge, and past it (the anchor moved off) it's pulled back in.
    pub(crate) fn keep_on_leash(&self, pos: Vec2, velocity: Vec2, now: u32) -> Vec2 {
        if !self.leashed() {
            return velocity;
        }
        let ahead = pos + velocity * 0.5;
        if self.anchors(now).any(|a| (ahead - a).length() <= self.cfg.link_range - 0.5) {
            return velocity;
        }
        let Some(anchor) = self.anchors(now).min_by(|a, b| (pos - *a).length().total_cmp(&(pos - *b).length())) else {
            // A route, but nobody to stay near (its neighbors' positions are stale): head for
            // the base, the one anchor that doesn't move.
            let offset = self.cfg.home - pos;
            let d = offset.length();
            return if d < 1e-3 { Vec2::ZERO } else { offset * (self.cfg.max_speed / d) };
        };
        let offset = pos - anchor;
        let distance = offset.length();
        if distance < 1e-3 {
            return velocity;
        }
        let out = offset * (1.0 / distance);
        let outward = velocity.x * out.x + velocity.y * out.y;
        let slack = self.cfg.link_range - distance;
        let margin = 2.0;
        let allowed = (slack / margin).clamp(0.0, 1.0) * self.cfg.max_speed;
        let mut v = velocity;
        if outward > allowed {
            v = v - out * (outward - allowed);
        }
        if slack < 0.0 {
            v = v - out * (-slack).min(1.0) * self.cfg.max_speed;
        }
        v
    }

    /// Where relays are needed: for every target drones are on that's beyond radio reach of the
    /// base, spots evenly spaced along the line from the base out to its ring, each within
    /// `link_range - 1` of the next. Free drones filling them chain the ring back to the base,
    /// so news of it reaches the rest of the swarm (and the ground station) — and news of
    /// others reaches it. Worked out from where the targets are, which every drone in contact
    /// hears anyway, so every drone works out the same spots, reaching all the way out.
    ///
    /// Only as many chains as there are drones to fill completely: half a chain each to two
    /// rings connects neither.
    pub(crate) fn relay_spots(&self, now: u32) -> [Option<Vec2>; MAX_RELAY_SPOTS] {
        // The rings: where each target drones are on is (or was last heard of, if lost: a ring
        // cut off holds it there or thereabouts, and the relays are what reconnects it).
        let mut rings = [None; MAX_TRACKS];
        let mut ids = [0 as TargetId; MAX_TRACKS];
        let mut n_rings = 0;
        for t in self.tracks.iter().flatten() {
            if t.cancelled || age_at(now, t.observed_at_ms) > self.cfg.last_seen_memory_ms || ids[..n_rings].contains(&t.id) || self.drones_on(t, now) == 0 {
                continue;
            }
            ids[n_rings] = t.id;
            rings[n_rings] = Some(if self.lost(t, now) { t.pos } else { self.predicted(t, now) });
            n_rings += 1;
        }
        // A tree reaching every ring, grown like Prim's: what's connected so far is the base,
        // the relay spots placed and the rings reached; the ring that takes the fewest relays to
        // reach from any of those comes next, by a chain from the nearest. Rings near each other
        // share the way out, rather than each getting its own chain from the base.
        let spacing = self.cfg.link_range - RELAY_MARGIN_M;
        let mut connected = [Vec2::ZERO; MAX_RELAY_SPOTS + MAX_TRACKS + 1];
        connected[0] = self.cfg.home;
        let mut n_connected = 1;
        let mut budget = self.free_drones_known(now) + self.ring_donors(now).iter().flatten().map(|&(_, n)| n as usize).sum::<usize>();
        let mut spots = [None; MAX_RELAY_SPOTS];
        let mut n_spots = 0;
        loop {
            // (ring, from, relays needed, how far out to its near side)
            let mut best: Option<(usize, Vec2, usize, f32)> = None;
            for (r, ring) in rings[..n_rings].iter().enumerate() {
                let Some(ring) = *ring else { continue };
                for &from in &connected[..n_connected] {
                    let out = (ring - from).length() - self.cfg.standoff_radius;
                    let need = if out <= self.cfg.link_range { 0 } else { (libm::ceilf(out / spacing) as usize).saturating_sub(1) };
                    if best.is_none_or(|(_, _, n, o)| need < n || (need == n && out < o)) {
                        best = Some((r, from, need, out));
                    }
                }
            }
            let Some((r, from, need, out)) = best else { break };
            // Not enough drones to complete even the cheapest: half a chain connects nothing.
            if need > budget || n_spots + need > MAX_RELAY_SPOTS {
                break;
            }
            budget -= need;
            let ring = rings[r].take().expect("chosen above");
            let dir = (ring - from) * (1.0 / (ring - from).length().max(1e-3));
            let hops = need + 1;
            for k in 1..=need {
                let spot = from + dir * (out * k as f32 / hops as f32);
                spots[n_spots] = Some(spot);
                n_spots += 1;
                connected[n_connected] = spot;
                n_connected += 1;
            }
            connected[n_connected] = ring;
            n_connected += 1;
        }
        spots
    }

    /// Drones each ring can give up to relay chains when there aren't free drones enough: all
    /// but `RING_MIN` of those on it — counted the cautious way, by the fewer of the count its
    /// sightings report and the drones on it we hear, so a ring whose count lags behind doesn't
    /// give up its last drones and lose its mouse. Per target: (its ID, how many).
    pub(crate) fn ring_donors(&self, now: u32) -> [Option<(TargetId, u8)>; MAX_TRACKS] {
        let mut out = [None; MAX_TRACKS];
        let mut n = 0;
        for t in self.tracks.iter().flatten().filter(|t| !t.cancelled && !self.lost(t, now)) {
            if out[..n].iter().flatten().any(|&(id, _)| id == t.id) {
                continue;
            }
            let heard = self.responders_heard(t.id, now) + (self.assigned == Some(t.id)) as u8;
            out[n] = Some((t.id, t.responders.min(heard).saturating_sub(RING_MIN)));
            n += 1;
        }
        out
    }

    /// Free drones (on no target, no operator order) we know of: this one if it's free, the
    /// ones we hear, and the ones whose status says so lately.
    pub(crate) fn free_drones_known(&self, now: u32) -> usize {
        let me = (self.order.is_none() && self.assigned.is_none()) as usize;
        me + (1..MAX_SYSIDS)
            .filter(|&k| k != self.sysid as usize)
            .filter(|&k| {
                let by_radio = self.neighbors[k].as_ref().filter(|n| now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms);
                match by_radio {
                    Some(n) => n.assigned.is_none(),
                    None => self.swarm_heard[k].is_some_and(|(at, free, _)| free && now.saturating_sub(at) <= FREE_DRONE_FRESH_MS),
                }
            })
            .count()
    }

    /// Relaying: the free drones share out the relay spots (see `relay_spots`) — each drone
    /// works out the same sharing from where it and the free drones it hears are: the nearest
    /// drone and spot pair first, then the nearest of the rest, and so on — and flies to the
    /// one it gets. Its leash still holds it: an outer spot is only reached once the inner ones
    /// are filled, which is the order the chain needs. None if it gets no spot. One sharing,
    /// the same in every drone's view, keeps relays from swapping spots back and forth.
    pub(crate) fn relay_duty(&self, pos: Vec2, now: u32) -> Option<Vec2> {
        if !self.cfg.relay_chains {
            return self.relay_for_a_responder(pos, now);
        }
        let Some(Spot::Relay(spot)) = self.spot_for(pos, now) else { return None };
        let offset = spot - pos;
        let distance = offset.length();
        Some(if distance < 0.3 { Vec2::ZERO } else { offset * (self.cfg.max_speed * (distance / 2.0).min(1.0) / distance) })
    }

    /// Without relay chains: a free drone that hears a responder out of contact, and is the
    /// nearest free drone to it, flies toward it (its own leash stops it at the edge of reach,
    /// which is the right spot for a relay); once the responder's route goes through us, we
    /// stay within its radio range for as long as it depends on us.
    pub(crate) fn relay_for_a_responder(&self, pos: Vec2, now: u32) -> Option<Vec2> {
        if !self.cfg.keep_link || self.route.is_none() {
            return None;
        }
        let fresh = |at: u32| now.saturating_sub(at) <= self.cfg.neighbor_timeout_ms;
        let mut target: Option<Vec2> = None;
        for n in self.neighbors.iter().flatten() {
            let Some((p, v, at)) = n.position.filter(|&(_, _, at)| fresh(at)) else { continue };
            let responding = n.assigned.is_some_and(|a| a as u32 != ANNOUNCE_OPERATOR);
            if !responding || !fresh(n.heartbeat_at_ms) {
                continue;
            }
            let theirs = p + v * (now.saturating_sub(at) as f32 / 1000.0);
            let depends_on_us = n.link_age_ms.is_some() && n.route_parent == self.sysid;
            let stranded = n.link_age_ms.is_none() && !self.claimed_by_closer_neighbor(theirs, pos, now);
            if depends_on_us || stranded {
                // The nearest one, if several.
                if target.is_none_or(|t| (theirs - pos).length() < (t - pos).length()) {
                    target = Some(theirs);
                }
            }
        }
        let t = target?;
        let offset = t - pos;
        let distance = offset.length();
        // Close enough to relay with some margin; no need to crowd it.
        let keep = self.cfg.link_range - 1.0;
        Some(if distance <= keep { Vec2::ZERO } else { offset * (self.cfg.max_speed * ((distance - keep) / 2.0).min(1.0) / distance) })
    }

    /// The relay or sweep spot this drone gets when the spots are shared out (see `Spot`), if
    /// any.
    pub(crate) fn spot_for(&self, pos: Vec2, now: u32) -> Option<Spot> {
        if !self.cfg.keep_link || self.route.is_none() || self.order.is_some() {
            return None;
        }
        let relays = self.relay_spots(now);
        if !self.cfg.relay_chains && relays.iter().all(Option::is_none) {
            return None;
        }
        // Us first, then the drones we hear (where they should be by now): free ones, and those
        // on a ring, which can be given up to a chain (see `ring_donors`) — but only after the
        // free ones: they count as `RING_DONOR_PENALTY` farther away — then the free drones we
        // only know of by their status, from across the swarm. (position, its ring)
        let mut drones: [Option<(Vec2, Option<TargetId>)>; MAX_SYSIDS] = [None; MAX_SYSIDS];
        let mut ids = [0u8; MAX_SYSIDS];
        drones[0] = Some((pos, self.assigned));
        ids[0] = self.sysid;
        let mut n_drones = 1;
        for (k, n) in self.neighbors.iter().enumerate() {
            let Some(n) = n else { continue };
            let Some((p, v, at)) = n.position else { continue };
            if k == self.sysid as usize || n.assigned == Some(ANNOUNCE_OPERATOR as TargetId) || now.saturating_sub(at) > self.cfg.neighbor_timeout_ms || n_drones == MAX_SYSIDS {
                continue;
            }
            drones[n_drones] = Some((p + v * (now.saturating_sub(at) as f32 / 1000.0), n.assigned));
            ids[n_drones] = k as u8;
            n_drones += 1;
        }
        if self.cfg.relay_chains {
            for k in (1..MAX_SYSIDS).filter(|&k| k != self.sysid as usize) {
                let heard = self.neighbors[k].as_ref().is_some_and(|n| n.position.is_some_and(|(_, _, at)| now.saturating_sub(at) <= self.cfg.neighbor_timeout_ms));
                let Some((at, free, p)) = self.swarm_heard[k] else { continue };
                if heard || !free || now.saturating_sub(at) > FREE_DRONE_FRESH_MS || n_drones == MAX_SYSIDS {
                    continue;
                }
                drones[n_drones] = Some((p, None));
                ids[n_drones] = k as u8;
                n_drones += 1;
            }
        }
        let mut drone_free = [true; MAX_SYSIDS];
        // First the relay spots, free drones and drones the rings can spare. A spot another drone
        // in a chain already holds (it's at it, nearer than us) is taken, whatever our sharing
        // says: each drone shares out the spots by what it hears, which over a lossy radio isn't
        // quite the same for all, and two drones going for the same spot left the next one
        // empty — a relay 2 m from another, then a gap of 6.
        let held_by_another = |spot: Vec2| {
            let mine = (spot - pos).length();
            self.neighbors.iter().flatten().any(|n| {
                n.chain
                    && n.position.is_some_and(|(p, v, at)| {
                        let p = p + v * (now.saturating_sub(at) as f32 / 1000.0);
                        now.saturating_sub(at) <= self.cfg.neighbor_timeout_ms && (p - spot).length() < RELAY_SPOT_STICKY && (p - spot).length() + 0.5 < mine
                    })
            })
        };
        let mut open = relays;
        for _ in 0..MAX_RELAY_SPOTS {
            let mut free = drone_free;
            let mut donors = self.ring_donors(now);
            let Some(k) = share(&drones[..n_drones], &mut free, &open, Some(&mut donors)) else { break };
            let spot = open[k].expect("shared out");
            if held_by_another(spot) {
                open[k] = None;
                continue;
            }
            return Some(Spot::Relay(spot));
        }
        if self.cfg.sweep_pattern != SweepPattern::Radar {
            return None;
        }
        // Radar: the drones that got a relay spot don't take sweep nodes.
        let mut donors = self.ring_donors(now);
        share(&drones[..n_drones], &mut drone_free, &relays, Some(&mut donors));
        if self.assigned.is_some() {
            return None;
        }
        // Then the sweep, free drones only: as many of its nodes as there are free drones left.
        if !self.cfg.relay_chains {
            return None;
        }
        // By system ID: the free drone with the lowest takes the innermost node, the next one
        // the next, and so on. The sweep moves all the time: shared out by distance, drones would
        // swap places along the ray every tick, or two about as far out (each knowing the other's
        // position a little late) would take the same node. An ID doesn't change, and every drone
        // knows the same ones.
        let left = drone_free[..n_drones].iter().zip(&drones[..n_drones]).filter(|(f, d)| **f && d.is_some_and(|(_, ring)| ring.is_none())).count();
        let nodes = self.sweep_nodes(left, now);
        let rank = ids[1..n_drones]
            .iter()
            .zip(&drone_free[1..n_drones])
            .zip(&drones[1..n_drones])
            .filter(|((id, free), d)| **free && d.is_some_and(|(_, ring)| ring.is_none()) && **id < self.sysid)
            .count();
        let node = nodes.get(rank).copied().flatten()?;
        // Where it'll be a moment on: the sweep moves, and the drone keeps up with it.
        let later = self.sweep_nodes(left, now.saturating_add((SWEEP_LOOKAHEAD_S * 1000.0) as u32));
        let ahead = later.get(rank).copied().flatten().filter(|a| (*a - node).length() < self.cfg.max_speed * SWEEP_LOOKAHEAD_S).unwrap_or(node);
        Some(Spot::Sweep(node, (ahead - node) * (1.0 / SWEEP_LOOKAHEAD_S)))
    }
}
