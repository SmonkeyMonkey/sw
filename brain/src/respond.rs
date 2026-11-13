//! Who goes to a target: shares, rings, standoff, watching an unattended one.

use super::*;

/// With adaptive responders, how long the share of drones per target must have been lower
/// before the drones on a ring above it leave.
pub(crate) const SHARE_DROP_MS: u32 = 5_000;

/// A drone heard of (by its status) on a target within this long is still on it (it says so
/// as soon as it leaves; relayed statuses take seconds to arrive), for drones out
/// of radio range of its ring working out whether it's full.
pub(crate) const RESPONDER_STATUS_FRESH_MS: u32 = 15_000;

/// How long a drone keeps wanting to leave its ring for a short target before it does, and how
/// long it then stays away from the ring it left.
pub(crate) const DONATE_AFTER_MS: u32 = 2_000;
pub(crate) const LEFT_MEMORY_MS: u32 = 30_000;

/// Shares out `spots` among the drones still free in `drone_free`, as each drone works it out
/// for itself: the nearest drone and spot pair first, then the nearest of the rest, and so on.
/// Free drones (no ring) can take any spot. With `donors`, so can drones on a ring that can
/// spare them (see `Brain::ring_donors`), counted `RING_DONOR_PENALTY` farther away so the free
/// ones go first; without, drones on rings take none. Returns the spot drone 0 (us) gets, if
/// any, and marks the drones that got one.
pub(crate) fn share(
    drones: &[Option<(Vec2, Option<TargetId>)>],
    drone_free: &mut [bool],
    spots: &[Option<Vec2>],
    mut donors: Option<&mut [Option<(TargetId, u8)>]>,
) -> Option<usize> {
    let mut spot_free = [true; MAX_RELAY_SPOTS];
    loop {
        let mut best: Option<(usize, usize, f32)> = None;
        for (d, entry) in drones.iter().enumerate().filter(|(d, _)| drone_free[*d]) {
            let Some((p, ring)) = *entry else { continue };
            let penalty = match (ring, donors.as_deref()) {
                (None, _) => 0.0,
                (Some(id), Some(donors)) if donors.iter().flatten().any(|&(t, n)| t == id && n > 0) => RING_DONOR_PENALTY,
                _ => continue,
            };
            for (k, spot) in spots.iter().enumerate().filter(|(k, _)| spot_free[*k]) {
                let Some(spot) = spot else { continue };
                // A drone already at a spot (holding it) keeps it over one a little nearer to
                // another spot: each drone works this out from slightly different news, and
                // drones would keep swapping spots. The same for every drone, so they all
                // still work out the same sharing.
                let dist = (*spot - p).length();
                let dist = if dist < RELAY_SPOT_STICKY { dist - RELAY_SPOT_STICKY } else { dist } + penalty;
                if best.is_none_or(|(_, _, b)| dist < b) {
                    best = Some((d, k, dist));
                }
            }
        }
        let (d, k, _) = best?;
        if d == 0 {
            return Some(k);
        }
        if let (Some(ring), Some(donors)) = (drones[d].and_then(|(_, ring)| ring), donors.as_deref_mut()) {
            if let Some((_, n)) = donors.iter_mut().flatten().find(|(t, _)| *t == ring) {
                *n -= 1;
            }
        }
        drone_free[d] = false;
        spot_free[k] = false;
    }
}

/// Drones a ring keeps on its target however much a relay chain needs them.
pub(crate) const RING_MIN: u8 = 2;

/// How much farther than sensor range from its target a responder counts as close to it: the
/// ring's drones are all within radio range of it there.
pub(crate) const SETTLE_MARGIN: f32 = 1.0;

/// How long a responder must have been close to its target before it counts as settled: long
/// enough to hear the heartbeats of the drones already on the ring, with some lost.
pub(crate) const SETTLE_MS: u32 = 2_000;

/// A target that had all its responders this recently is treated as still full by newcomers.
pub(crate) const FULL_MEMORY_MS: u32 = 2_000;

impl Brain {
    /// How many drones a target gets: `responders`, or with `adaptive_responders`, its share of
    /// the swarm — the drones it knows of (itself, its neighbors, and by status from across the
    /// swarm lately) over the targets it knows of (confirmed or attended) plus one, at least one
    /// and at most `responders`. The one more is the target not found yet, so some drones are
    /// always left to search for it. A drone joins a target by the share as it is now; one on a
    /// target leaves it by the share once it has been that low for `SHARE_DROP_MS` (the newest on
    /// a ring first).
    pub(crate) fn update_share(&mut self, now: u32) {
        let share = self.current_share(now);
        self.join_share = share;
        if share >= self.stay_share || now.saturating_sub(*self.share_drop_since.get_or_insert(now)) >= SHARE_DROP_MS {
            self.stay_share = share;
            self.share_drop_since = None;
        }
    }

    pub(crate) fn current_share(&self, now: u32) -> u8 {
        let connected = self.cfg.keep_link && self.cfg.relay_chains;
        if !self.cfg.adaptive_responders && !connected {
            return self.cfg.responders;
        }
        let drones = 1 + (1..MAX_SYSIDS)
            .filter(|&k| k != self.sysid as usize)
            .filter(|&k| {
                let by_radio = self.neighbors[k].as_ref().is_some_and(|n| now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms);
                let by_status = self.swarm_heard[k].is_some_and(|(at, _, _)| now.saturating_sub(at) <= SWARM_MEMORY_MS);
                by_radio || by_status
            })
            .count();
        let mut ids = [0 as TargetId; MAX_TRACKS];
        let mut targets = 0;
        for t in self.tracks.iter().flatten() {
            let active = !t.cancelled && !self.lost(t, now) && (self.confirmed(t, now) || t.responders > 0 || self.assigned == Some(t.id));
            if active && !ids[..targets].contains(&t.id) {
                ids[targets] = t.id;
                targets += 1;
            }
        }
        if targets == 0 && !self.cfg.adaptive_responders {
            return self.cfg.responders;
        }
        // Connected, a reserve stays out of the rings to sweep the field and relay: one radar
        // ray's worth, at most half the swarm. Rings stay within the chains' reach of the base, so
        // without a reserve nobody would look over the rest of the field.
        let reserve = if connected { self.cfg.sweep_geometry().per_ray.min(drones / 2) } else { 0 };
        let shares = if self.cfg.adaptive_responders { targets + 1 } else { targets.max(1) };
        ((drones.saturating_sub(reserve) / shares) as u8).clamp(1, self.cfg.responders)
    }

    /// Drones on target `id`, as far as we can tell: the count reported with it, or the
    /// responders we hear (and this one), whichever is more.
    pub(crate) fn drones_on(&self, t: &Track, now: u32) -> u8 {
        t.responders.max(self.responders_heard(t.id, now) + (self.assigned == Some(t.id)) as u8)
    }

    /// Whether some drone not on any target is about (heard as a neighbor, or by its status
    /// from anywhere in the swarm, lately): it'll take a target that's short, so no ring needs
    /// to give one up.
    pub(crate) fn a_free_drone_is_about(&self, now: u32) -> bool {
        (1..MAX_SYSIDS).filter(|&k| k != self.sysid as usize).any(|k| {
            let neighbor = self.neighbors[k].as_ref().is_some_and(|n| now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms && n.assigned.is_none());
            let status = self.swarm_heard[k].is_some_and(|(at, free, _)| free && now.saturating_sub(at) <= FREE_DRONE_FRESH_MS);
            neighbor || status
        })
    }

    /// Division of labor when there aren't drones enough for every target: the newest drone on
    /// a ring leaves it for a target with at least two fewer drones on it, if no free drone is
    /// about to take that one instead and it stays that way for `DONATE_AFTER_MS`. Rings even
    /// out this way (12 drones and 5 mice: 3, 3, 2, 2, 2) and then stop: no one leaves a ring
    /// for one just one drone short of it. Decided from what each drone hears about the targets
    /// and its ring, not from a count of the whole swarm: no drone knows that, and a share worked
    /// out from it would rise and fall all the time, with drones leaving and rejoining rings.
    pub(crate) fn consider_leaving_for_a_short_target(&mut self, now: u32) {
        let Some(mine) = self.assigned else {
            self.leave_since = None;
            return;
        };
        let wants = self.wants_to_leave(mine, now);
        if !wants {
            self.leave_since = None;
            return;
        }
        let since = *self.leave_since.get_or_insert(now);
        if now.saturating_sub(since) >= DONATE_AFTER_MS {
            self.left = Some((mine, now));
            self.assigned = None;
            self.leave_since = None;
            self.urgent_beacon = true;
        }
    }

    pub(crate) fn wants_to_leave(&self, mine: TargetId, now: u32) -> bool {
        if self.order.is_some() || self.a_free_drone_is_about(now) {
            return false;
        }
        let Some(my_track) = self.tracks.iter().flatten().find(|t| t.id == mine) else { return false };
        let on_mine = self.drones_on(my_track, now);
        // Only the newest on the ring: the one that would leave if it had one too many.
        let settled = self.settled(now);
        let behind_me = self.neighbors.iter().enumerate().any(|(sysid, n)| {
            n.as_ref().is_some_and(|n| {
                n.assigned == Some(mine) && now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms && (!n.settled, sysid as u8) > (!settled, self.sysid)
            })
        });
        if behind_me {
            return false;
        }
        let here = self.predicted(my_track, now);
        self.tracks.iter().flatten().any(|t| {
            t.id != mine
                && !t.cancelled
                && age_at(now, t.observed_at_ms) <= SHORT_FRESH_MS
                // Confirmed, or someone's on it: drones far off may have heard of it from a
                // single witness.
                && (self.confirmed(t, now) || t.responders > 0)
                // Not just another name for ours.
                && (t.apart(mine) || my_track.apart(t.id) || (self.predicted(t, now) - here).length() > self.cfg.merge_radius)
                && self.drones_on(t, now).saturating_add(2) <= on_mine
        })
    }

    /// Whether this drone should respond to target `id` (now at `target`), so that about
    /// `responders` drones do. Decided from its own neighbor table — assignments ride on
    /// HEARTBEAT — so there's no negotiation round, and every drone applying the same rule to
    /// the same information reaches the same answer:
    ///  - a drone not yet on it joins if the drones already on it plus the free drones closer to
    ///    it than this one still leave a place;
    ///  - a drone already on it stays unless `responders` others on it rank ahead of it:
    ///    settled drones (see `settled`) before newcomers, then by sysid. Every drone announces
    ///    whether it's settled, so all of them rank the same way, and when a target ends up
    ///    with one too many it's a newcomer that leaves, not a drone holding the ring. Not by
    ///    distance: on the standoff ring everyone is about equally far, and ranking by distance
    ///    there would have drones swap in and out forever.
    pub(crate) fn is_responder(&self, id: TargetId, target: Vec2, reported: u8, me: Vec2, now: u32) -> bool {
        let staying = self.assigned == Some(id);
        // Only watching it till another came: it's here now.
        if staying && self.custody == Some(id) && self.responders_heard(id, now) > 0 {
            return false;
        }
        // Joining goes by the share now; staying, by the share as it's been for a while (see
        // `update_share`), so a share that rises and falls with every target found or lost
        // doesn't have drones leaving rings and coming back.
        let responders = if staying { self.stay_share } else { self.join_share };
        // Left it for a target that was short (see `consider_leaving_for_a_short_target`): not
        // straight back.
        if !staying && self.left.is_some_and(|(left, at)| left == id && now.saturating_sub(at) < LEFT_MEMORY_MS) {
            return false;
        }
        let settled = self.settled(now);
        // A newcomer can't always hear every responder (they may be beyond radio range on the
        // far side of the ring), so it also goes by the count reported along with the target —
        // otherwise it joins, gets close, hears them all, leaves, and does it again forever.
        if !staying && reported >= responders {
            return false;
        }
        // ...and by whether it was full a moment ago. Both counts come from heartbeats, and one
        // lost to the radio makes a full ring look one short for a moment — long enough for a
        // newcomer to grab the "free" place, fly over, find out, and leave. A place that's
        // really free (a responder died or left) stays free, and is taken a couple of seconds on.
        let full_recently = self
            .tracks
            .iter()
            .flatten()
            .find(|t| t.id == id)
            .is_some_and(|t| t.full_at_ms != 0 && now.saturating_sub(t.full_at_ms) < FULL_MEMORY_MS);
        if !staying && full_recently {
            return false;
        }
        if !staying {
            // A drone in the formation leaves a target in another stretch to that stretch's group:
            // flying past one on the way to its own stretch, it may be the nearest drone it can
            // hear (the group there out of its radio range), and would stay on a mouse that isn't
            // its front's. Its home stretch, that is (see `home_stretch`), whatever its front
            // sweeps now. And not at all if it's one of the drones its front keeps searching.
            // Unless it sees a target nobody is on or on the way to: then it watches it until one
            // of the drones it's theirs to comes (see `custody`), rather than leave it unwatched
            // while nobody else knows of it.
            if self.outsider(target, now) {
                return self.unattended(id, now);
            }
            if let Some(ahead) = self.formation_ahead(id, target, now) {
                return ahead < responders;
            }
        }
        let my_distance = (me - target).length();
        let mut ahead = 0u8;
        for (sysid, n) in self.neighbors.iter().enumerate() {
            let Some(n) = n else { continue };
            let Some((pos, vel, heard_at_ms)) = n.position else { continue };
            let age_ms = now.saturating_sub(heard_at_ms);
            if age_ms > self.cfg.neighbor_timeout_ms {
                continue;
            }
            let counts = match n.assigned {
                Some(theirs) if theirs == id => !staying || (!n.settled, sysid as u8) < (!settled, self.sysid),
                // Busy with another target: not coming here.
                Some(_) => false,
                None => !staying && ((pos + vel * (age_ms as f32 / 1000.0)) - target).length() < my_distance,
            };
            if counts {
                ahead = ahead.saturating_add(1);
            }
        }
        // With relay chains, also the drones we only know of by status: the free ones nearer to it,
        // so only the nearest go, not every free drone that hears of a new target; and the ones
        // already on their way to it. The ring doesn't hear those yet, so the count reported with
        // the target leaves them out, and two drones far off could both take the one place left.
        if self.cfg.relay_chains && !staying {
            ahead = ahead.saturating_add(
                (1..MAX_SYSIDS)
                    .filter(|&k| k != self.sysid as usize && self.neighbors[k].as_ref().is_none_or(|n| now.saturating_sub(n.last_heard_ms()) > self.cfg.neighbor_timeout_ms))
                    .filter(|&k| {
                        self.swarm_heard[k].is_some_and(|(at, free, p)| {
                            let fresh = now.saturating_sub(at) <= FREE_DRONE_FRESH_MS;
                            fresh && ((free && (p - target).length() < my_distance) || self.swarm_target[k] == id)
                        })
                    })
                    .count() as u8,
            );
        }
        ahead < responders
    }

    /// Settled in on its target: it has been close to it (close enough to hear every drone on
    /// its ring) for `SETTLE_MS`. Until then it's a newcomer, and gives way to anyone settled —
    /// it may have taken the target on from beyond radio range of the ring (a report travels
    /// farther than the radio, by relay), not knowing the ring was already full.
    pub(crate) fn settled(&self, now: u32) -> bool {
        self.near_target_since_ms.is_some_and(|at| now.saturating_sub(at) >= SETTLE_MS)
    }

    /// Neighbors we hear that say they're on target `id`.
    pub(crate) fn responders_heard(&self, id: TargetId, now: u32) -> u8 {
        self.neighbors
            .iter()
            .flatten()
            .filter(|n| n.assigned == Some(id) && now.saturating_sub(n.heartbeat_at_ms) <= self.cfg.neighbor_timeout_ms)
            .count() as u8
    }

    /// Drones on target `id`, as far as this one knows: the neighbors it hears say so, and the
    /// drones out of its radio range whose relayed status lately did (a drone coming from afar
    /// can't hear the ring yet).
    pub(crate) fn responders_known(&self, id: TargetId, now: u32) -> u8 {
        let out_of_range = (1..MAX_SYSIDS)
            .filter(|&k| k != self.sysid as usize && self.neighbors[k].as_ref().is_none_or(|n| now.saturating_sub(n.heartbeat_at_ms) > self.cfg.neighbor_timeout_ms))
            .filter(|&k| self.swarm_target[k] == id && self.swarm_heard[k].is_some_and(|(at, _, _)| now.saturating_sub(at) <= RESPONDER_STATUS_FRESH_MS))
            .count();
        (self.responders_heard(id, now) as usize + out_of_range).min(u8::MAX as usize) as u8
    }

    /// Holds the standoff ring around `to`: flies in from outside, backs out from inside, and
    /// slows down near the ring so the drone eases into place instead of overshooting it.
    pub(crate) fn toward(&self, id: TargetId, from: Vec2, to: Vec2, now: u32) -> Vec2 {
        let offset = to - from;
        let distance = offset.length();
        if distance < 1e-3 {
            return Vec2::ZERO;
        }
        let r = self.ring_radius(id, from, to, now);
        let speed = self.cfg.max_speed * ((distance - r) / r).clamp(-1.0, 1.0);
        // Hearing none of the target's other drones, round the ring to a side of the target of its
        // own, by system ID, a golden angle on from the one before: drones on a target that can't
        // hear each other (jammed) would otherwise all come at it down the same line, chasing it,
        // and hold the same spot on the ring. Hearing them, the rings and separation share the ring
        // out, and a side of its own would keep rings around mice next to each other from sharing a
        // mouse between them.
        if self.responders_heard(id, now) > 0 {
            return offset * (speed / distance);
        }
        let side = self.sysid as f32 * 2.399_963;
        let at = libm::atan2f(-offset.y, -offset.x);
        let off = libm::remainderf(side - at, core::f32::consts::TAU);
        let around = Vec2::new(offset.y, -offset.x) * (1.0 / distance);
        let near = (2.0 - distance / r).clamp(0.0, 1.0);
        let turn = 0.5 * self.cfg.max_speed * (off / core::f32::consts::FRAC_PI_2).clamp(-1.0, 1.0) * near;
        clamp_length(offset * (speed / distance) + around * turn, self.cfg.max_speed)
    }

    /// Which standoff ring this drone holds. The inner ring is at `standoff_radius`, inside
    /// sensor range, so the drones on it keep seeing the target and keep the sighting confirmed.
    /// It only fits so many drones a bit over a separation radius apart; any more and separation
    /// and the pull toward the ring fight forever. So drones stack outward in rings, each picking
    /// its ring by how many of the target's other responders are already closer to it.
    pub(crate) fn ring_radius(&self, id: TargetId, me: Vec2, target: Vec2, now: u32) -> f32 {
        let spacing = 1.5 * self.cfg.separation_radius;
        let my_distance = (me - target).length();
        let mut closer = 0u32;
        // Only drones on this target hold its rings; a searcher passing by doesn't push them out.
        for n in self.neighbors.iter().flatten().filter(|n| n.assigned == Some(id)) {
            let Some((pos, _, heard_at_ms)) = n.position else { continue };
            if now.saturating_sub(heard_at_ms) <= self.cfg.neighbor_timeout_ms && (pos - target).length() < my_distance {
                closer += 1;
            }
        }
        let mut radius = self.cfg.standoff_radius;
        loop {
            let capacity = ((core::f32::consts::TAU * radius / spacing) as u32).max(1);
            if closer < capacity {
                return radius;
            }
            closer -= capacity;
            radius += spacing;
        }
    }

    /// A drone of a front that leaves `target` to the rest of the swarm: it's in another front's
    /// stretch, or this one is its front's search reserve (see `in_reserve`).
    pub(crate) fn outsider(&self, target: Vec2, now: u32) -> bool {
        if self.slot_of(self.sysid, now).is_none() {
            return false;
        }
        let (index, fronts) = self.home_stretch(self.sysid);
        !self.cfg.front_path(fronts, index, 1).holds(target) || self.in_reserve(now)
    }

    /// Whether target `id` is one this drone sees right now with no drone on it or on the way to
    /// it, as far as it knows.
    pub(crate) fn unattended(&self, id: TargetId, now: u32) -> bool {
        self.tracks.iter().flatten().any(|t| t.id == id && t.seen_now) && self.responders_known(id, now) == 0
    }
}
