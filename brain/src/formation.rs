//! The formation: fronts, segments, who's in them, and each drone's place.

use super::*;

/// Drones the formation's roster holds at most (see `Brain::update_roster`), and so the most
/// `BrainConfig::sweep_for` plans for.
pub(crate) const MAX_ROSTER: usize = 128;

/// How long a change in who's free must last before the formation's roster takes it in: a
/// drone joining, or a drone gone (to a mouse, or lost) — a drone that comes back within it
/// never left the line.
pub(crate) const ROSTER_JOIN_MS: u32 = 1_000;
pub(crate) const ROSTER_LEAVE_MS: u32 = 5_000;

/// A drone not heard of, directly or by status, for this long is taken for gone: its front
/// closes the gap it left.
pub(crate) const GONE_AFTER_MS: u32 = 60_000;
/// How long after the fronts set out the drones launched last are still on their way to theirs
/// (see `Brain::update_front`).
pub(crate) const ASSEMBLE_MS: u32 = 30_000;
/// How often the base launches a drone (as the simulator's does; see `front_start_ms`).
pub(crate) const LAUNCH_INTERVAL_MS: u32 = 1_000;
/// How long before the last of its drones can be at its place a front sets out (see
/// `BrainConfig::front_start_ms`).
pub(crate) const START_SLACK_MS: u32 = 15_000;
/// A segment of a front none of whose drones has been heard free for this long is taken to have
/// none free (see `Brain::slot_of`).
pub(crate) const EMPTY_AFTER_MS: u32 = 20_000;
/// How long a drone hears no other at all before it takes the radio for gone (see
/// `Brain::toward_front`).
pub(crate) const SILENT_AFTER_MS: u32 = 10_000;

/// A front keeps at least this many of its drones searching, whatever targets turn up in its
/// stretch (see `in_reserve`): the drones of a front that all went for mice left its stretch
/// unsearched for good.
pub(crate) const MIN_SWEEPERS: usize = 2;
/// Fronts the field is split into at most (a 600 m field with 160 drones has 26).
pub(crate) const MAX_STRETCHES: usize = 32;

/// Which front of `fronts` the drone ranked `rank` of `n` free ones (by system ID) is in, and
/// that front's drones: from rank `first`, `count` of them. The fronts split the drones in turn,
/// as evenly as they go.
pub(crate) fn front_of(rank: usize, n: usize, fronts: usize) -> (usize, usize, usize) {
    let fronts = fronts.clamp(1, n.max(1));
    let first = |k: usize| k * n / fronts;
    let index = (0..fronts).rev().find(|&k| first(k) <= rank).unwrap_or(0);
    (index, first(index), first(index + 1) - first(index))
}

/// A drone's front as it has it planned (see `Brain::update_front`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct FrontPlan {
    /// Its drones believed free, in order: the first `count`. They have places in it.
    pub(crate) members: [u8; MAX_ROSTER],
    pub(crate) count: usize,
    /// Of those, the ones in the roster (heard free lately): the first `heard_count`.
    pub(crate) heard: [u8; MAX_ROSTER],
    pub(crate) heard_count: usize,
    /// Which front of how many.
    pub(crate) index: usize,
    pub(crate) fronts: usize,
    /// Its places: system IDs `first + 1` to `first + size`, in `segments` segments.
    pub(crate) first: usize,
    pub(crate) size: usize,
    pub(crate) segments: usize,
    /// Its free drones close up into one line (see `Brain::slot_of`).
    pub(crate) closed: bool,
    /// When the fronts set out (mission clock, ms).
    pub(crate) start_ms: u32,
}

/// Which of `parts` place `q` of `places` is in: (which, its first place, how many places). As
/// even as they go.
pub(crate) fn split(q: usize, places: usize, parts: usize) -> (usize, usize, usize) {
    let parts = parts.clamp(1, places.max(1));
    let (size, bigger) = (places / parts, places % parts);
    let j = (0..parts).find(|&j| q < (j + 1) * size + (j + 1).min(bigger)).unwrap_or(parts - 1);
    (j, j * size + j.min(bigger), size + (j < bigger) as usize)
}

/// The way a front flies over its stretch of the field: pass after pass along the field's
/// longer side, back and forth, each in the next lane, from the first lane to the last and back
/// (see `lane_of`), moving over to the next lane over the last stretch of each pass. Where along
/// the stretch the front is goes by the clock alone, however many lanes there are: a front
/// closed up into fewer drones and more lanes is just where it would have been, only in another
/// lane, so a drone leaving or joining it moves the line across the field, never along it. In
/// (u, v): across the passes and along them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct FrontPath {
    /// Across: from `left`, `width` wide, in `lanes` lanes.
    pub(crate) left: f32,
    pub(crate) width: f32,
    pub(crate) lanes: usize,
    /// Along: from `low` to `high`, setting out from `high` if `down`.
    pub(crate) low: f32,
    pub(crate) high: f32,
    pub(crate) down: bool,
    /// u runs the other way (the base is on that side), v is y.
    pub(crate) flip_u: bool,
    pub(crate) along_y: bool,
    /// The front's stretch along the field, for targets: up to the boundaries with the fronts
    /// beside it, and past the field's ends.
    pub(crate) band: (f32, f32),
}

impl FrontPath {
    pub(crate) fn lane_width(&self) -> f32 {
        self.width / self.lanes as f32
    }

    pub(crate) fn length_along(&self) -> f32 {
        (self.high - self.low).max(0.1)
    }

    /// The lane of pass `p`: from the first to the last and back, each lane once each way, so
    /// a lane's end cut short as the front turns into the next one is looked over on its way back.
    pub(crate) fn lane_of(&self, p: u32) -> usize {
        let k = self.lanes as u32;
        let r = p % (2 * k);
        (if r < k { r } else { 2 * k - 1 - r }) as usize
    }

    /// The left edge of the lane the front is in, `q` along its way (u), and how far along the
    /// field it is (v). Over the last stretch of a pass it moves over to the next pass's lane,
    /// as far along as across at most (so on a turn it's flying at least as much along).
    pub(crate) fn at(&self, q: f32) -> (f32, f32) {
        let (len, w) = (self.length_along(), self.lane_width());
        let pass = libm::floorf(q.max(0.0) / len);
        let along = q.max(0.0) - pass * len;
        let p = pass as u32;
        let up = p.is_multiple_of(2) != self.down;
        let v = if up { self.low + along } else { self.high - along };
        let (this, next) = (self.lane_of(p) as f32, self.lane_of(p.wrapping_add(1)) as f32);
        let turn = (1.5 * w).min(len);
        let over = ((along - (len - turn)) / turn).clamp(0.0, 1.0);
        (self.left + w * (this + (next - this) * over), v)
    }

    /// Where a point of the field is, in (u, v) (see `field_point`).
    pub(crate) fn uv(&self, p: Vec2) -> (f32, f32) {
        let (u, v) = if self.along_y { (p.x, p.y) } else { (p.y, p.x) };
        (if self.flip_u { -u } else { u }, v)
    }

    pub(crate) fn field_point(&self, u: f32, v: f32) -> Vec2 {
        let u = if self.flip_u { -u } else { u };
        if self.along_y { Vec2::new(u, v) } else { Vec2::new(v, u) }
    }

    /// Where the drone ranked `rank` of the front's `members` is, `q` along the loop.
    pub(crate) fn slot(&self, q: f32, rank: usize, members: usize) -> Vec2 {
        let (left, v) = self.at(q);
        self.field_point(left + (rank as f32 + 0.5) * self.lane_width() / members.max(1) as f32, v)
    }

    /// The middle of the front, `q` along the loop.
    pub(crate) fn center(&self, q: f32) -> Vec2 {
        let (left, v) = self.at(q);
        self.field_point(left + 0.5 * self.lane_width(), v)
    }

    /// Whether `p` is in the front's stretch of the field.
    pub(crate) fn holds(&self, p: Vec2) -> bool {
        let v = if self.along_y { p.y } else { p.x };
        v >= self.band.0 && v <= self.band.1
    }
}

/// How fast (m/s) the formation sweeps (see `BrainConfig::formation_slot`): well within a
/// drone's top speed, so a drone catching up with its slot can.
pub(crate) const FORMATION_SPEED: f32 = 1.5;

/// How long the planned fleet's formations take to look over the field once (s): a mouse at
/// 0.8 m/s gets about 50 m in that time, so ground looked over doesn't stay clear for long.
pub(crate) const PLANNED_PASS_S: f32 = 60.0;

/// How far ahead (s) a drone following a sweep node looks to keep up with it.
pub(crate) const SWEEP_LOOKAHEAD_S: f32 = 0.5;

impl Brain {
    /// Who's in the formation as of now: every free drone not relaying — this one, the free
    /// neighbors it hears that aren't in a chain, and the free drones it knows of by status from
    /// across the swarm — in order of system ID.
    pub(crate) fn formation_members(&self, now: u32) -> ([u8; MAX_ROSTER], usize) {
        let mut ids = [0u8; MAX_ROSTER];
        let mut n = 0;
        let me_in = self.order.is_none() && self.assigned.is_none() && !self.relaying;
        for k in 1..MAX_SYSIDS {
            let member = if k == self.sysid as usize {
                me_in
            } else {
                let heard = self.neighbors[k].as_ref().filter(|nb| now.saturating_sub(nb.last_heard_ms()) <= self.cfg.neighbor_timeout_ms);
                match heard {
                    Some(nb) => nb.assigned.is_none() && !nb.chain,
                    None => self.swarm_heard[k].is_some_and(|(at, free, _)| free && now.saturating_sub(at) <= FREE_DRONE_FRESH_MS),
                }
            };
            if member && n < MAX_ROSTER {
                ids[n] = k as u8;
                n += 1;
            }
        }
        (ids, n)
    }

    /// Keeps the formation's roster: drones joining are taken on after `ROSTER_JOIN_MS`, drones
    /// gone (to a mouse, or lost) are dropped only after `ROSTER_LEAVE_MS`. Meanwhile everyone
    /// keeps their slot and the formation holds its line, with a gap where a drone left, rather
    /// than re-spacing the whole line every time a drone goes for a mouse.
    pub(crate) fn update_roster(&mut self, now: u32) {
        if self.cfg.sweep_pattern != SweepPattern::Formation {
            self.roster_len = 0;
            return;
        }
        let (ids, n) = self.formation_members(now);
        if ids[..n] == self.roster[..self.roster_len] {
            self.roster_differs_since = None;
            return;
        }
        let leaving = self.roster[..self.roster_len].iter().any(|id| !ids[..n].contains(id));
        let wait = if self.roster_len == 0 {
            0
        } else if leaving {
            ROSTER_LEAVE_MS
        } else {
            ROSTER_JOIN_MS
        };
        let since = *self.roster_differs_since.get_or_insert(now);
        if now.saturating_sub(since) >= wait {
            self.roster = ids;
            self.roster_len = n;
            self.roster_differs_since = None;
        }
    }

    /// Consensus on who in the formation goes for target `id` (at `target`): the drones whose
    /// slots are nearest it. Every drone in a front knows the same roster and the same clock, so
    /// works out the same slots and comes to the same answer, where the positions heard over the
    /// radio are late and carry each drone's GPS error. The number of drones ahead of this one:
    /// those on it already, and the free ones in its front whose slots are nearer. None if this
    /// drone isn't in a front.
    pub(crate) fn formation_ahead(&self, id: TargetId, target: Vec2, now: u32) -> Option<u8> {
        let mine = (self.slot_of(self.sysid, now)? - target).length();
        let busy = |m: u8| self.neighbors[m as usize].as_ref().is_some_and(|n| n.assigned.is_some() && now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms);
        let home = self.home_stretch(self.sysid).0;
        let nearer = self
            .front_members()
            .iter()
            .filter(|&&m| m != self.sysid && !busy(m) && self.home_stretch(m).0 == home)
            .filter(|&&m| {
                self.slot_of(m, now).is_some_and(|slot| {
                    let theirs = (slot - target).length();
                    theirs < mine || (theirs == mine && m < self.sysid)
                })
            })
            .count();
        Some((self.responders_known(id, now) as usize + nearer).min(u8::MAX as usize) as u8)
    }

    /// The drones of this drone's front heard free lately, in order (none if it's in no front):
    /// those it goes by in working out who of them takes a target.
    pub(crate) fn front_members(&self) -> &[u8] {
        match &self.front {
            Some(f) => &f.heard[..f.heard_count],
            None => &[],
        }
    }

    /// The drones with places in this drone's front, in order (see `update_front`).
    pub(crate) fn front_places(&self) -> &[u8] {
        match &self.front {
            Some(f) => &f.members[..f.count],
            None => &[],
        }
    }

    /// Where member `member` of this drone's front is at `now`. The front's places go to its
    /// drones by system ID, cut into segments of neighbors (see `BrainConfig::segment_max`); the
    /// free drones of a segment spread evenly over its share of the lane. A front with at most
    /// half as many free drones as a segment holds closes up instead: a line of them a spacing
    /// apart, mowing its stretch in as many lanes as that takes, so they stay in radio range of
    /// each other. None for a drone that isn't in it.
    ///
    /// Where the front is goes by the mission clock alone, the same for every drone: parts of a
    /// front out of radio range of each other, or counting different numbers of drones, still
    /// agree on where its line is.
    pub(crate) fn slot_of(&self, member: u8, now: u32) -> Option<Vec2> {
        let front = self.front.as_ref()?;
        if !self.front_places().contains(&member) {
            return None;
        }
        let segments = if front.closed { 1 } else { front.segments };
        let segment = |m: u8| split(m as usize - 1 - front.first, front.size, segments);
        let (j, _, _) = segment(member);
        let (mut rank, mut count) = (0, 0);
        for &m in self.front_places().iter().filter(|&&m| segment(m).0 == j) {
            rank += (m < member) as usize;
            count += 1;
        }
        let q = now.saturating_sub(front.start_ms) as f32 / 1000.0 * FORMATION_SPEED;
        if front.closed {
            return Some(self.cfg.front_path(front.fronts, front.index, count).slot(q, rank, count));
        }
        let path = self.cfg.front_path(front.fronts, front.index, front.size);
        let (left, v) = path.at(q);
        // In places: how far apart the segment's free drones are, and where the first is. Too few
        // to span the segment's share a spacing apart at most, they keep a spacing apart, so they
        // stay in radio range of each other. The segments' drones line up from the middle of the
        // front out, each segment's up against the next one's toward the middle, so the front stays
        // one line however few are free in any of them. For the segments between it and the middle,
        // it goes by how many it heard free lately: drones heard of long ago can be gone for mice.
        let place = path.lane_width() / front.size as f32;
        let most_gap = self.cfg.formation_spacing() / place;
        // Segment k's first place, and how many places it has.
        let share = |k: usize| {
            let first_k = (0..front.size).find(|&q| split(q, front.size, segments).0 == k).unwrap_or(0);
            let (_, first_k, size_k) = split(first_k, front.size, segments);
            (first_k as f32, size_k as f32)
        };
        // How many drones each other segment has free. A segment beside its own goes by whom it
        // heard free lately: drones heard of long ago can be gone for mice, and would leave holes
        // in the line. Segments farther off, beyond the reach of its statuses, go by whom it takes
        // to be free, since nothing is heard of them lately either way. On a long front (over three
        // status ranges), a segment beside its own counts all it takes to be free as long as it
        // heard any of them free lately: one drone not heard a while would otherwise make it look
        // narrower than it is, and the two segments' places would overlap.
        let reach = self.cfg.status_max_hops as f32 * self.cfg.link_range / place;
        let long = front.size as f32 > 3.0 * reach;
        let free = |k: usize| {
            let places = self.front_places().iter().filter(|&&m| segment(m).0 == k);
            if k == j {
                count
            } else if long {
                // None if none of it heard free lately (gone for mice), else as many as it takes.
                if places.clone().any(|&m| self.heard_free_lately(m, now)) || k.abs_diff(j) > 1 { places.count() } else { 0 }
            } else {
                places.filter(|&&m| k.abs_diff(j) > 1 || self.heard_free_lately(m, now)).count()
            }
        };
        let gap_of = |k: usize| if free(k) == 0 { 0.0 } else { (share(k).1 / free(k) as f32).min(most_gap) };
        let block = |k: usize| gap_of(k) * free(k) as f32;
        // The middle segment's middle, or the boundary between the two middle ones: all of them
        // with drones enough, each spans its own share.
        let half = segments / 2;
        let middle = if segments % 2 == 1 { share(half).0 + share(half).1 / 2.0 } else { share(half).0 };
        let start = if segments % 2 == 1 && j == half {
            middle - block(j) / 2.0
        } else if j < half {
            // Left of the middle: its right edge where the next one's block starts.
            let mut edge = if segments % 2 == 1 { middle - block(half) / 2.0 } else { middle };
            for k in j + 1..half {
                edge -= block(k);
            }
            edge - block(j)
        } else {
            let mut edge = if segments % 2 == 1 { middle + block(half) / 2.0 } else { middle };
            for k in half + segments % 2..j {
                edge += block(k);
            }
            edge
        };
        let gap = gap_of(j);
        let u = left + place * (start + (rank as f32 + 0.5) * gap);
        Some(path.field_point(u, v))
    }

    /// How many drones have been launched: the mission's fleet, or more if drones were launched
    /// since (system IDs go up with every launch).
    pub(crate) fn launched(&self) -> usize {
        (self.cfg.fleet as usize).max(self.highest_sysid as usize).clamp(1, BASE_SYSID as usize - 1)
    }

    /// Drone `m`'s home stretch: the one its front sweeps (see `update_front`), and of how many.
    /// A drone takes on targets only in its own.
    pub(crate) fn home_stretch(&self, m: u8) -> (usize, usize) {
        let launched = self.launched();
        let fronts = self.cfg.sweep_layout(launched).rows.clamp(1, MAX_STRETCHES);
        (front_of((m as usize).saturating_sub(1).min(launched - 1), launched, fronts).0, fronts)
    }

    /// Whether drone `m` was heard free, directly or by status, within `EMPTY_AFTER_MS`.
    pub(crate) fn heard_free_lately(&self, m: u8, now: u32) -> bool {
        let fresh = |at: u32| now.saturating_sub(at) <= EMPTY_AFTER_MS;
        let direct = self.neighbors[m as usize].as_ref().is_some_and(|n| n.assigned.is_none() && !n.chain && fresh(n.last_heard_ms()));
        direct || self.swarm_heard[m as usize].is_some_and(|(at, free, _)| free && fresh(at))
    }

    /// Whether drone `m` is free, as far as this drone last heard, or was never heard of at all
    /// (launched, not yet heard). One it should be hearing (`near`) not heard of for
    /// `GONE_AFTER_MS` is gone. Going by what it last heard, not only by whom it heard lately, a
    /// drone out of range of its front (back from a mouse, say) doesn't take itself for the whole
    /// front and fly to a place another drone holds.
    pub(crate) fn believed_free(&self, m: u8, near: bool, now: u32) -> bool {
        if self.is_revoked(m) {
            return false;
        }
        if let Some(n) = self.neighbors[m as usize].as_ref().filter(|n| now.saturating_sub(n.last_heard_ms()) <= self.cfg.neighbor_timeout_ms) {
            return n.assigned.is_none() && !n.chain;
        }
        match self.swarm_heard[m as usize] {
            Some((at, free, _)) => free && (!near || now.saturating_sub(at) <= GONE_AFTER_MS),
            None => true,
        }
    }

    /// Plans this drone's front. The drones launched are split into as many fronts as there are
    /// drones for fronts as wide as the field (one if there aren't), in turn by system ID, each
    /// with its own stretch of the field, for good: what front a drone is in doesn't hang on who
    /// it hears. The front's members are those of its drones believed free (see `believed_free`).
    pub(crate) fn update_front(&mut self, now: u32) {
        let formation = self.cfg.sweep_pattern == SweepPattern::Formation && self.cfg.search == SearchStrategy::Coverage;
        let in_roster = self.roster[..self.roster_len].contains(&self.sysid);
        if !formation || !in_roster {
            self.front = None;
            return;
        }
        let launched = self.launched();
        let fronts = self.cfg.sweep_layout(launched).rows.clamp(1, MAX_STRETCHES);
        let (index, first, size) = front_of((self.sysid as usize - 1).min(launched - 1), launched, fronts);
        let full = self.cfg.front_path(fronts, index, size);
        let most = self.cfg.segment_max(full.lane_width() / size as f32);
        let segments = size.div_ceil(most).max(1);
        // The drones of its own segment are within status range: the rest of a long front may
        // never be heard of.
        let segment = |m: u8| split(m as usize - 1 - first, size, segments).0;
        let rostered = |m: u8| self.roster[..self.roster_len].contains(&m);
        let mine = segment(self.sysid);
        let start_ms = self.cfg.front_start_ms(launched, index);
        // Hearing others of its own segment free, it leaves out the drones of the segment gone
        // quiet, as they do: what it last heard of one out of earshot can be what they never heard,
        // and drones side by side would work out different lines. Not while the fronts are still
        // gathering: drones launched a few at a time fly out out of range of each other, and each
        // group, going by whom it heard, would take itself for the whole front.
        let gathered = now > start_ms.saturating_add(ASSEMBLE_MS);
        let hearing = gathered && (first + 1..=first + size).map(|m| m as u8).any(|m| m != self.sysid && segment(m) == mine && rostered(m));
        let (mut members, mut count, mut heard, mut heard_count) = ([0u8; MAX_ROSTER], 0, [0u8; MAX_ROSTER], 0);
        for m in (first + 1..=first + size).map(|m| m as u8) {
            // The roster's drones too: one just gone for a mouse keeps its place a while.
            let in_roster = rostered(m);
            let near = segment(m) == mine;
            if in_roster || (!(near && hearing) && self.believed_free(m, near, now)) {
                members[count] = m;
                count += 1;
            }
            if in_roster {
                heard[heard_count] = m;
                heard_count += 1;
            }
        }
        // At most half as many free as a segment holds: every one of them hears of every other,
        // so they all close up alike. (In a longer line even a drone at its end hears of `most`;
        // the margin is for statuses lost on the way.)
        let closed = segments == 1 || 2 * count <= most;
        self.front = Some(FrontPlan { members, count, heard, heard_count, index, fronts, first, size, segments, closed, start_ms });
    }

    /// One of its front's search reserve: of the front's drones, the `MIN_SWEEPERS` with the
    /// highest system IDs. They leave targets to the rest of the front, so it never stops
    /// searching its stretch. Everyone in the front knows the same roster, so agrees on who they
    /// are.
    pub(crate) fn in_reserve(&self, now: u32) -> bool {
        if self.slot_of(self.sysid, now).is_none() {
            return false;
        }
        self.front_members().iter().filter(|&&m| m > self.sysid).count() < MIN_SWEEPERS
    }

    /// The formation slot this drone takes, if it's free and not relaying,
    /// and how fast the slot is moving (see `BrainConfig::formation_slot`). The formation is
    /// every free drone not relaying — this one, the free neighbors it hears that aren't in a
    /// chain, and the free drones it knows of by status from across the swarm — in order of
    /// system ID: an ID doesn't change, and every drone knows the same ones.
    pub(crate) fn formation_spot(&self, now: u32) -> Option<(Vec2, Vec2)> {
        if self.cfg.sweep_pattern != SweepPattern::Formation || self.cfg.search != SearchStrategy::Coverage || self.order.is_some() || self.assigned.is_some() || self.relaying {
            return None;
        }
        let slot = self.slot_of(self.sysid, now)?;
        // Where it'll be a moment on: the formation moves, and the drone keeps up with it.
        let later = self.slot_of(self.sysid, now.saturating_add((SWEEP_LOOKAHEAD_S * 1000.0) as u32)).unwrap_or(slot);
        let moving = (later - slot) * (1.0 / SWEEP_LOOKAHEAD_S);
        let slot = self.toward_front(slot, now).unwrap_or(slot);
        Some((slot, clamp_length(moving, self.cfg.max_speed)))
    }

    /// Whether this drone has heard no other drone at all for `SILENT_AFTER_MS`.
    pub(crate) fn hears_no_one(&self, now: u32) -> bool {
        self.neighbors.iter().flatten().map(Neighbor::last_heard_ms).max().is_none_or(|t| now.saturating_sub(t) > SILENT_AFTER_MS)
    }

    /// Hearing none of its front free, though it takes others of it to be: across the field to
    /// where the one heard of last is by now, as far along as its own place (as far along as
    /// theirs: that goes by the clock alone). What it last heard of the rest can be what they never
    /// heard, and with that it works out other lanes than theirs, out of range of them, and would
    /// never hear otherwise. Within earshot, it goes by whom it hears, as they do (see
    /// `update_front`).
    pub(crate) fn toward_front(&self, slot: Vec2, now: u32) -> Option<Vec2> {
        let front = self.front.as_ref()?;
        if front.heard_count > 1 || now <= front.start_ms.saturating_add(ASSEMBLE_MS) {
            return None;
        }
        let path = self.cfg.front_path(front.fronts, front.index, front.count);
        // Where each of the rest was heard last (u across the field), and as far over as the
        // front has moved across the field since (on a turn into the next lane), by the clock.
        let lanes = self.cfg.front_path(front.fronts, front.index, if front.closed { self.front_places().len() } else { front.size });
        let q = |t: u32| t.saturating_sub(front.start_ms) as f32 / 1000.0 * FORMATION_SPEED;
        let heard = front.members[..front.count]
            .iter()
            .filter(|&&m| m != self.sysid)
            .flat_map(|&m| {
                let direct = self.neighbors[m as usize].as_ref().filter(|n| n.assigned.is_none()).and_then(|n| n.position).map(|(p, _, at)| (at, p));
                let status = self.swarm_heard[m as usize].filter(|(_, free, _)| *free).map(|(at, _, p)| (at, p));
                direct.into_iter().chain(status)
            })
            .map(|(at, pos)| (at, path.uv(pos).0 + lanes.at(q(now)).0 - lanes.at(q(at)).0));
        // Hearing no one at all, with any of them heard of until it went quiet and within radio
        // range, it's the radio that's gone (jammed), not them: it keeps its own place, as they
        // keep theirs, by the clock, rather than all of them flying for where the others were heard
        // last and squeezing the line together. With all of them out of range, it's this drone
        // that's strayed (with the places between gone for mice, say), and it rejoins the one heard
        // of last.
        let (here, _) = path.uv(self.last_pos);
        let last = self.neighbors.iter().flatten().map(Neighbor::last_heard_ms).max().unwrap_or(0);
        let gone_quiet = |(at, u): &(u32, f32)| last.saturating_sub(*at) <= SILENT_AFTER_MS && (here - u).abs() <= self.cfg.link_range;
        // With ranging, it knows: drones it can range but not hear mean the radio's gone. Without,
        // it goes by what it last heard, which can be wrong: two beside it gone for a mouse without
        // its hearing look like neighbors jammed.
        let radio_gone = match self.nearby {
            Some(nearby) => nearby.iter().any(Option::is_some),
            None => heard.clone().any(|d| gone_quiet(&d)),
        };
        if self.hears_no_one(now) && radio_gone {
            return None;
        }
        let (_, theirs) = heard.max_by_key(|d| d.0)?;
        let (mine, v) = path.uv(slot);
        // A spacing short of them, on its own side: when the radio goes for a whole front at once,
        // every drone does this together, and they mustn't converge on one spot (without ranging,
        // nothing else keeps them apart).
        let gap = self.cfg.formation_spacing();
        let u = if (mine - theirs).abs() <= gap { mine } else { theirs + gap * libm::copysignf(1.0, mine - theirs) };
        Some(path.field_point(u, v))
    }

    /// Holding a formation slot: keeping up with it (it moves at `moving`), and catching up if
    /// behind.
    pub(crate) fn follow_node(&self, node: Vec2, moving: Vec2, pos: Vec2) -> Vec2 {
        let offset = node - pos;
        let distance = offset.length();
        let catch_up = if distance < 1e-3 { Vec2::ZERO } else { offset * (self.cfg.max_speed * (distance / 2.0).min(1.0) / distance) };
        clamp_length(moving + catch_up, self.cfg.max_speed)
    }
}
