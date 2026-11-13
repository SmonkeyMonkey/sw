//! Targets: tracks, sightings taken in and checked, merging, quorum, refuting.

use super::*;

/// Targets one drone keeps track of at once.
pub const MAX_TRACKS: usize = 6;
/// Targets remembered per target as seen apart from it (`Track::apart_from`).
pub(crate) const MAX_APART: usize = 4;

/// A report under a target's name is taken for that target up to this many times
/// `agree_radius` off (the observer's GPS error): see the sightings in `on_tick`.
pub(crate) const NAME_RADIUS_FACTOR: f32 = 2.0;

/// Seen within this long, for a target to count as short of drones.
pub(crate) const SHORT_FRESH_MS: u32 = 2_000;

/// Both seen within this long, for two targets to count as seen apart at the same moment.
pub(crate) const APART_FRESH_MS: u32 = 1_000;
/// A track not heard of for this long takes a report under its name however far off (see
/// `take_sighting`): the target has had time to move.
pub(crate) const STALE_NAME_MS: u32 = 3_000;

/// Distinct witnesses remembered per target — only `quorum` of them are ever needed.
pub(crate) const MAX_WITNESSES: usize = 6;

/// A drone that recently saw a target firsthand, counted toward its quorum.
#[derive(Clone, Copy)]
pub(crate) struct Witness {
    pub(crate) sysid: u8,
    pub(crate) seen_at_ms: LocalMs,
}

/// Everything this drone knows about one target.
#[derive(Clone, Copy)]
pub(crate) struct Track {
    pub(crate) id: TargetId,
    /// Where the target was when last observed, and how it was moving; see `Brain::predict`.
    pub(crate) pos: Vec2,
    pub(crate) vel: Vec2,
    /// When the original observer actually saw the target, not when it was relayed — so the
    /// freshest observation wins no matter how many hops it took. On *this* drone's clock:
    /// converted from the age carried in the message, never compared with another drone's clock.
    pub(crate) observed_at_ms: LocalMs,
    pub(crate) observer_sysid: u8,
    /// The same moment on the observer's own clock, and the observer's signature on the
    /// observation (see `auth`); None for our own until it's signed for sending.
    pub(crate) observed_ms: u32,
    pub(crate) signature: Option<[u8; 64]>,
    /// When we last broadcast this track, and what: which observation (observer, its
    /// `observed_ms`), and how many drones it said were on the target.
    pub(crate) sent: Option<(u32, u8, u32, u8)>,
    /// Sightings of it relayed by neighbors since we last sent it (see `SIGHTING_COPIES`).
    pub(crate) copies: u8,
    pub(crate) hops: u8,
    /// Who we heard the freshest observation from (the frame's sender). Only when that's the
    /// observer itself, firsthand, do we know it really said so: anyone can put any observer
    /// in a sighting, and a sighting relayed further than one hop says nothing about who made
    /// it up.
    pub(crate) heard_from: u8,
    /// Drones on this target, as the observer of the freshest observation counted them.
    pub(crate) responders: u8,
    /// The operator cancelled it: kept track of (and relayed, with the flag) but not responded to.
    pub(crate) cancelled: bool,
    /// We've already marked the ground around where it was last seen as unsearched.
    pub(crate) hinted: bool,
    /// When it last had a full set of responders, as far as we could tell (0: never).
    pub(crate) full_at_ms: u32,
    pub(crate) trace_id: [u8; 16],
    /// This drone's own span for the target; cited as parent by whoever we relay it to.
    pub(crate) span_id: [u8; 8],
    pub(crate) witnesses: [Option<Witness>; MAX_WITNESSES],
    /// Our own sensor sees it this tick.
    pub(crate) seen_now: bool,
    /// Our own previous sighting, for estimating its velocity — the sensor only gives positions.
    pub(crate) own_last: Option<(Vec2, u32)>,
    /// Other targets this one has been seen well apart from at the same moment (0: empty): it
    /// can't be the same target as any of them, however close they get — see `note_apart`.
    pub(crate) apart_from: [TargetId; MAX_APART],
}

impl Track {
    pub(crate) fn apart(&self, id: TargetId) -> bool {
        self.apart_from.contains(&id)
    }

    pub(crate) fn note_apart(&mut self, id: TargetId) {
        if id == 0 || id == self.id || self.apart(id) {
            return;
        }
        // Full: forget the oldest.
        self.apart_from.rotate_right(1);
        self.apart_from[0] = id;
    }
}

impl Track {
    pub(crate) fn witness(&mut self, sysid: u8, seen_at_ms: LocalMs) {
        if let Some(w) = self.witnesses.iter_mut().flatten().find(|w| w.sysid == sysid) {
            w.seen_at_ms = w.seen_at_ms.max(seen_at_ms);
            return;
        }
        // A free slot, or else the stalest witness makes room.
        let slot = self.witnesses.iter().position(Option::is_none).unwrap_or_else(|| {
            (0..MAX_WITNESSES)
                .min_by_key(|&i| self.witnesses[i].map_or(LocalMs::MIN, |w| w.seen_at_ms))
                .unwrap_or(0)
        });
        self.witnesses[slot] = Some(Witness { sysid, seen_at_ms });
    }

    pub(crate) fn forget_witness(&mut self, sysid: u8) {
        for w in self.witnesses.iter_mut() {
            if w.is_some_and(|w| w.sysid == sysid) {
                *w = None;
            }
        }
    }
}

/// Signatures checked per tick at most: one takes ~6 ms on the board, and a tick is 50 ms. What
/// doesn't get checked waits in `backlog` (see `Brain::authenticate`).
pub(crate) const MAX_VERIFY_PER_TICK: u8 = 3;
/// Sightings waiting for a signature check or for their observer's certificate...
pub(crate) const BACKLOG: usize = 8;
/// ...for at most this long.
pub(crate) const BACKLOG_MAX_MS: u32 = 1_000;
/// Observations whose signatures checked out, remembered so the copies every relay passes on
/// aren't checked again.
pub(crate) const VERIFIED_MEMORY: usize = 32;
/// How often a drone passes on the certificate of an observer whose sightings it sends.
pub(crate) const CERT_REPEAT_MS: u32 = 2_000;

/// An observation as signed (see `auth::sighting_bytes`) and its signature, to tell a copy of
/// one already checked from anything else. The signature too: the right observation under a
/// wrong signature would be taken here, and then fail with everyone we relay it to.
#[derive(Clone, Copy, PartialEq)]
pub(crate) struct Attested {
    pub(crate) observer: u8,
    pub(crate) observed_ms: u32,
    pub(crate) numbers: [u32; 4],
    pub(crate) signature: [u8; 64],
}

impl Attested {
    pub(crate) fn of(s: &SWARM_TARGET_SIGHTING_DATA) -> Self {
        Self { observer: s.observer_sysid, observed_ms: s.observed_ms, numbers: [s.x, s.y, s.vx, s.vy].map(f32::to_bits), signature: s.signature }
    }
}

/// Whether a sighting's signature checks out (see `Brain::authenticate`).
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum Authentic {
    Yes,
    /// Can't tell yet: no certificate for its observer, or no time left this tick.
    NotYet,
    No,
}

/// A track's observation as a sighting to send: signed, if it's been (see `Track::signature`).
pub(crate) fn sighting_of(t: &Track, now: u32) -> SWARM_TARGET_SIGHTING_DATA {
    SWARM_TARGET_SIGHTING_DATA {
        target_id: t.id,
        age_ms: age_at(now, t.observed_at_ms),
        x: t.pos.x,
        y: t.pos.y,
        vx: t.vel.x,
        vy: t.vel.y,
        observer_sysid: t.observer_sysid,
        cancelled: t.cancelled as u8,
        responders: t.responders,
        hops: t.hops,
        trace_id: t.trace_id,
        parent_span_id: t.span_id,
        observed_ms: t.observed_ms,
        signature: t.signature.unwrap_or([0; 64]),
    }
}

/// Whether a track is news worth an early beacon: new to us, or a different count of drones on
/// it than we last said.
pub(crate) fn is_news(t: &Track) -> bool {
    t.sent.is_none_or(|(_, _, _, responders)| responders != t.responders)
}

/// Whether a track goes out with this beacon (see `SIGHTING_COPIES`).
pub(crate) fn worth_sending(t: &Track, me: u8, now: u32) -> bool {
    let Some((sent_at, _, _, responders)) = t.sent else {
        // New to us: right away.
        return true;
    };
    // Our own, while we see it: every beacon, it's the freshest there is.
    if t.observer_sysid == me && t.seen_now {
        return true;
    }
    // A different count of drones on it is news drones far off need, to know whether to come.
    if responders != t.responders {
        return true;
    }
    let since = now.saturating_sub(sent_at);
    since >= SIGHTING_REFRESH_MS || (since >= SIGHTING_REPEAT_MS && t.copies < SIGHTING_COPIES)
}

/// A few distinct system IDs, without a heap.
pub(crate) struct Observers {
    pub(crate) ids: [u8; MAX_TRACKS],
    pub(crate) len: usize,
}

impl Observers {
    pub(crate) fn new() -> Self {
        Self { ids: [0; MAX_TRACKS], len: 0 }
    }
    pub(crate) fn len(&self) -> usize {
        self.len
    }
    pub(crate) fn contains(&self, id: &u8) -> bool {
        self.ids[..self.len].contains(id)
    }
    pub(crate) fn push(&mut self, id: u8) {
        self.ids[self.len] = id;
        self.len += 1;
    }
    pub(crate) fn iter(&self) -> core::slice::Iter<'_, u8> {
        self.ids[..self.len].iter()
    }
}

impl Brain {
    pub(crate) fn distrusts(&self, sysid: u8, now: u32) -> bool {
        now < self.distrusted_until[sysid as usize]
    }

    /// Where an observation made at `seen_at_ms` puts the target by `now`, assuming it kept
    /// its velocity (for at most `max_prediction_ms`).
    pub(crate) fn predict(&self, pos: Vec2, vel: Vec2, seen_at_ms: LocalMs, now: u32) -> Vec2 {
        let elapsed = age_at(now, seen_at_ms).min(self.cfg.max_prediction_ms);
        pos + vel * (elapsed as f32 / 1000.0)
    }

    pub(crate) fn predicted(&self, t: &Track, now: u32) -> Vec2 {
        self.predict(t.pos, t.vel, t.observed_at_ms, now)
    }

    /// Distinct trusted drones that saw this target recently.
    pub(crate) fn votes(&self, t: &Track, now: u32) -> u8 {
        t.witnesses
            .iter()
            .flatten()
            .filter(|w| age_at(now, w.seen_at_ms) <= self.cfg.vote_window_ms && !self.distrusts(w.sysid, now))
            .count() as u8
    }

    /// Seen by us right now, or by enough independent witnesses recently.
    pub(crate) fn confirmed(&self, t: &Track, now: u32) -> bool {
        t.seen_now || self.votes(t, now) >= self.cfg.quorum
    }

    /// The track a target at `pos` (now) belongs to: the nearest one predicted within
    /// `agree_radius`, other than those `skip` rules out. This is how unlabeled positions become
    /// "the same target" over time.
    pub(crate) fn track_near_except(&self, pos: Vec2, now: u32, skip: impl Fn(&Track) -> bool) -> Option<usize> {
        let mut best: Option<(usize, f32)> = None;
        for (i, t) in self.tracks.iter().enumerate() {
            let Some(t) = t else { continue };
            if skip(t) {
                continue;
            }
            let d = (self.predicted(t, now) - pos).length();
            if d <= self.cfg.agree_radius && best.is_none_or(|(_, bd)| d < bd) {
                best = Some((i, d));
            }
        }
        best.map(|(i, _)| i)
    }

    pub(crate) fn track_by_id(&self, id: TargetId) -> Option<usize> {
        self.tracks.iter().position(|t| t.is_some_and(|t| t.id == id))
    }

    /// A slot for a new track: a free one, or else the stalest track's.
    pub(crate) fn free_slot(&self) -> usize {
        self.tracks.iter().position(Option::is_none).unwrap_or_else(|| {
            (0..MAX_TRACKS)
                .min_by_key(|&i| self.tracks[i].map_or(LocalMs::MIN, |t| t.observed_at_ms))
                .unwrap_or(0)
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_track(&self, id: TargetId, pos: Vec2, vel: Vec2, seen_at_ms: LocalMs, observer: u8, hops: u8, trace_id: [u8; 16]) -> Track {
        Track {
            id,
            pos,
            vel,
            observed_at_ms: seen_at_ms,
            observer_sysid: observer,
            observed_ms: 0,
            signature: None,
            sent: None,
            copies: 0,
            hops,
            heard_from: observer,
            responders: 0,
            cancelled: self.cancelled_ids.contains(&id),
            hinted: false,
            full_at_ms: 0,
            apart_from: [0; MAX_APART],
            trace_id,
            span_id: [0; 8],
            witnesses: [None; MAX_WITNESSES],
            seen_now: false,
            own_last: None,
        }
    }

    /// Takes in a sighting from another drone whose signature checked out.
    pub(crate) fn take_sighting(&mut self, header: &MavHeader, theirs: &SWARM_TARGET_SIGHTING_DATA, now: u32, event: &mut Option<TraceEvent>) {
        let observer = theirs.observer_sysid;
        // Their age, plus our estimate of the hop, turned into a time on our own clock.
        let age_ms = theirs.age_ms.saturating_add(self.cfg.hop_latency_ms);
        let seen_at_ms = self.seen_at(observer, theirs.observed_ms, now as LocalMs - age_ms as LocalMs);
        let pos = Vec2::new(theirs.x, theirs.y);
        let vel = Vec2::new(theirs.vx, theirs.vy);
        let hops = theirs.hops.saturating_add(1);

        // By name if the position agrees too — a name can end up on the wrong target (see
        // `merge_radius`), and position is what's actually observed. "Agrees" allows for
        // the observer's GPS: up to `name_radius` off is still the same target, unless our
        // own sensor sees it right now (then only `agree_radius`, and a report farther off
        // than that is the one that's wrong). Narrower than the GPS error, one mouse would turn
        // into several copies under one name, and the count of who's on it with them.
        let theirs_now = self.predict(pos, vel, seen_at_ms, now);
        let name_radius = NAME_RADIUS_FACTOR * self.cfg.agree_radius;
        let by_id = self.track_by_id(theirs.target_id);
        let off = by_id.and_then(|i| self.tracks[i]).map(|t| (t.seen_now, (self.predicted(&t, now) - theirs_now).length()));
        if off.is_some_and(|(seen, d)| seen && d > self.cfg.agree_radius && d <= name_radius) {
            return;
        }
        // Or, unheard of a while, wherever it is now: a mouse runs off from where it was last
        // reported, and news of it after a gap is far from there, not a second mouse.
        let stale = by_id.and_then(|i| self.tracks[i]).is_some_and(|t| !t.seen_now && age_at(now, t.observed_at_ms) > STALE_NAME_MS && seen_at_ms > t.observed_at_ms);
        let found = by_id
            .filter(|_| stale || off.is_some_and(|(_, d)| d <= name_radius))
            .or_else(|| self.track_near_except(theirs_now, now, |t| t.apart(theirs.target_id)));
        let i = match found {
            Some(i) => i,
            None => {
                let slot = self.free_slot();
                let mut track = self.new_track(theirs.target_id, pos, vel, seen_at_ms, observer, hops, theirs.trace_id);
                // The freshness check below won't update a track this new, and without the
                // count a newcomer takes a place the target no longer has.
                track.responders = theirs.responders;
                track.heard_from = header.system_id;
                track.observed_ms = theirs.observed_ms;
                track.signature = Some(theirs.signature);
                self.tracks[slot] = Some(track);
                if event.is_none() {
                    *event = Some(TraceEvent::Adopted {
                        trace_id: theirs.trace_id,
                        parent_span_id: theirs.parent_span_id,
                        observer_sysid: observer,
                        hops,
                        age_ms,
                    });
                    self.event_track = Some(theirs.target_id);
                }
                // Relay new news right away instead of waiting for the next scheduled beacon.
                self.urgent_beacon = true;
                slot
            }
        };
        let same_spot = self.tracks[i].is_some_and(|t| !t.apart(theirs.target_id) && (self.predicted(&t, now) - theirs_now).length() <= self.cfg.merge_radius);
        if same_spot {
            let old = self.tracks[i].expect("just found or created").id;
            self.rename_track(i, old.min(theirs.target_id));
        }
        let t = self.tracks[i].as_mut().expect("just found or created");
        // Not `theirs.cancelled`: only the operator's command cancels a target (see
        // `execute`), not another drone saying so, or any one drone could call any target
        // off. The ground station repeats the command for drones that missed it.
        t.witness(observer, seen_at_ms);
        if seen_at_ms > t.observed_at_ms {
            // Fresher news about a target we already know: update the facts but keep our
            // own trace context, so our span ID keeps pointing into the trace we cite.
            t.pos = pos;
            t.vel = vel;
            t.observed_at_ms = seen_at_ms;
            t.observer_sysid = observer;
            t.hops = hops;
            t.heard_from = header.system_id;
            t.observed_ms = theirs.observed_ms;
            t.signature = Some(theirs.signature);
            t.responders = theirs.responders;
        }
        // Neighbors passing it on: the more, the less we need to. Relays only, not the drones
        // that see it: if only they talk about it, whoever is out of their range hears it from
        // us or not at all.
        if theirs.hops > 0 {
            t.copies = t.copies.saturating_add(1);
        }
    }

    /// When an observation was made, on our clock: as long ago as the relays say, but no more
    /// recently than the observer's own clock allows — one it made 5 s before one we took 2 s
    /// ago is at least 7 s old, whatever age a relay puts on it. So replaying an old signed
    /// sighting as news doesn't work once we've heard anything newer from its observer.
    pub(crate) fn seen_at(&mut self, observer: u8, observed_ms: u32, claimed: LocalMs) -> LocalMs {
        let clock = &mut self.observer_clock[observer as usize];
        match *clock {
            Some((newest, at)) if observed_ms <= newest => claimed.min(at - (newest - observed_ms) as LocalMs),
            _ => {
                *clock = Some((observed_ms, claimed));
                claimed
            }
        }
    }

    /// Learns a drone's public key from its certificate, if the launch base issued it for this
    /// mission. Costs a signature check, so a drone's is checked only until we have it.
    pub(crate) fn learn_certificate(&mut self, cert: &SWARM_KEY_CERT_DATA) {
        if cert.sysid == 0
            || cert.mission != self.keys.mission
            || self.certificates[cert.sysid as usize].is_some()
            || self.is_revoked(cert.sysid)
            || self.verify_budget == 0
        {
            return;
        }
        self.verify_budget -= 1;
        self.checks_done += 1;
        if auth::certificate_valid(cert, self.keys.mission, &self.keys.authority) {
            self.certificates[cert.sysid as usize] = Some((cert.public_key, cert.signature));
        } else {
            self.forged += 1;
        }
    }

    /// Whether a sighting is what its observer signed. A copy of one already checked (every
    /// relay passes on the same observation, signature and all) costs nothing; anything else
    /// costs a check, as long as this tick has any left.
    pub(crate) fn authenticate(&mut self, s: &SWARM_TARGET_SIGHTING_DATA) -> Authentic {
        let Some((public, _)) = self.certificates[s.observer_sysid as usize] else {
            return Authentic::NotYet;
        };
        let attested = Attested::of(s);
        if self.verified.contains(&Some(attested)) {
            return Authentic::Yes;
        }
        if self.verify_budget == 0 {
            return Authentic::NotYet;
        }
        self.verify_budget -= 1;
        self.checks_done += 1;
        if auth::verify(&public, &auth::sighting_bytes(self.keys.mission, s), &s.signature) {
            self.remember_verified(attested);
            Authentic::Yes
        } else {
            Authentic::No
        }
    }

    pub(crate) fn remember_verified(&mut self, attested: Attested) {
        self.verified[self.next_verified] = Some(attested);
        self.next_verified = (self.next_verified + 1) % VERIFIED_MEMORY;
    }

    /// Keeps a sighting to check on a later tick, unless a copy is already waiting, there's no
    /// room, or it's been waiting too long: then it's dropped, like a lost frame (its observer
    /// sends a newer one soon enough).
    pub(crate) fn wait_for_check(&mut self, header: MavHeader, s: SWARM_TARGET_SIGHTING_DATA, received: u32, now: u32) {
        let attested = Attested::of(&s);
        if self.backlog.iter().flatten().any(|(_, w, _)| Attested::of(w) == attested) {
            return;
        }
        let slot = self.backlog.iter_mut().find(|w| w.is_none());
        match slot {
            Some(slot) if now.saturating_sub(received) <= BACKLOG_MAX_MS => *slot = Some((header, s, received)),
            _ => self.unchecked_dropped += 1,
        }
    }

    /// Remembers which targets have (or just had) all their responders: by the count reported
    /// with their sightings, or by the responders we hear ourselves.
    pub(crate) fn note_full_targets(&mut self, now: u32) {
        let responders = self.join_share;
        for i in 0..MAX_TRACKS {
            let Some(t) = self.tracks[i] else { continue };
            let heard = self.responders_heard(t.id, now) + (self.assigned == Some(t.id)) as u8;
            if t.responders.max(heard) >= responders {
                if let Some(t) = self.tracks[i].as_mut() {
                    t.full_at_ms = now.max(1);
                }
            }
        }
    }

    /// Not seen for `forget_after_ms`: no longer a target to fly to, just a "last seen here".
    pub(crate) fn lost(&self, t: &Track, now: u32) -> bool {
        age_at(now, t.observed_at_ms) > self.cfg.forget_after_ms
    }

    /// For each target that has just been lost (or that we've just heard of, already lost),
    /// marks the ground around where it should be as unsearched, once: the coverage search then
    /// sends the free drones nearest to it there to look again.
    pub(crate) fn hint_lost_targets(&mut self, now: u32) {
        for i in 0..MAX_TRACKS {
            let Some(t) = self.tracks[i] else { continue };
            if t.hinted || t.cancelled || !self.lost(&t, now) {
                continue;
            }
            // Where it has probably got to by now: carried on along its last known velocity for
            // longer than a live track is extrapolated (the drones are coming to look, not to
            // hold a ring, so a rough guess beats where it was).
            let elapsed = age_at(now, t.observed_at_ms).min(8_000) as f32 / 1000.0;
            let around = t.pos + t.vel * elapsed;
            let radius = 2.0 * self.cfg.scan_radius;
            for x in 0..GRID {
                for y in 0..GRID {
                    if (self.cell_center((x, y)) - around).length() <= radius {
                        self.searched_at[x][y] = LAST_SEEN_HINT;
                    }
                }
            }
            if let Some(t) = self.tracks[i].as_mut() {
                t.hinted = true;
            }
        }
    }

    /// Two tracks predicted at the same spot are one target that two drones spotted
    /// independently: keep the lower ID (so the whole swarm converges on one name), the
    /// freshest facts, and everyone's witnesses.
    pub(crate) fn merge_duplicate_tracks(&mut self, now: u32) {
        self.note_targets_apart(now);
        for a in 0..MAX_TRACKS {
            for b in (a + 1)..MAX_TRACKS {
                let (Some(ta), Some(tb)) = (self.tracks[a], self.tracks[b]) else { continue };
                // Both in our own view at once, or seen well apart before: definitely two
                // targets. Under one name, they're one however far apart (see the sightings
                // above), unless we see both.
                // Or either is a while out of date: where it was isn't where it is (see
                // `STALE_NAME_MS`).
                let stale = |t: &Track| !t.seen_now && age_at(now, t.observed_at_ms) > STALE_NAME_MS;
                let same_name = ta.id == tb.id
                    && ((self.predicted(&ta, now) - self.predicted(&tb, now)).length() <= NAME_RADIUS_FACTOR * self.cfg.agree_radius || stale(&ta) || stale(&tb));
                if (ta.seen_now && tb.seen_now)
                    || (!same_name
                        && (ta.apart(tb.id)
                            || tb.apart(ta.id)
                            || (self.predicted(&ta, now) - self.predicted(&tb, now)).length() > self.cfg.merge_radius))
                {
                    continue;
                }
                let (mut keep, other, keep_slot, drop_slot) = if ta.id <= tb.id { (ta, tb, a, b) } else { (tb, ta, b, a) };
                if other.observed_at_ms > keep.observed_at_ms {
                    keep.pos = other.pos;
                    keep.vel = other.vel;
                    keep.observed_at_ms = other.observed_at_ms;
                    keep.observer_sysid = other.observer_sysid;
                    keep.observed_ms = other.observed_ms;
                    keep.signature = other.signature;
                    keep.copies = other.copies;
                    keep.hops = other.hops;
                    keep.heard_from = other.heard_from;
                    keep.responders = other.responders;
                }
                for w in other.witnesses.iter().flatten() {
                    keep.witness(w.sysid, w.seen_at_ms);
                }
                keep.seen_now |= other.seen_now;
                for &id in other.apart_from.iter() {
                    keep.note_apart(id);
                }
                keep.cancelled |= other.cancelled;
                if keep.own_last.is_none() {
                    keep.own_last = other.own_last;
                }
                if self.assigned == Some(other.id) {
                    self.assigned = Some(keep.id);
                }
                self.tracks[keep_slot] = Some(keep);
                self.tracks[drop_slot] = None;
            }
        }
    }

    /// Two targets under different names, both freshly seen and well apart (twice
    /// `merge_radius`: farther than one target's sightings ever spread with GPS error), are two
    /// targets for good. Without this, two mice running side by side, closer than
    /// `merge_radius`, became one track — and the drones on one of them found themselves one
    /// ring too many on the other, and let it go.
    pub(crate) fn note_targets_apart(&mut self, now: u32) {
        for a in 0..MAX_TRACKS {
            for b in (a + 1)..MAX_TRACKS {
                let (Some(ta), Some(tb)) = (self.tracks[a], self.tracks[b]) else { continue };
                let fresh = |t: &Track| age_at(now, t.observed_at_ms) <= APART_FRESH_MS;
                if ta.id == tb.id || !fresh(&ta) || !fresh(&tb) {
                    continue;
                }
                if (self.predicted(&ta, now) - self.predicted(&tb, now)).length() >= 2.0 * self.cfg.merge_radius {
                    let (ida, idb) = (ta.id, tb.id);
                    if let Some(t) = self.tracks[a].as_mut() {
                        t.note_apart(idb);
                    }
                    if let Some(t) = self.tracks[b].as_mut() {
                        t.note_apart(ida);
                    }
                }
            }
        }
    }

    /// Gives track `i` a new name, and our assignment with it: still the same target, so a
    /// responder on it stays on it rather than suddenly weighing it as a newcomer would.
    pub(crate) fn rename_track(&mut self, i: usize, id: TargetId) {
        let Some(t) = self.tracks[i].as_mut() else { return };
        if self.assigned == Some(t.id) {
            self.assigned = Some(id);
        }
        t.id = id;
    }

    /// Checking reports in person: a drone standing where an unconfirmed target should be that
    /// sees nothing there drops it — and if the report was fresh, it wasn't just out of date,
    /// so its observer stops being trusted (and stops counting as a witness anywhere). Only if
    /// we heard it from the observer itself, though (see `Track::heard_from`): blaming whoever a
    /// report names would let anyone frame an honest drone with a made-up sighting in its name,
    /// and get it shut out of the swarm's consensus.
    pub(crate) fn refute(&mut self, pos: Vec2, now: u32) -> Option<u8> {
        let mut refuted = None;
        for i in 0..MAX_TRACKS {
            let Some(t) = self.tracks[i] else { continue };
            if t.cancelled || self.lost(&t, now) || self.confirmed(&t, now) || (self.predicted(&t, now) - pos).length() > self.cfg.verify_radius {
                continue;
            }
            self.tracks[i] = None;
            let fresh = age_at(now, t.observed_at_ms) <= self.cfg.refute_max_age_ms;
            let firsthand = t.hops == 1 && t.heard_from == t.observer_sysid;
            if fresh && firsthand && t.observer_sysid != self.sysid {
                let liar = t.observer_sysid;
                self.distrusted_until[liar as usize] = now.saturating_add(self.cfg.distrust_ms);
                for slot in self.tracks.iter_mut() {
                    if let Some(other) = slot.as_mut() {
                        other.forget_witness(liar);
                        // Gone if the liar was its only source.
                        if other.observer_sysid == liar && other.witnesses.iter().all(Option::is_none) {
                            *slot = None;
                        }
                    }
                }
                refuted = Some(liar);
            }
        }
        refuted
    }
}
