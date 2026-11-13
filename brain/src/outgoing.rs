//! What goes on the air, and when: beacons, heartbeats, statuses, relays.

use super::*;

/// What a drone announces in HEARTBEAT.custom_mode while following an operator order: busy,
/// so neighbors neither count it as a free searcher nor as a responder to any target.
pub(crate) const ANNOUNCE_OPERATOR: u32 = 0xFFFF;

/// Frames received for others that one tick can relay (operator commands, statuses), plus
/// this drone's own acknowledgements.
pub(crate) const OUTBOX: usize = 10;

/// Messages one tick can send: heartbeat, position, a sighting per known target and the
/// certificate of its observer, its own status, and whatever it relays.
pub const MAX_OUTGOING: usize = 4 + 2 * MAX_TRACKS + OUTBOX;

/// One frame to send: with no header, this drone's own (its system ID, the runner's sequence
/// numbers); with one, a frame relayed for someone else, sent under the header it came with —
/// like a MAVLink router, so whoever gets it knows who it's really from, however many hops
/// away.
pub type Frame = (Option<MavHeader>, MavMessage);

/// Beacons slow down in a crowd: a drone hearing more than this many neighbors stretches its
/// beacon period in proportion (up to `MAX_BEACON_STRETCH` times), so each drone hears about
/// the same number of beacons a second (80) however many are around, rather than the more the
/// more drones are near. Not fewer neighbors than this: the leash goes by how fresh the
/// neighbors' positions are, and keeping the link suffers with them any staler.
pub(crate) const BEACON_CROWD: usize = 16;
pub(crate) const MAX_BEACON_STRETCH: u32 = 5;

/// Sightings: a drone sends its own observation of a target it sees at every beacon; the rest of
/// what it knows (others' observations, targets last seen a while ago) once a
/// `SIGHTING_REPEAT_MS`, unless this many neighbors sent the same since it last did (see
/// `STATUS_COPIES`), and once a `SIGHTING_REFRESH_MS` whatever they did. Every drone repeating
/// every target at every beacon was the second biggest load on the air. Repeating less often
/// than this slowed the search down: a drone flying past hears a "last seen here" only briefly.
pub(crate) const SIGHTING_COPIES: u8 = 3;
pub(crate) const SIGHTING_REPEAT_MS: u32 = 400;
pub(crate) const SIGHTING_REFRESH_MS: u32 = 2_000;

/// Statuses are flooded through the whole swarm (the ground station wants them, and so does the
/// swarm: the formation's roster, free drones far off), but not by everyone. A drone that hears
/// a new status waits a moment (`STATUS_WAIT_MS`, staggered) and relays it only if fewer than
/// this many neighbors relayed it meanwhile: where they have, its own copy would reach nobody
/// new. The counter-based scheme of Ni et al., "The broadcast storm problem in a mobile ad hoc
/// network" (1999): with every drone relaying every status, a crowd would hear hundreds a
/// second each.
pub(crate) const STATUS_COPIES: u8 = 3;
/// How long to wait before relaying a status: this, plus a few more ticks' worth (up to this
/// many, in a crowd).
pub(crate) const STATUS_WAIT_MS: u32 = 50;
pub(crate) const STATUS_MAX_TICKS: u32 = 10;

/// Statuses that can wait to be relayed at once; past that, the drone is in a crowd big enough
/// that others will, and it doesn't.
pub(crate) const STATUS_PENDING: usize = 64;

/// A status this drone may relay, unless enough neighbors do it first.
#[derive(Clone)]
pub(crate) struct PendingStatus {
    pub(crate) header: MavHeader,
    pub(crate) status: SWARM_DRONE_STATUS_DATA,
    pub(crate) heard_at: u32,
    pub(crate) due_ms: u32,
    pub(crate) copies: u8,
}

/// Recently relayed frames, remembered so a flooded frame is relayed once, not forever.
pub(crate) const RECENT_RELAYS: usize = 32;
/// ...for this long: past it, an identical frame is new (a ground station repeating a command).
pub(crate) const RELAY_MEMORY_MS: u32 = 2_000;

/// HEARTBEAT.custom_mode as `Brain::announcement` packs it: (target it's on, settled on it,
/// link age in ms, route parent).
pub(crate) fn unpack_announcement(custom_mode: u32) -> (Option<TargetId>, bool, Option<u32>, u8) {
    let task = (custom_mode & 0xFFFF) as TargetId;
    let settled = custom_mode & ANNOUNCE_SETTLED != 0;
    let age = (custom_mode >> 16) & ANNOUNCE_NO_LINK;
    let parent = (custom_mode >> 24) as u8;
    ((task != 0).then_some(task), settled, (age != ANNOUNCE_NO_LINK).then_some(age * 100), parent)
}

/// HEARTBEAT.custom_mode bits 16..21: link age in tenths of a second, this for "no route" (and
/// one less is the most it says: past `LINK_TIMEOUT_MS` it's no route anyway).
pub(crate) const ANNOUNCE_NO_LINK: u32 = 0x1F;

/// HEARTBEAT.custom_mode bit: the drone is part of a relay chain out to a target (a relay, or
/// a responder held at the edge of reach) — the drone its route goes through mustn't leave it.
pub(crate) const ANNOUNCE_CHAIN: u32 = 1 << 21;

/// HEARTBEAT.custom_mode bit: the drone's leash is holding it back from where it needs to be:
/// it needs a relay farther out.
pub(crate) const ANNOUNCE_STRETCHED: u32 = 1 << 22;

/// HEARTBEAT.custom_mode bit announcing that the drone is settled on its target.
pub(crate) const ANNOUNCE_SETTLED: u32 = 1 << 23;

impl Brain {
    /// Phase 2: the MAVLink messages to transmit this tick (heartbeat, position, and a
    /// sighting per known target).
    /// What to send this tick, just the messages: see `outgoing_frames` for the headers.
    pub fn outgoing(&mut self, now: u32) -> [Option<MavMessage>; MAX_OUTGOING] {
        self.outgoing_frames(now).map(|f| f.map(|(_, msg)| msg))
    }

    /// What to send this tick. A frame with a header is a relay, and goes out under it.
    pub fn outgoing_frames(&mut self, now: u32) -> [Option<Frame>; MAX_OUTGOING] {
        let mut out: [Option<MavMessage>; MAX_OUTGOING] = [const { None }; MAX_OUTGOING];
        // Observers of the sightings we send, whose certificates may need to go with them.
        let mut certify = Observers::new();

        // Also sent right away when this drone takes on or drops a target, so neighbors deciding
        // who responds to what see the change within a tick instead of up to a second later.
        let announcement = self.announcement(now);
        // An urgent heartbeat for a new task or route parent; the link age alone drifts every
        // tick and rides along with the regular one.
        let changed = (self.announced ^ announcement) & !(ANNOUNCE_NO_LINK << 16) != 0 && now.saturating_sub(self.last_heartbeat_ms) >= 200;
        // While keeping the link, the heartbeat goes out with every beacon: the link age it
        // carries only travels a hop per heartbeat, and at 1 Hz it would age a second per hop.
        let link_beat = self.cfg.keep_link && self.route.is_some() && (now >= self.next_link_beat_ms || self.urgent_beacon);
        if link_beat {
            self.next_link_beat_ms = now.saturating_add(self.cfg.beacon_period_ms);
        }
        if now >= self.next_heartbeat_ms || changed || link_beat {
            #[cfg(test)]
            tests::count_cause(if now >= self.next_heartbeat_ms { 0 } else if changed { 1 } else { 2 });
            self.last_heartbeat_ms = now;
            self.next_heartbeat_ms = now.saturating_add(self.cfg.heartbeat_period_ms);
            self.announced = announcement;
            out[0] = Some(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                // Autopilot-specific mode field: here, the target this drone is on (0 = none),
                // or that it's busy with an operator order.
                custom_mode: announcement,
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
                base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            }));
        }

        // On schedule, the whole beacon; early, the position and only the news it's early for
        // (new targets, a changed count of drones on one). An early beacon that repeated every
        // target undid the sightings' savings in a crowd: with GPS error, 64 drones each sent
        // four a second.
        let scheduled = now >= self.next_beacon_ms;
        if scheduled || self.urgent_beacon {
            #[cfg(test)]
            tests::count_cause(if scheduled { 3 } else { 4 });
            self.urgent_beacon = false;
            // The position goes out early too: neighbors deciding who goes where need it with the
            // news (without it, a newcomer between a ring and the next drone, and the search and
            // the link with it, did worse).
            {
                self.next_beacon_ms = now.saturating_add(self.beacon_now_ms);
                // MAVLink's local frame is NED (z down); the field is flat, so z is just 0.
                out[1] = Some(MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
                    time_boot_ms: now,
                    x: self.last_pos.x,
                    y: self.last_pos.y,
                    z: 0.0,
                    vx: self.last_velocity.x,
                    vy: self.last_velocity.y,
                    vz: 0.0,
                }));
            }
            // Our own observations get signed as they go out, not every time the sensor sees
            // the target (a signature takes 2.4 ms on the board).
            for t in self.tracks.iter_mut().flatten().filter(|t| t.signature.is_none() && t.observer_sysid == self.sysid) {
                let mut signed = sighting_of(t, now);
                auth::sign(&self.signer, self.keys.mission, self.sysid, &mut signed, t.observed_ms);
                self.signatures_made += 1;
                t.signature = Some(signed.signature);
                // Our own, coming back from a neighbor, needn't be checked.
                self.verified[self.next_verified] = Some(Attested::of(&signed));
                self.next_verified = (self.next_verified + 1) % VERIFIED_MEMORY;
            }
            let mut slots = out[2..2 + MAX_TRACKS].iter_mut();
            let me = self.sysid;
            for t in self.tracks.iter_mut().flatten().filter(|t| t.signature.is_some() && if scheduled { worth_sending(t, me, now) } else { is_news(t) }) {
                let Some(slot) = slots.next() else { break };
                *slot = Some(MavMessage::SWARM_TARGET_SIGHTING(sighting_of(t, now)));
                t.sent = Some((now, t.observer_sysid, t.observed_ms, t.responders));
                t.copies = 0;
                if certify.len() < MAX_TRACKS && !certify.contains(&t.observer_sysid) {
                    certify.push(t.observer_sysid);
                }
            }
        }

        let mut slot = 2 + MAX_TRACKS;
        // How sure we are of our position, with every heartbeat: neighbors use it to decide
        // whether to navigate by us.
        if out[0].is_some() {
            let mut flags = EstimatorStatusFlags::empty();
            if self.nav != Nav::DeadReckoning {
                flags |= EstimatorStatusFlags::ESTIMATOR_POS_HORIZ_ABS;
            }
            if now < self.gps_rejected_until {
                flags |= EstimatorStatusFlags::ESTIMATOR_GPS_GLITCH;
            }
            out[slot] = Some(MavMessage::ESTIMATOR_STATUS(ESTIMATOR_STATUS_DATA {
                time_usec: now as u64 * 1000,
                vel_ratio: 0.0,
                pos_horiz_ratio: 0.0,
                pos_vert_ratio: 0.0,
                mag_ratio: 0.0,
                hagl_ratio: 0.0,
                tas_ratio: 0.0,
                pos_horiz_accuracy: self.pos_accuracy,
                pos_vert_accuracy: 0.0,
                flags,
            }));
            slot += 1;
        }
        let sticks_lost = self.sticks_lost(now);
        if now >= self.next_status_ms || self.urgent_status || sticks_lost != self.reported_sticks_lost {
            self.next_status_ms = now.saturating_add(self.cfg.status_period_ms);
            self.urgent_status = false;
            self.reported_sticks_lost = sticks_lost;
            self.status_seq = self.status_seq.wrapping_add(1);
            let (ack_command, ack_result) = self.last_ack.unwrap_or((0, MavResult::MAV_RESULT_ACCEPTED));
            out[slot] = Some(MavMessage::SWARM_DRONE_STATUS(SWARM_DRONE_STATUS_DATA {
                age_ms: 0,
                x: self.last_pos.x,
                y: self.last_pos.y,
                target_id: self.assigned.unwrap_or(0),
                ack_command,
                sysid: self.sysid,
                seq: self.status_seq,
                hops: 0,
                mode: self.mode.to_u8(),
                // 5: flown by hand, but the sticks' signal is lost and it's hovering.
                order: if self.sticks_lost(now) { 5 } else { Order::code(self.order) },
                ack_result: ack_result as u8,
                nav: self.nav.to_u8(),
                pos_accuracy_dm: (self.pos_accuracy * 10.0).clamp(0.0, 255.0) as u8,
                fleet: self.highest_sysid,
            }));
            slot += 1;
        }
        // Now and then, the certificate of each observer whose sightings we send, ours too: so
        // whoever gets them can check them, however far from their observer.
        for &observer in certify.iter() {
            let Some((public_key, signature)) = self.certificates[observer as usize] else { continue };
            let sent = &mut self.certificate_sent[observer as usize];
            if sent.is_some_and(|at| now.saturating_sub(at) < CERT_REPEAT_MS) {
                continue;
            }
            *sent = Some(now);
            out[slot] = Some(MavMessage::SWARM_KEY_CERT(SWARM_KEY_CERT_DATA { mission: self.keys.mission, sysid: observer, public_key, signature }));
            slot += 1;
        }
        let mut out = out.map(|m| m.map(|m| (None, m)));
        for queued in self.outbox.iter_mut() {
            if let Some(frame) = queued.take() {
                out[slot] = Some(frame);
                slot += 1;
            }
        }

        out
    }

    /// The beacon period, stretched in a crowd (see `BEACON_CROWD`).
    pub(crate) fn beacon_period(&self, now: u32) -> u32 {
        let heard = self
            .neighbors
            .iter()
            .flatten()
            .filter(|n| n.position.is_some_and(|(_, _, at)| now.saturating_sub(at) <= self.cfg.neighbor_timeout_ms))
            .count();
        let period = self.cfg.beacon_period_ms as u64 * heard.max(BEACON_CROWD) as u64 / BEACON_CROWD as u64;
        period.min((self.cfg.beacon_period_ms * MAX_BEACON_STRETCH) as u64) as u32
    }

    /// HEARTBEAT.custom_mode: the target this drone is on, `ANNOUNCE_OPERATOR` while it follows
    /// an operator order, 0 if it's free.
    ///
    /// Bits 16..21 carry the age of our freshest news of the ground station in tenths of a
    /// second (`ANNOUNCE_NO_LINK`, 0x1F: no route; news past `LINK_TIMEOUT_MS` is no route
    /// anyway), bit 21 whether we're in a relay chain (`ANNOUNCE_CHAIN`), bit 22 whether the
    /// leash is holding us back (`ANNOUNCE_STRETCHED`), bit 23 whether we've settled on our
    /// target (`ANNOUNCE_SETTLED`), and bits 24..32 the neighbor our route goes through, for
    /// neighbors working out their own route.
    pub(crate) fn announcement(&self, now: u32) -> u32 {
        let task = if self.order.is_some() { ANNOUNCE_OPERATOR } else { self.assigned.unwrap_or(0) as u32 };
        let settled = if self.order.is_none() && self.assigned.is_some() && self.settled(now) { ANNOUNCE_SETTLED } else { 0 };
        // Rounded up: rounding down would make news a little younger at every hop, enough to
        // cancel out the hop latency and let a routing loop keep itself alive forever.
        let (age, parent) = self.route.map_or((ANNOUNCE_NO_LINK, 0), |(age, p)| (age.div_ceil(100).min(ANNOUNCE_NO_LINK - 1), p as u32));
        let chain = if self.in_chain() { ANNOUNCE_CHAIN } else { 0 };
        let stretched = if self.stretched { ANNOUNCE_STRETCHED } else { 0 };
        task | settled | chain | stretched | (age << 16) | (parent << 24)
    }

    /// Holds a new status to relay a moment later (see `STATUS_COPIES`), staggered by sysid and
    /// seq (not the PRNG: that would change what the drone does).
    pub(crate) fn hold_status(&mut self, header: MavHeader, st: &SWARM_DRONE_STATUS_DATA, now: u32) {
        // Spread over more ticks in a crowd, so only a few relay in the first one and the rest
        // hear them first: about `STATUS_COPIES` a tick.
        let heard = self.neighbors.iter().flatten().filter(|n| now.saturating_sub(n.heartbeat_at_ms) <= self.cfg.neighbor_timeout_ms).count() as u32;
        let ticks = (heard / STATUS_COPIES as u32).clamp(3, STATUS_MAX_TICKS);
        let stagger = (self.sysid as u32 * 37 + st.seq as u32 * 11) % ticks * TICK_MS;
        let pending = PendingStatus { header, status: st.clone(), heard_at: now, due_ms: now.saturating_add(STATUS_WAIT_MS + stagger), copies: 0 };
        // A newer status for the same drone replaces an older one still waiting.
        let slot = self.status_pending.iter().position(|p| p.as_ref().is_some_and(|p| p.status.sysid == st.sysid))
            .or_else(|| self.status_pending.iter().position(Option::is_none));
        if let Some(i) = slot {
            self.status_pending[i] = Some(pending);
        }
    }

    pub(crate) fn relay_due_statuses(&mut self, now: u32) {
        for i in 0..STATUS_PENDING {
            if self.status_pending[i].as_ref().is_some_and(|p| now >= p.due_ms) {
                let p = self.status_pending[i].take().expect("just checked");
                if p.copies < STATUS_COPIES {
                    self.relay_status(p, now);
                }
            }
        }
    }

    pub(crate) fn relay_status(&mut self, p: PendingStatus, now: u32) {
        let mut relayed = p.status;
        relayed.hops += 1;
        relayed.age_ms = relayed.age_ms.saturating_add(now.saturating_sub(p.heard_at) + self.cfg.hop_latency_ms);
        self.relay(p.header, MavMessage::SWARM_DRONE_STATUS(relayed));
    }

    /// One of our own frames, for the next `outgoing`.
    pub(crate) fn queue(&mut self, msg: MavMessage) {
        self.enqueue((None, msg));
    }

    /// Someone else's frame, passed on under its own header.
    pub(crate) fn relay(&mut self, header: MavHeader, msg: MavMessage) {
        self.enqueue((Some(header), msg));
    }

    pub(crate) fn enqueue(&mut self, frame: Frame) {
        match self.outbox.iter_mut().find(|m| m.is_none()) {
            Some(slot) => *slot = Some(frame),
            None => self.outbox_dropped = self.outbox_dropped.saturating_add(1),
        }
    }

    /// True the first time a frame with this exact content is seen within `RELAY_MEMORY_MS`,
    /// so flooding stops: each drone relays a given frame once.
    pub(crate) fn first_time(&mut self, msg: &MavMessage, now: u32) -> bool {
        let hash = fingerprint(msg);
        if self.recent_relays.iter().any(|&(h, at)| h == hash && now.saturating_sub(at) <= RELAY_MEMORY_MS && at != 0) {
            return false;
        }
        self.recent_relays[self.next_relay_slot] = (hash, now.max(1));
        self.next_relay_slot = (self.next_relay_slot + 1) % RECENT_RELAYS;
        true
    }
}
