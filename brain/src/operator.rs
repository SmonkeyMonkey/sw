//! The operator's orders: checked, carried out, acknowledged, relayed.

use super::*;

/// The commands the ground station can give the whole swarm at once (`target_system` 0).
/// Others are for one drone at a time: a go-to or a return home for everyone is a mistake.
pub(crate) const BROADCAST_COMMANDS: [MavCmd; 3] = [MavCmd::MAV_CMD_SWARM_SET_MODE, MavCmd::MAV_CMD_SWARM_SET_RESPONDERS, MavCmd::MAV_CMD_SWARM_CANCEL_TARGET];

/// How far inside the field's edge a go-to order is moved, if it was for a spot on or past it.
pub(crate) const GOTO_MARGIN_M: f32 = 1.0;

/// An operator order a drone follows instead of deciding for itself, until told to resume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Order {
    /// Fly to this spot and stay there.
    GoTo(Vec2),
    /// Stay where it was when ordered.
    Hold(Vec2),
    /// Fly back to the base and land.
    ReturnHome,
    /// Flown by hand from the remote: `stick` is where the operator is pushing (each axis
    /// -1..1, +y north, +x east), as of `at_ms`. The ground station numbers its stick input
    /// (MANUAL_CONTROL.buttons carries the number; our drones have no buttons to read), so old
    /// input relayed late or twice is ignored. If the sticks go quiet for `STICK_TIMEOUT_MS`
    /// (signal lost), it hovers where it is until they come back or it's released.
    Manual { stick: Vec2, at_ms: u32 },
}

/// Manual control with no stick input for this long: the link is presumed lost and the drone
/// hovers.
pub const STICK_TIMEOUT_MS: u32 = 1_000;
/// ...and after this long without them, it gives up on the operator and rejoins the swarm.
pub const STICKS_GIVE_UP_MS: u32 = 5_000;

/// Back from a return-to-launch this close to the base, it asks to land.
pub(crate) const HOME_LAND_M: f32 = 5.0;

impl Order {
    /// Wire encoding used by SWARM_DRONE_STATUS.order (0 = no order).
    pub fn code(order: Option<Order>) -> u8 {
        match order {
            None => 0,
            Some(Order::GoTo(_)) => 1,
            Some(Order::Hold(_)) => 2,
            Some(Order::ReturnHome) => 3,
            Some(Order::Manual { .. }) => 4,
        }
    }
}

/// Orders (signed commands and sticks) and revocations checked per tick at most, on top of
/// sightings: the sticks come ten times a second, orders seldom, so one a tick keeps up. Each
/// copy of one already checked costs nothing; what doesn't get checked is dropped, and the
/// copy another relay passes on, or the ground station's repeat, is checked instead. A drone
/// flooding forged orders can use this up (and so hold up real ones) but nothing more: the cost
/// on the board stays at 6 ms a tick.
pub(crate) const OPERATOR_CHECKS_PER_TICK: u8 = 1;
/// How old an order may be, on the mission clock, and still be taken (the ground station
/// repeats an unacknowledged one for a few seconds)...
pub(crate) const ORDER_MAX_AGE_MS: u32 = 10_000;
/// ...and how far ahead of our clock (mission clocks agree only so well).
pub(crate) const ORDER_CLOCK_SLACK_MS: u32 = 1_000;
/// Drones revoked in one mission that a drone keeps track of.
pub(crate) const MAX_REVOKED: usize = 8;

impl Brain {
    /// Flying an operator order: straight there, easing in over the last couple of meters.
    /// Collision avoidance still applies on top (it's added in `on_tick`), so the operator
    /// can't fly a drone into its neighbors.
    pub(crate) fn follow(&mut self, order: Order, pos: Vec2, now: u32) -> Vec2 {
        self.assigned = None;
        self.mode = Mode::Operator;
        let goal = match order {
            Order::GoTo(p) | Order::Hold(p) => p,
            Order::ReturnHome => self.cfg.home,
            // The sticks: straight to velocity (and nothing at all once their signal is lost —
            // its autopilot brakes it to a hover; `decide` hands it back to the swarm if they
            // stay lost).
            Order::Manual { stick, at_ms } => {
                if now.saturating_sub(at_ms) > STICK_TIMEOUT_MS {
                    return Vec2::ZERO;
                }
                return clamp_length(stick, 1.0) * self.cfg.max_speed;
            }
        };
        let offset = goal - pos;
        let distance = offset.length();
        // Close enough for the base to guide it onto a free pad (see the backend's world.rs):
        // every drone back makes for the one spot otherwise, and they bunch up over it.
        if order == Order::ReturnHome && distance < HOME_LAND_M {
            self.landing = true;
        }
        if self.landing || distance < 1e-3 {
            return Vec2::ZERO;
        }
        offset * (self.cfg.max_speed * (distance / 2.0).min(1.0) / distance)
    }

    /// Operator traffic: commands for this drone (or for everyone) are carried out; commands for
    /// others, and other drones' statuses, are relayed so they reach across the swarm — the
    /// ground station's own radio only reaches the drones near the base.
    pub(crate) fn handle_operator_traffic(&mut self, pos: Vec2, inbox: &[(MavHeader, MavMessage)], now: u32) {
        // Orders for us, and revocations, are checked first: what's left of the budget goes to
        // the ones we only relay.
        self.operator_budget = OPERATOR_CHECKS_PER_TICK;
        let ours = |m: &MavMessage, me: u8| match m {
            MavMessage::SWARM_SIGNED_COMMAND(c) => c.target_system == me || c.target_system == 0,
            MavMessage::SWARM_SIGNED_STICKS(st) => st.target == me,
            MavMessage::SWARM_REVOKE(_) => true,
            _ => false,
        };
        let me = self.sysid;
        let first = inbox.iter().filter(|(_, m)| ours(m, me));
        let then = inbox.iter().filter(|(_, m)| !ours(m, me));
        for (header, msg) in first.chain(then).filter(|(_, m)| well_formed(m)) {
            match msg {
                MavMessage::SWARM_SIGNED_COMMAND(c) => self.take_order(*header, c, pos, now),
                MavMessage::SWARM_SIGNED_STICKS(st) => self.take_sticks(*header, st, now),
                MavMessage::SWARM_REVOKE(r) => self.take_revocation(*header, r, now),
                // From its author only (relays keep the author's header): a drone can't make up
                // a status for another. Nor can a revoked one keep sending its own.
                MavMessage::SWARM_DRONE_STATUS(st) if st.sysid != self.sysid && st.sysid != 0 && st.sysid == header.system_id && !self.is_revoked(st.sysid) => {
                    let at = now.saturating_sub(st.age_ms);
                    // The highest system ID it knows of, too: passed on in our own status, it
                    // reaches the whole swarm, where the statuses themselves go only so many hops.
                    // The fronts are worked out from how many drones were launched, and drones that
                    // disagreed on it would work out fronts of different widths.
                    for id in [st.sysid, st.fleet] {
                        if id < BASE_SYSID {
                            self.highest_sysid = self.highest_sysid.max(id);
                        }
                    }
                    let slot = &mut self.swarm_heard[st.sysid as usize];
                    if slot.is_none_or(|(seen, _, _)| at >= seen) {
                        *slot = Some((at, st.order == 0 && st.target_id == 0, Vec2::new(st.x, st.y)));
                        self.swarm_target[st.sysid as usize] = st.target_id;
                        // With relay chains, the whole swarm's positions are what keeps drones
                        // from searching the same ground: there it has searched too.
                        if self.cfg.relay_chains && now.saturating_sub(at) <= FREE_DRONE_FRESH_MS {
                            self.mark_searched(Vec2::new(st.x, st.y), at);
                        }
                    }
                    let newer = self.status_relayed[st.sysid as usize]
                        .is_none_or(|(seq, at)| (st.seq.wrapping_sub(seq) as i8) > 0 || now.saturating_sub(at) > 10_000);
                    if newer {
                        self.status_relayed[st.sysid as usize] = Some((st.seq, now));
                        if st.hops < self.cfg.status_max_hops {
                            self.hold_status(*header, st, now);
                        }
                    } else if let Some(p) = self.status_pending.iter_mut().flatten().find(|p| p.status.sysid == st.sysid && p.status.seq == st.seq) {
                        // Someone else relayed what we're holding.
                        p.copies = p.copies.saturating_add(1);
                    }
                }
                _ => {}
            }
        }
        self.relay_due_statuses(now);
    }

    /// Whether an order given at `issued_ms` (mission clock) is recent enough to take. Our
    /// clock is the mission's too (`SensorReading::time_ms`).
    pub(crate) fn fresh(issued_ms: u32, now: u32) -> bool {
        issued_ms <= now.saturating_add(ORDER_CLOCK_SLACK_MS) && now.saturating_sub(issued_ms) <= ORDER_MAX_AGE_MS
    }

    /// Takes one signature check from this tick's operator budget, if any is left.
    pub(crate) fn operator_check(&mut self) -> bool {
        if self.operator_budget == 0 {
            return false;
        }
        self.operator_budget -= 1;
        self.checks_done += 1;
        true
    }

    /// An order from the ground station (see `auth`): only the operator's signature makes it
    /// one, whatever the header says. Checked before it's relayed too, so nobody's forgeries
    /// get carried across the swarm.
    pub(crate) fn take_order(&mut self, header: MavHeader, c: &SWARM_SIGNED_COMMAND_DATA, pos: Vec2, now: u32) {
        let target = c.target_system;
        let broadcast = target == 0;
        if c.mission != self.keys.mission || !Self::fresh(c.issued_ms, now) || (broadcast && !BROADCAST_COMMANDS.contains(&c.command)) {
            return;
        }
        let msg = MavMessage::SWARM_SIGNED_COMMAND(c.clone());
        let print = fingerprint(&msg);
        let newest = self.orders_seen[target as usize];
        // A repeat of the newest order checked for that drone: the ground station repeats an
        // order until it's acknowledged, so it's relayed again (once in a while), but taken only
        // once. Anything older was overtaken, or is being played back.
        if let Some((issued, known)) = newest {
            if c.issued_ms < issued || (c.issued_ms == issued && known != print) {
                return;
            }
            if c.issued_ms == issued {
                if target != self.sysid && self.first_time(&msg, now) {
                    self.relay(header, msg);
                }
                return;
            }
        }
        if !self.operator_check() {
            return;
        }
        if !auth::verify(&self.keys.operator, &auth::command_bytes(c), &c.signature) {
            self.orders_forged += 1;
            return;
        }
        self.orders_seen[target as usize] = Some((c.issued_ms, print));
        let cmd = auth::command_of(c);
        if target == self.sysid {
            let result = self.execute(&cmd, pos);
            self.acknowledge(cmd.command, result);
        } else {
            // Nobody acknowledges a broadcast.
            if broadcast {
                self.execute(&cmd, pos);
            }
            if self.first_time(&msg, now) {
                self.relay(header, msg);
            }
        }
    }

    /// Stick input from the ground station: signed like an order, and only newer than any
    /// checked for that drone counts, here and for relaying, so the last frame can't circle round
    /// the relays and keep a drone "hearing" sticks after the operator let go.
    pub(crate) fn take_sticks(&mut self, header: MavHeader, st: &SWARM_SIGNED_STICKS_DATA, now: u32) {
        let target = st.target as usize;
        let newer = self.sticks_seen[target].is_none_or(|seen| st.issued_ms > seen);
        if st.mission != self.keys.mission || st.target == 0 || !Self::fresh(st.issued_ms, now) || !newer || !self.operator_check() {
            return;
        }
        if !auth::verify(&self.keys.operator, &auth::sticks_bytes(st), &st.signature) {
            self.orders_forged += 1;
            return;
        }
        self.sticks_seen[target] = Some(st.issued_ms);
        if st.target == self.sysid {
            // Stick axes are -1000..1000: x forward (north here: no heading to be relative
            // to), y right (east).
            let stick = Vec2::new(st.y as f32 / 1000.0, st.x as f32 / 1000.0);
            if !matches!(self.order, Some(Order::Manual { .. })) {
                self.urgent_status = true;
            }
            self.order = Some(Order::Manual { stick, at_ms: now });
            self.landing = false;
        } else {
            self.relay(header, MavMessage::SWARM_SIGNED_STICKS(st.clone()));
        }
    }

    /// The base revoking a drone's key (see `auth::Authority::revoke`): from now on its
    /// sightings, statuses, beacons and certificate are ignored. If it's us, we come home.
    pub(crate) fn take_revocation(&mut self, header: MavHeader, r: &SWARM_REVOKE_DATA, now: u32) {
        if r.mission != self.keys.mission || r.sysid == 0 {
            return;
        }
        let msg = MavMessage::SWARM_REVOKE(r.clone());
        let print = fingerprint(&msg);
        let known = self.revoked.iter().flatten().find(|(id, _)| *id == r.sysid).copied();
        match known {
            // The base repeats it: passed on again now and then, for drones that missed it.
            Some((_, p)) if p == print => {
                if self.first_time(&msg, now) {
                    self.relay(header, msg);
                }
                return;
            }
            Some(_) => return,
            None => {}
        }
        if !self.operator_check() {
            return;
        }
        if !auth::revocation_valid(r, self.keys.mission, &self.keys.authority) {
            self.orders_forged += 1;
            return;
        }
        // Past `MAX_REVOKED`, the oldest is forgotten (its certificate stays forgotten, so its
        // sightings still don't count, until a relay passes it on again).
        self.revoked.rotate_right(1);
        self.revoked[0] = Some((r.sysid, print));
        let sysid = r.sysid as usize;
        self.certificates[sysid] = None;
        self.neighbors[sysid] = None;
        self.swarm_heard[sysid] = None;
        self.swarm_target[sysid] = 0;
        for t in self.tracks.iter_mut().flatten() {
            t.forget_witness(r.sysid);
        }
        self.backlog.iter_mut().filter(|w| w.as_ref().is_some_and(|(_, s, _)| s.observer_sysid == r.sysid)).for_each(|w| *w = None);
        if r.sysid == self.sysid {
            self.order = Some(Order::ReturnHome);
            self.urgent_status = true;
        }
        if self.first_time(&msg, now) {
            self.relay(header, msg);
        }
    }

    pub(crate) fn is_revoked(&self, sysid: u8) -> bool {
        self.revoked.iter().flatten().any(|(id, _)| *id == sysid)
    }

    /// Carries out an operator command, returning how it went.
    pub(crate) fn execute(&mut self, cmd: &COMMAND_LONG_DATA, pos: Vec2) -> MavResult {
        // The parameter each of these goes by has to be a number (see `well_formed`).
        let needs = match cmd.command {
            MavCmd::MAV_CMD_DO_REPOSITION => Vec2::new(cmd.param5, cmd.param6).is_finite(),
            MavCmd::MAV_CMD_SWARM_SET_MODE | MavCmd::MAV_CMD_SWARM_SET_RESPONDERS | MavCmd::MAV_CMD_SWARM_CANCEL_TARGET => cmd.param1.is_finite(),
            _ => true,
        };
        if !needs {
            return MavResult::MAV_RESULT_DENIED;
        }
        match cmd.command {
            // No GPS here: x/y in the local field frame, in place of latitude/longitude. Kept
            // inside the field: an order to fly off it is one to its edge.
            MavCmd::MAV_CMD_DO_REPOSITION => {
                let edge = Vec2::new((self.cfg.field_half.x - GOTO_MARGIN_M).max(0.0), (self.cfg.field_half.y - GOTO_MARGIN_M).max(0.0));
                let to = Vec2::new(cmd.param5.clamp(-edge.x, edge.x), cmd.param6.clamp(-edge.y, edge.y));
                self.order = Some(Order::GoTo(to));
            }
            MavCmd::MAV_CMD_NAV_LOITER_UNLIM => self.order = Some(Order::Hold(pos)),
            MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => self.order = Some(Order::ReturnHome),
            MavCmd::MAV_CMD_MISSION_START => {
                self.order = None;
                self.landing = false;
            }
            MavCmd::MAV_CMD_SWARM_SET_MODE => {
                self.cfg = self.cfg.with_mode(SwarmMode::from_u8(cmd.param1 as u8));
            }
            MavCmd::MAV_CMD_SWARM_SET_RESPONDERS => {
                self.cfg.responders = (cmd.param1 as u8).clamp(1, 8);
                // param2: adapt to how many targets there are (1) or not (0).
                if cmd.param2.is_finite() {
                    self.cfg.adaptive_responders = cmd.param2 >= 0.5;
                }
            }
            MavCmd::MAV_CMD_SWARM_CANCEL_TARGET => {
                let id = cmd.param1 as TargetId;
                if !self.cancelled_ids.contains(&id) {
                    self.cancelled_ids.rotate_right(1);
                    self.cancelled_ids[0] = id;
                }
                for t in self.tracks.iter_mut().flatten().filter(|t| t.id == id) {
                    t.cancelled = true;
                }
                // Spread the word with the next sightings rather than waiting for the beacon.
                self.urgent_beacon = true;
            }
            _ => return MavResult::MAV_RESULT_UNSUPPORTED,
        }
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Standard COMMAND_ACK (heard directly if the ground station is in range) plus the same
    /// acknowledgement in our next status, which is relayed all the way back.
    pub(crate) fn acknowledge(&mut self, command: MavCmd, result: MavResult) {
        self.last_ack = Some((command as u16, result));
        self.urgent_status = true;
        self.queue(MavMessage::COMMAND_ACK(COMMAND_ACK_DATA { command, result }));
    }
}
