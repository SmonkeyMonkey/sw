//! A swarm drone's brain: everything one drone decides on its own, from its sensor reading and
//! the MAVLink messages it heard. It knows nothing about tokio, UDP, OpenTelemetry, or the
//! simulation — each tick it gets only its own sensor reading and the MAVLink messages that
//! arrived since the last tick. That boundary is what lets the exact same code run inside the
//! PC simulator (backend/src/agent.rs) and on the STM32 board (firmware/), where UART carries
//! the same MAVLink frames and the board never needs to know the other drones are simulated.
//!
//! `no_std` and allocation-free: fixed-size tables instead of maps, its own tiny PRNG instead
//! of `rand`, `libm` for square roots.
//!
//! A tick has two phases so tracing can stay outside the brain: `on_tick` updates state and may
//! report a `TraceEvent`; the runner turns that into an OpenTelemetry span and hands the span's
//! IDs back via `attach_span`; only then does `outgoing` build the frames, so the very first
//! broadcast of a new sighting already carries its trace context.

#![no_std]

pub mod auth;
pub mod hil;
pub mod radio;
mod vec2;
mod config;
mod tracks;
mod nav;
mod respond;
mod outgoing;
mod relay;
mod operator;
mod avoid;
mod battery;
mod search;
mod formation;

use mavlink_swarm::mavlink_core::MavHeader;
use mavlink_swarm::mavlink_core::{Message, MavlinkVersion};
use mavlink_swarm::{
    EstimatorStatusFlags, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult, MavState, MavType, COMMAND_ACK_DATA,
    COMMAND_LONG_DATA, ESTIMATOR_STATUS_DATA, HEARTBEAT_DATA, LOCAL_POSITION_NED_DATA, SWARM_DRONE_STATUS_DATA,
    SWARM_KEY_CERT_DATA, SWARM_REVOKE_DATA, SWARM_SIGNED_COMMAND_DATA, SWARM_SIGNED_STICKS_DATA, SWARM_TARGET_SIGHTING_DATA,
};
use ed25519_dalek::SigningKey;

pub use auth::Keys;
pub use vec2::Vec2;
pub use config::*;
use formation::*;
pub use tracks::*;
use battery::*;
pub use outgoing::*;
pub use operator::*;
use relay::*;
use respond::*;
pub use search::*;
pub use nav::*;

/// MAV_COMP_ID_AUTOPILOT1: the brain plays the role of the drone's autopilot component.
pub const COMPONENT_ID: u8 = 1;

/// The operator's ground station: MAVLink's conventional system ID for a GCS.
pub const GCS_SYSID: u8 = 255;

/// The launch base. Its radio beacons whether or not the operator's ground station is running,
/// so the drones keep their link to the base (and with it the leash, relay chains and the sweep)
/// either way: hearing it is hearing the base, like hearing the ground station.
pub const BASE_SYSID: u8 = 250;

/// Radio frames a drone keeps between two ticks; the rest are dropped, like lost packets. The
/// same for the simulated drones and the board, so they hear the same. Sized for a crowd: most
/// arrive at launch, with everyone still bunched up around the base.
pub const MAX_INBOX: usize = 96;

/// Most targets one drone's sensor reports in a single reading.
pub const MAX_SEEN: usize = 4;

/// Everything one drone's own sensors report on a given tick — the only view of the world its
/// brain gets besides radio messages. It deliberately does *not* include other drones'
/// positions: those are only learned from their MAVLink broadcasts.
#[derive(Clone, Copy, Debug)]
pub struct SensorReading {
    /// Milliseconds since the swarm started. Shared clock across drones for simplicity — real
    /// drones would use GPS time for the same purpose.
    pub time_ms: u32,
    /// The GPS fix, if there is one (`gps_fix`). Only one input to where the drone thinks it
    /// is: the brain checks it against its neighbors and keeps going without it (see
    /// `Brain::update_position`).
    pub own_pos: Vec2,
    pub gps_fix: bool,
    /// Ground velocity from the drone's own sensors (IMU, optical flow), GPS or not — what the
    /// airframe is actually doing, which with real inertia isn't the same as what the brain
    /// last asked for. A little biased, so position integrated from it drifts.
    pub own_velocity: Vec2,
    /// Targets within sensor range this tick, *relative to the drone* (a camera knows where a
    /// target is from the drone, not on the map). Positions only: a sensor sees *a* target,
    /// not which one — telling them apart over time is the brain's job (see `Track`).
    pub targets_seen: [Option<Vec2>; MAX_SEEN],
    /// Other drones the onboard proximity sensing (cameras, rangefinders, UWB ranging) picks up
    /// close by: (system ID, position and velocity *relative to this drone*) — UWB ranging
    /// knows who it ranged. No GPS in it. `None` if the drone has no such sensing; then
    /// collision avoidance falls back on the positions neighbors broadcast, GPS error and all.
    pub nearby: Option<[Option<(u8, Vec2, Vec2)>; MAX_NEARBY]>,
    /// The battery's state of charge, 0 (empty) to 1 (full); NaN if the drone doesn't know it.
    /// Knowing it, the drone goes back to the base to charge in time (see `Brain::battery_low`).
    pub battery: f32,
}

/// Most drones the proximity sensing reports at once.
pub const MAX_NEARBY: usize = 6;

impl SensorReading {
    pub fn targets(&self) -> impl Iterator<Item = Vec2> + '_ {
        self.targets_seen.iter().flatten().copied()
    }

    /// With anything not a finite number taken out: a position fix that isn't one is no fix, a
    /// velocity that isn't is none, and a target or drone sensed at one wasn't sensed.
    fn sanitized(&self) -> SensorReading {
        let mut r = *self;
        r.gps_fix &= r.own_pos.is_finite();
        if !r.gps_fix {
            r.own_pos = Vec2::ZERO;
        }
        if !r.own_velocity.is_finite() {
            r.own_velocity = Vec2::ZERO;
        }
        for seen in r.targets_seen.iter_mut() {
            *seen = seen.filter(|p| p.is_finite());
        }
        if let Some(nearby) = r.nearby.as_mut() {
            for n in nearby.iter_mut() {
                *n = n.filter(|(_, p, v)| p.is_finite() && v.is_finite());
            }
        }
        if r.battery.is_finite() {
            r.battery = r.battery.clamp(0.0, 1.0);
        }
        r
    }
}

/// Whether every number in a frame from the radio that the brain reads is a finite one (and a
/// claimed position accuracy a positive one). A frame that fails is dropped whole, like one
/// lost to the radio. Commands are checked one by one instead (see `Brain::execute`): MAVLink
/// uses NaN in a command parameter for "not given".
fn well_formed(msg: &MavMessage) -> bool {
    match msg {
        MavMessage::SWARM_TARGET_SIGHTING(s) => Vec2::new(s.x, s.y).is_finite() && Vec2::new(s.vx, s.vy).is_finite(),
        MavMessage::LOCAL_POSITION_NED(p) => Vec2::new(p.x, p.y).is_finite() && Vec2::new(p.vx, p.vy).is_finite(),
        MavMessage::ESTIMATOR_STATUS(e) => e.pos_horiz_accuracy.is_finite() && e.pos_horiz_accuracy > 0.0,
        MavMessage::SWARM_DRONE_STATUS(st) => Vec2::new(st.x, st.y).is_finite(),
        _ => true,
    }
}

/// A frame's content in 32 bits (FNV-1a), to tell a copy from something new.
fn fingerprint(msg: &MavMessage) -> u32 {
    let mut bytes = [0u8; 280];
    let len = msg.ser(MavlinkVersion::V2, &mut bytes);
    let mut hash = 0x811C_9DC5u32 ^ msg.message_id();
    for &b in &bytes[..len] {
        hash = (hash ^ b as u32).wrapping_mul(0x0100_0193);
    }
    hash
}

/// "Never searched", as a time: long enough ago that such a cell outranks any searched one.
const NEVER: LocalMs = -120_000;

/// Where a lost target was last seen: a minute staler still than "never searched", which at
/// `travel_cost_s_per_m` outweighs 30 m of travel — so a free drone anywhere on the field goes
/// there before plain unsearched ground.
const LAST_SEEN_HINT: LocalMs = NEVER - 60_000;

/// What a drone is doing about the target, for telemetry and the viewer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// No sighting, or an unconfirmed one too far away to check.
    Searching,
    /// Flying to an unconfirmed sighting to see it with its own sensors.
    Investigating,
    /// Target confirmed by quorum (or seen firsthand): holding the standoff ring around it.
    Converging,
    /// Following an operator order, which overrides everything above.
    Operator,
}

impl Mode {
    /// Wire encoding used by SWARM_HIL_COMMAND.mode and SWARM_DRONE_STATUS.mode.
    pub fn to_u8(self) -> u8 {
        match self {
            Mode::Searching => 0,
            Mode::Investigating => 1,
            Mode::Converging => 2,
            Mode::Operator => 3,
        }
    }

    pub fn from_u8(value: u8) -> Mode {
        match value {
            1 => Mode::Investigating,
            2 => Mode::Converging,
            3 => Mode::Operator,
            _ => Mode::Searching,
        }
    }
}

/// With adaptive responders, a drone heard of (by its status) within this long still counts as
/// part of the swarm: long, since a drone off on its own hears nobody for a while.
const SWARM_MEMORY_MS: u32 = 30_000;

/// A free drone heard of (by its status) within this long is about to take a target that's
/// short: no ring gives one up.
const FREE_DRONE_FRESH_MS: u32 = 3_000;

/// What this drone last heard from another drone.
#[derive(Clone, Copy)]
struct Neighbor {
    /// Last LOCAL_POSITION_NED: (position, velocity, when it arrived).
    position: Option<(Vec2, Vec2, u32)>,
    /// When its last HEARTBEAT arrived — or when we first heard it at all, so a neighbor whose
    /// heartbeats never get through is still eventually declared lost.
    heartbeat_at_ms: u32,
    /// The target it last said it's responding to (HEARTBEAT custom_mode), if any.
    assigned: Option<TargetId>,
    /// Whether it says it has settled in on `assigned` (see `Brain::settled`).
    settled: bool,
    /// How old its freshest news of the ground station was when it sent its last HEARTBEAT
    /// (None: it has no route to it).
    link_age_ms: Option<u32>,
    /// The neighbor its route goes through (0: the ground station itself, or no route).
    route_parent: u8,
    /// It says it's part of a relay chain, and whether its leash is holding it back (see
    /// `ANNOUNCE_CHAIN`, `ANNOUNCE_STRETCHED`).
    chain: bool,
    stretched: bool,
    /// How sure it says it is of its own position (ESTIMATOR_STATUS), m.
    pos_accuracy: Option<f32>,
}

impl Neighbor {
    fn last_heard_ms(&self) -> u32 {
        self.position.map_or(self.heartbeat_at_ms, |(_, _, t)| {
            t.max(self.heartbeat_at_ms)
        })
    }
}

/// A moment on this drone's own clock (ms since it booted). Signed, because a relayed
/// observation can be older than this drone's uptime — a drone that just powered on still has
/// to tell a 2 s-old report from a 100 ms-old one.
type LocalMs = i64;

/// How long ago `at` was, as of `now`.
fn age_at(now: u32, at: LocalMs) -> u32 {
    (now as LocalMs - at).clamp(0, u32::MAX as LocalMs) as u32
}

/// Identifies one target swarm-wide: the sysid of the first drone that saw it, then that
/// drone's own counter. Duplicate tracks of one target (two drones spotting it independently)
/// merge into the lowest ID, so the whole swarm ends up calling it the same thing — which is
/// what lets an operator refer to "target 0x0301".
pub type TargetId = u16;

/// One slot per possible MAVLink system ID, so lookup needs no allocation (keeps the brain
/// portable to the board). 256 * 32 bytes = 8 KB, fine even on the STM32's 512 KB of RAM.
const MAX_SYSIDS: usize = 256;

/// At most this many lost neighbors are reported per tick; any more are reported next tick.
/// A fixed-size array keeps `TickOutput` allocation-free.
pub const MAX_LOST_PER_TICK: usize = 4;

/// A neighbor this drone just declared lost after its heartbeats stopped.
#[derive(Clone, Copy, Debug)]
pub struct LostNeighbor {
    pub sysid: u8,
    pub silent_ms: u32,
}

/// Something worth recording as a span: the start of a sighting's propagation, or one hop of it.
#[derive(Clone, Copy, Debug)]
pub enum TraceEvent {
    /// This drone just spotted a target nobody had told it about — the root of a new
    /// propagation trace.
    Spotted,
    /// This drone just learned of a target it had no knowledge of — one hop in its trace.
    Adopted {
        trace_id: [u8; 16],
        parent_span_id: [u8; 8],
        observer_sysid: u8,
        hops: u8,
        age_ms: u32,
    },
}

pub struct TickOutput {
    pub velocity: Vec2,
    /// At most one per tick; a second new target in the same tick just goes untraced.
    pub event: Option<TraceEvent>,
    pub lost: [Option<LostNeighbor>; MAX_LOST_PER_TICK],
    /// The observer whose report this drone just checked in person and found to be wrong.
    pub refuted: Option<u8>,
    /// Back at base after a return-to-launch: the airframe should land now.
    pub land: bool,
    /// ...to charge: it went home low on charge, on its own (not sent home by the operator), and
    /// a charged airframe can take over from it.
    pub to_charge: bool,
    /// Where the drone thinks it is, and where that came from.
    pub position: Vec2,
    pub nav: Nav,
}

/// A tick of the simulator and the board (`SensorReading` every 50 ms).
const TICK_MS: u32 = 50;

pub struct Brain {
    sysid: u8,
    cfg: BrainConfig,
    keys: Keys,
    /// `keys.secret`, ready to sign with (deriving it costs as much as a signature).
    signer: SigningKey,
    /// Per drone, from its certificate: its public key and the certificate, to check its
    /// sightings and to pass the certificate on with them.
    certificates: [Option<([u8; 32], [u8; 64])>; MAX_SYSIDS],
    /// Per drone: when we last sent its certificate.
    certificate_sent: [Option<u32>; MAX_SYSIDS],
    /// Per observer: its newest observation we've taken, on its clock and on ours (see
    /// `seen_at`).
    observer_clock: [Option<(u32, LocalMs)>; MAX_SYSIDS],
    verified: [Option<Attested>; VERIFIED_MEMORY],
    next_verified: usize,
    /// Signature checks left this tick.
    verify_budget: u8,
    /// (header, sighting, when it arrived)
    backlog: [Option<(MavHeader, SWARM_TARGET_SIGHTING_DATA, u32)>; BACKLOG],
    /// Sightings dropped for a bad signature, and for waiting too long to be checked.
    forged: u32,
    unchecked_dropped: u32,
    /// Signatures checked (certificates included) and made, since the start.
    checks_done: u32,
    signatures_made: u32,
    tracks: [Option<Track>; MAX_TRACKS],
    /// Low byte of the next `TargetId` this drone hands out.
    next_track_number: u8,
    /// The target this tick's `TraceEvent` is about, for `attach_span`.
    event_track: Option<TargetId>,
    /// The target this drone is responding to (investigating or converging on), if any.
    assigned: Option<TargetId>,
    /// When it first got within `SETTLE_RADIUS` of `assigned` (None: not yet, or no target).
    near_target_since_ms: Option<u32>,
    /// What the last HEARTBEAT announced (see `announcement`).
    announced: u32,
    wander_heading: Vec2,
    rng: XorShift32,
    next_beacon_ms: u32,
    /// The heartbeat that carries the link's age while keeping the link: at the unstretched
    /// beacon period (see `beacon_period`), or the age would grow by the stretch every hop.
    next_link_beat_ms: u32,
    /// This tick's beacon period, stretched in a crowd (see `beacon_period`). Neighbors in the
    /// same crowd stretch theirs about as much, so it's also how stale their positions get.
    beacon_now_ms: u32,
    next_heartbeat_ms: u32,
    /// Set when there's news worth broadcasting before the next scheduled beacon.
    urgent_beacon: bool,
    last_pos: Vec2,
    /// Measured ground velocity from the last sensor reading.
    last_velocity: Vec2,
    /// Drones the proximity sensing picked up this tick (sysid, relative position, relative
    /// velocity).
    nearby: Option<[Option<(u8, Vec2, Vec2)>; MAX_NEARBY]>,
    /// Where we think we are, how sure we are of it (m), and where that came from.
    estimate: Option<Vec2>,
    pos_accuracy: f32,
    nav: Nav,
    /// GPS that disagreed with the neighbors is ignored until then.
    gps_rejected_until: u32,
    last_tick_ms: Option<u32>,
    neighbors: [Option<Neighbor>; MAX_SYSIDS],
    /// The most recent position report from any neighbor. Kept separately from `neighbors`
    /// because lost neighbors get dropped from the table, but a drone that has lost everyone
    /// still needs to remember where it last heard the swarm in order to fly back to it.
    last_contact: Option<(Vec2, Vec2, u32)>,
    /// Per observer sysid: ignore its reports until this time (0 = trusted).
    distrusted_until: [u32; MAX_SYSIDS],
    mode: Mode,
    /// When we last heard the ground station directly.
    gcs_heard_at: Option<u32>,
    /// Our route to the ground station, recomputed every tick: how old our freshest news of it
    /// is, and the neighbor that news came through (0: we hear the ground station ourselves).
    route: Option<(u32, u8)>,
    /// When we last had a route (None: never had one).
    route_at: Option<u32>,
    /// Our target is within reach of our anchors, so responding to it needn't cost the link.
    target_in_reach: bool,
    /// The leash held this drone back this tick (it wanted to be farther out).
    stretched: bool,
    /// Relaying for a drone farther out this tick (see `relay_duty`), or holding a sweep node
    /// of the formation (see `formation_spot`).
    relaying: bool,
    on_sweep: bool,
    /// Throttles extra heartbeats sent because our announcement changed.
    last_heartbeat_ms: u32,
    /// The operator's current order, if any: overrides everything the drone would do itself.
    order: Option<Order>,
    /// Back home after a return-to-launch: landing.
    landing: bool,
    /// The battery's charge as last read, if the drone knows it; one reading and when (ms), to
    /// work out how fast it's going down from; and that rate (charge a second).
    battery: Option<f32>,
    battery_sample: Option<(f32, u32)>,
    drain_per_s: Option<f32>,
    /// Going back to the base to charge (see `battery_low`): a return-to-launch it gave itself.
    recharging: bool,
    /// The target it's on only until a drone it's theirs to comes: one it saw with nobody on
    /// it, in another front's stretch or as its front's reserve (see `is_responder`).
    custody: Option<TargetId>,
    /// Targets the operator cancelled, for tracks of them that show up later.
    cancelled_ids: [TargetId; 8],
    /// Frames to send next `outgoing`: relays for others and our own acknowledgements.
    outbox: [Option<Frame>; OUTBOX],
    /// Frames that didn't fit in `outbox` and were dropped, since the start.
    outbox_dropped: u32,
    /// Fingerprints of recently relayed frames and when they were relayed.
    recent_relays: [(u32, u32); RECENT_RELAYS],
    next_relay_slot: usize,
    /// The formation's roster as this drone keeps it (see `update_roster`): the system IDs in
    /// it, in order, and since when what it hears differs from it.
    roster: [u8; MAX_ROSTER],
    roster_len: usize,
    roster_differs_since: Option<u32>,
    /// The share of drones per target (see `update_share`): for joining one, for staying on
    /// one, and since when the share has been below the one for staying.
    join_share: u8,
    stay_share: u8,
    share_drop_since: Option<u32>,
    /// Since when this drone has wanted to leave its ring for a target that's short, and the
    /// ring it last left that way, and when (see `consider_leaving_for_a_short_target`).
    leave_since: Option<u32>,
    left: Option<(TargetId, u32)>,
    /// Per drone, anywhere in the swarm (from its status, relayed): when it was last heard of,
    /// whether it was free then (on no target, no operator order), and where it said it was.
    swarm_heard: [Option<(u32, bool, Vec2)>; MAX_SYSIDS],
    /// The highest system ID heard of, ours included: how many drones have been launched.
    highest_sysid: u8,
    /// This drone's front as it has it planned (see `update_front`): which one, of how many,
    /// its loop, and where on the loop it is when. None while it's in no front.
    front: Option<FrontPlan>,
    /// Per drone, from the same status: the target it said it was on (0: none).
    swarm_target: [TargetId; MAX_SYSIDS],
    /// Per drone: when the newest stick input checked for it was given (its `issued_ms`).
    sticks_seen: [Option<u32>; MAX_SYSIDS],
    /// Per drone (0: everyone): the newest order checked for it, when it was given and the
    /// frame's fingerprint (see `fingerprint`), so its repeats are relayed without a check.
    orders_seen: [Option<(u32, u32)>; MAX_SYSIDS],
    /// Order and revocation checks left this tick (see `OPERATOR_CHECKS_PER_TICK`).
    operator_budget: u8,
    /// Orders and revocations dropped for a bad signature, since the start.
    orders_forged: u32,
    /// Drones whose keys the base revoked, and the revocation's fingerprint (see `revoke`).
    revoked: [Option<(u8, u32)>; MAX_REVOKED],
    /// Per drone: the newest status heard for it (seq), and when.
    status_relayed: [Option<(u8, u32)>; MAX_SYSIDS],
    /// Statuses waiting to be relayed (see `STATUS_COPIES`).
    status_pending: [Option<PendingStatus>; STATUS_PENDING],
    status_seq: u8,
    next_status_ms: u32,
    /// Set when there's news for the ground station before the next scheduled status.
    urgent_status: bool,
    /// Whether the last status said the sticks were lost, so losing (or regaining) them is
    /// reported at once rather than at the next scheduled status.
    reported_sticks_lost: bool,
    /// The last operator command handled and how it went, reported in our status.
    last_ack: Option<(u16, MavResult)>,
    /// Coverage map: when each cell was last flown over, by us or a neighbor, `[x][y]`.
    searched_at: [[LocalMs; GRID]; GRID],
    /// The cell this drone is heading for while searching.
    search_goal: Option<(usize, usize)>,
}

impl Brain {
    /// `sysid` is the drone's MAVLink system ID (1..=255; 0 is reserved for broadcast), and
    /// `keys` what the launch base issued it for this mission (see `auth`).
    pub fn new(sysid: u8, cfg: BrainConfig, seed: u32, keys: Keys) -> Self {
        let mut rng = XorShift32::new(seed);
        let wander_heading = rng.random_unit_vector();
        let signer = SigningKey::from_bytes(&keys.secret);
        let mut certificates = [None; MAX_SYSIDS];
        certificates[sysid as usize] = Some((signer.verifying_key().to_bytes(), keys.certificate));
        Self {
            sysid,
            cfg,
            keys,
            signer,
            certificates,
            certificate_sent: [None; MAX_SYSIDS],
            observer_clock: [None; MAX_SYSIDS],
            verified: [None; VERIFIED_MEMORY],
            next_verified: 0,
            verify_budget: 0,
            backlog: [const { None }; BACKLOG],
            forged: 0,
            unchecked_dropped: 0,
            checks_done: 0,
            signatures_made: 0,
            tracks: [None; MAX_TRACKS],
            // From anywhere: a drone flying as one that went to charge starts afresh, and its
            // first targets' names would otherwise be the ones that drone gave its first.
            next_track_number: (seed.wrapping_sub(1) % 255) as u8 + 1,
            event_track: None,
            assigned: None,
            near_target_since_ms: None,
            announced: 0,
            wander_heading,
            rng,
            // Stagger beacons across drones so they don't all transmit in the same tick.
            next_beacon_ms: (sysid as u32 * 37) % cfg.beacon_period_ms,
            next_link_beat_ms: (sysid as u32 * 37) % cfg.beacon_period_ms,
            beacon_now_ms: cfg.beacon_period_ms,
            // Heartbeat right away, so the network learns about this drone immediately.
            next_heartbeat_ms: 0,
            urgent_beacon: false,
            last_pos: Vec2::ZERO,
            last_velocity: Vec2::ZERO,
            nearby: None,
            estimate: None,
            pos_accuracy: GPS_ACCURACY_M,
            nav: Nav::Gps,
            gps_rejected_until: 0,
            last_tick_ms: None,
            neighbors: [None; MAX_SYSIDS],
            last_contact: None,
            distrusted_until: [0; MAX_SYSIDS],
            mode: Mode::Searching,
            searched_at: [[NEVER; GRID]; GRID],
            search_goal: None,
            gcs_heard_at: None,
            route: None,
            route_at: None,
            last_heartbeat_ms: 0,
            target_in_reach: false,
            stretched: false,
            relaying: false,
            on_sweep: false,
            order: None,
            landing: false,
            battery: None,
            battery_sample: None,
            drain_per_s: None,
            recharging: false,
            custody: None,
            cancelled_ids: [0; 8],
            outbox: [const { None }; OUTBOX],
            outbox_dropped: 0,
            recent_relays: [(0, 0); RECENT_RELAYS],
            next_relay_slot: 0,
            status_relayed: [None; MAX_SYSIDS],
            status_pending: [const { None }; STATUS_PENDING],
            sticks_seen: [None; MAX_SYSIDS],
            orders_seen: [None; MAX_SYSIDS],
            operator_budget: 0,
            orders_forged: 0,
            revoked: [None; MAX_REVOKED],
            swarm_heard: [None; MAX_SYSIDS],
            swarm_target: [0; MAX_SYSIDS],
            highest_sysid: sysid,
            front: None,
            roster: [0; MAX_ROSTER],
            roster_len: 0,
            roster_differs_since: None,
            join_share: cfg.responders,
            stay_share: cfg.responders,
            share_drop_since: None,
            leave_since: None,
            left: None,
            status_seq: 0,
            // Staggered like beacons.
            next_status_ms: (sysid as u32 * 83) % cfg.status_period_ms,
            urgent_status: false,
            reported_sticks_lost: false,
            last_ack: None,
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// The target this drone is currently responding to.
    pub fn assigned_target(&self) -> Option<TargetId> {
        self.assigned
    }

    /// The operator order it's following, if any.
    pub fn order(&self) -> Option<Order> {
        self.order
    }

    /// Whether manual control has lost its stick signal (and the drone is hovering).
    fn sticks_lost(&self, now: u32) -> bool {
        matches!(self.order, Some(Order::Manual { at_ms, .. }) if now.saturating_sub(at_ms) > STICK_TIMEOUT_MS)
    }

    /// Where it thinks it is, how sure it is (m), and why.
    pub fn position(&self) -> (Vec2, f32, Nav) {
        (self.estimate.unwrap_or(self.cfg.home), self.pos_accuracy, self.nav)
    }

    pub fn config(&self) -> &BrainConfig {
        &self.cfg
    }

    pub fn sysid(&self) -> u8 {
        self.sysid
    }

    /// What this drone did this tick beyond its mode, for the viewer: relaying in a chain out to
    /// a ring, holding a node of the sweep, or neither.
    pub fn role(&self) -> Role {
        if self.relaying {
            Role::Relay
        } else if self.on_sweep {
            Role::Sweep
        } else {
            Role::None
        }
    }

    /// Relays and acknowledgements dropped so far for want of room to send them.
    pub fn outbox_dropped(&self) -> u32 {
        self.outbox_dropped
    }

    /// Sightings dropped so far because their signature was wrong (forged, or altered on the
    /// way), and because they couldn't be checked in time.
    pub fn sightings_rejected(&self) -> (u32, u32) {
        (self.forged, self.unchecked_dropped)
    }

    /// Orders, stick input and revocations dropped so far because their signature was wrong:
    /// someone other than the operator (or the base) trying to give them.
    pub fn orders_rejected(&self) -> u32 {
        self.orders_forged
    }

    /// Whether the base has revoked this drone's key (see `auth::Authority::revoke`).
    pub fn revoked(&self, sysid: u8) -> bool {
        self.is_revoked(sysid)
    }

    /// Signatures checked and made since the start: what the brain's crypto has cost, for a
    /// runner that wants to know (the fake board takes as long as the board would).
    pub fn crypto_work(&self) -> (u32, u32) {
        (self.checks_done, self.signatures_made)
    }

    /// The keys it flies with: a compromised drone's radio signs with them too.
    pub fn keys(&self) -> &Keys {
        &self.keys
    }

    /// Knows about at least one target.
    pub fn informed(&self) -> bool {
        self.tracks.iter().any(Option::is_some)
    }

    /// Switches to `mode` (see `SwarmMode`), from now on.
    pub fn set_mode(&mut self, mode: SwarmMode) {
        self.cfg = self.cfg.with_mode(mode);
    }

    /// The targets this drone knows of right now (not lost or cancelled): where it thinks each
    /// is, and whether it counts it as confirmed. For the viewer.
    pub fn known_targets(&self, now: u32) -> [Option<(Vec2, bool)>; MAX_TRACKS] {
        let mut out = [None; MAX_TRACKS];
        for (slot, t) in out.iter_mut().zip(self.tracks.iter().flatten().filter(|t| !t.cancelled && !self.lost(t, now))) {
            *slot = Some((self.predicted(t, now), self.confirmed(t, now)));
        }
        out
    }

    /// Phase 1: update what the drone knows from its sensors and inbox, and decide where to go.
    pub fn on_tick(
        &mut self,
        reading: &SensorReading,
        inbox: &[(MavHeader, MavMessage)],
    ) -> TickOutput {
        let reading = &reading.sanitized();
        let now = reading.time_ms;
        self.update_battery(reading.battery, now);
        let mut event = None;
        self.event_track = None;
        self.last_velocity = reading.own_velocity;
        self.nearby = reading.nearby;
        for t in self.tracks.iter_mut().flatten() {
            t.seen_now = false;
        }

        // Neighbor table: positions for keeping clear of them, heartbeats for knowing they're
        // still alive.
        for (header, msg) in inbox.iter().filter(|(_, m)| well_formed(m)) {
            let from = header.system_id;
            // The ground station (and the base's own radio) is a radio, not a neighbor drone:
            // nothing to fly around or wait for — but hearing it directly means we're one hop
            // from the base.
            if from == GCS_SYSID || from == BASE_SYSID {
                if matches!(msg, MavMessage::HEARTBEAT(_)) {
                    self.gcs_heard_at = Some(now);
                }
                continue;
            }
            if from == self.sysid || from == 0 || self.is_revoked(from) {
                continue;
            }
            // Relayed for someone further off, under their header (see `Frame`): they aren't
            // in radio range just because their frame is.
            let relayed = match msg {
                MavMessage::SWARM_SIGNED_COMMAND(_) | MavMessage::SWARM_SIGNED_STICKS(_) | MavMessage::SWARM_REVOKE(_) => true,
                MavMessage::SWARM_DRONE_STATUS(st) => st.hops > 0,
                _ => false,
            };
            if relayed {
                continue;
            }
            if from < BASE_SYSID {
                self.highest_sysid = self.highest_sysid.max(from);
            }
            let entry = self.neighbors[from as usize].get_or_insert(Neighbor {
                position: None,
                heartbeat_at_ms: now,
                assigned: None,
                settled: false,
                link_age_ms: None,
                route_parent: 0,
                chain: false,
                stretched: false,
                pos_accuracy: None,
            });
            match msg {
                MavMessage::HEARTBEAT(hb) => {
                    entry.heartbeat_at_ms = now;
                    let (assigned, settled, link_age_ms, parent) = unpack_announcement(hb.custom_mode);
                    entry.assigned = assigned;
                    entry.settled = settled;
                    entry.link_age_ms = link_age_ms;
                    entry.route_parent = parent;
                    entry.chain = hb.custom_mode & ANNOUNCE_CHAIN != 0;
                    entry.stretched = hb.custom_mode & ANNOUNCE_STRETCHED != 0;
                }
                MavMessage::ESTIMATOR_STATUS(e) => entry.pos_accuracy = Some(e.pos_horiz_accuracy),
                MavMessage::LOCAL_POSITION_NED(p) => {
                    let report = (Vec2::new(p.x, p.y), Vec2::new(p.vx, p.vy), now);
                    entry.position = Some(report);
                    self.last_contact = Some(report);
                    // Wherever a neighbor flies, it's looking: that spot is searched.
                    self.mark_searched(report.0, now);
                }
                _ => {}
            }
        }

        self.beacon_now_ms = self.beacon_period(now);
        // Where are we? Everything from here on goes by this, not by the GPS fix.
        let me = self.update_position(reading, now);
        self.mark_searched(me, now);

        // Own sensor first: a firsthand observation always beats whatever we had.
        for rel in reading.targets() {
            let pos = me + rel;
            // One to one: two targets seen at once are two targets, whatever the tracks say.
            let i = match self.track_near_except(pos, now, |t| t.seen_now) {
                Some(i) => i,
                None => {
                    // A target nobody told us about: the start of a new propagation trace.
                    let id = ((self.sysid as TargetId) << 8) | self.next_track_number as TargetId;
                    self.next_track_number = self.next_track_number.wrapping_add(1).max(1);
                    let slot = self.free_slot();
                    self.tracks[slot] = Some(self.new_track(id, pos, Vec2::ZERO, now as LocalMs, self.sysid, 0, [0; 16]));
                    if event.is_none() {
                        event = Some(TraceEvent::Spotted);
                        self.event_track = Some(id);
                    }
                    self.urgent_beacon = true;
                    slot
                }
            };
            let t = self.tracks[i].as_mut().expect("just found or created");
            // Velocity from consecutive sightings, smoothed; a new track (or a long gap) starts
            // from zero rather than from a stale estimate.
            t.vel = match t.own_last {
                Some((prev, at)) if now > at && now - at <= 500 => {
                    let instant = (pos - prev) * (1000.0 / (now - at) as f32);
                    t.vel + (instant - t.vel) * 0.3
                }
                _ => Vec2::ZERO,
            };
            t.own_last = Some((pos, now));
            t.hinted = false;
            let id = t.id;
            let responders = self.responders_heard(id, now) + (self.assigned == Some(id)) as u8;
            let t = self.tracks[i].as_mut().expect("just found or created");
            t.responders = responders;
            t.pos = pos;
            t.observed_at_ms = now as LocalMs;
            t.observer_sysid = self.sysid;
            t.observed_ms = now;
            t.signature = None;
            t.hops = 0;
            t.heard_from = self.sysid;
            t.seen_now = true;
            t.witness(self.sysid, now as LocalMs);
        }

        // Failure detection: a neighbor whose heartbeats stopped is dropped and reported.
        let mut lost = [None; MAX_LOST_PER_TICK];
        let mut lost_count = 0;
        for sysid in 0..MAX_SYSIDS {
            if lost_count == MAX_LOST_PER_TICK {
                break;
            }
            let Some(n) = self.neighbors[sysid] else {
                continue;
            };
            let silent_ms = now.saturating_sub(n.heartbeat_at_ms);
            if silent_ms > self.cfg.heartbeat_timeout_ms {
                self.neighbors[sysid] = None;
                lost[lost_count] = Some(LostNeighbor {
                    sysid: sysid as u8,
                    silent_ms,
                });
                lost_count += 1;
            }
        }

        // Certificates first: a sighting can only be checked against its observer's.
        self.verify_budget = MAX_VERIFY_PER_TICK;
        for (_, msg) in inbox {
            if let MavMessage::SWARM_KEY_CERT(cert) = msg {
                self.learn_certificate(cert);
            }
        }

        // Sightings from others: each is a witness for its target, and the freshest observation
        // of a target is what we act on and relay. Only what its observer signed counts (see
        // `auth`). Reports from observers we've caught being wrong are dropped entirely — not
        // checked, not counted, not adopted, not relayed. What's just arrived goes first, then
        // what's been waiting to be checked.
        let waiting = core::mem::replace(&mut self.backlog, [const { None }; BACKLOG]);
        let arrived = inbox.iter().filter(|(_, m)| well_formed(m)).filter_map(|(header, msg)| match msg {
            MavMessage::SWARM_TARGET_SIGHTING(s) => Some((*header, s.clone(), now)),
            _ => None,
        });
        for (header, theirs, received) in arrived.chain(waiting.into_iter().flatten()) {
            let observer = theirs.observer_sysid;
            if observer == 0 || self.distrusts(observer, now) || self.is_revoked(observer) {
                continue;
            }
            match self.authenticate(&theirs) {
                Authentic::Yes => self.take_sighting(&header, &theirs, now, &mut event),
                Authentic::NotYet => self.wait_for_check(header, theirs, received, now),
                Authentic::No => self.forged += 1,
            }
        }

        self.merge_duplicate_tracks(now);
        self.handle_operator_traffic(me, inbox, now);
        self.update_route(now);
        let (memory, memory_cancelled) = (self.cfg.last_seen_memory_ms, self.cfg.cancelled_forget_after_ms);
        for slot in self.tracks.iter_mut() {
            if slot.is_some_and(|t| age_at(now, t.observed_at_ms) > if t.cancelled { memory_cancelled } else { memory }) {
                *slot = None;
            }
        }
        self.hint_lost_targets(now);
        self.update_share(now);
        self.update_roster(now);
        self.update_front(now);
        self.consider_leaving_for_a_short_target(now);
        self.note_full_targets(now);

        let refuted = self.refute(me, now);
        let goal = self.decide(me, now);
        let push = self.separation(me, now) * self.cfg.max_speed;
        let velocity = clamp_length(goal + push, self.cfg.max_speed);
        let wanted = velocity;
        let velocity = self.keep_on_leash(me, velocity, now);
        // Held back by the leash (it took off most of what this drone wanted).
        self.stretched = wanted.length() > STRETCHED_MIN_SPEED && (wanted - velocity).length() > 0.5 * wanted.length();
        let velocity = clamp_length(
            self.limit_closing_speed(me, velocity, now),
            self.cfg.max_speed,
        );
        self.last_pos = me;
        // The last line of defense: whatever went wrong upstream, never a NaN to the motors.
        let velocity = if velocity.is_finite() { velocity } else { Vec2::ZERO };

        TickOutput {
            velocity,
            event,
            lost,
            refuted,
            land: self.landing,
            to_charge: self.landing && self.recharging,
            position: me,
            nav: self.nav,
        }
    }

    /// Consensus and division of labor: respond to the nearest target this drone is a
    /// responder for — converge on it if enough drones have seen it independently (or we see it
    /// ourselves), otherwise go check it if it's near. A single report thus draws only a few
    /// nearby drones, and a confirmed target only `responders` of them; everyone else keeps
    /// searching.
    fn decide(&mut self, pos: Vec2, now: u32) -> Vec2 {
        // Set again by the search if it's relaying or on the sweep.
        self.relaying = false;
        self.on_sweep = false;
        if matches!(self.order, Some(Order::Manual { at_ms, .. }) if now.saturating_sub(at_ms) > STICKS_GIVE_UP_MS) {
            // The sticks stayed lost: back to the swarm, as if released. Flown out of contact
            // (the leash doesn't hold the operator), it would otherwise hover there forever,
            // unreachable; the swarm's own rule takes it back toward the base until it's heard
            // from again. Sticks reaching it again later take it back.
            self.order = None;
            self.urgent_status = true;
        }
        // Low on charge, home to the base, whatever it was doing or ordered to: the swarm takes
        // it for gone from its status (a return-to-launch), and the base sends a charged drone
        // up in its place.
        // Landed to charge, and charged again: on whichever airframe took off as this drone (the
        // board flies as it whatever airframe does), back to the swarm.
        if self.landing && self.recharging && self.battery.is_some_and(|b| b >= RELAUNCH_CHARGE) {
            self.landing = false;
            self.recharging = false;
            self.order = None;
            self.battery_sample = None;
            self.urgent_status = true;
        }
        if !self.recharging && self.battery_low(pos) {
            self.recharging = true;
            self.urgent_status = true;
        }
        if self.recharging {
            self.order = Some(Order::ReturnHome);
        }
        if let Some(order) = self.order {
            return self.follow(order, pos, now);
        }
        if self.cfg.relay_chains {
            // Keeping the swarm one network comes first: a drone the relay spots need (free,
            // or one its ring can spare) relays rather than respond.
            if let Some(v) = self.relay_duty(pos, now) {
                if let Some(mine) = self.assigned.take() {
                    self.left = Some((mine, now));
                    self.near_target_since_ms = None;
                    self.urgent_beacon = true;
                }
                self.relaying = true;
                self.mode = Mode::Searching;
                return v;
            }
        }
        if self.link_lost(now) {
            // Back toward the base until some drone (or the ground station) is heard again.
            self.assigned = None;
            self.mode = Mode::Searching;
            let offset = self.cfg.home - pos;
            let distance = offset.length();
            return if distance < 1e-3 { Vec2::ZERO } else { offset * (self.cfg.max_speed / distance) };
        }
        // The target it's on, or else the nearest one this drone is a responder for, confirmed
        // or not: an unconfirmed one nearby is exactly what needs a second witness, and going for
        // confirmed targets first would send everyone to the first one found while the next stayed
        // unconfirmed and unattended.
        // (id, where it should be by now, confirmed, distance)
        let mut best: Option<(TargetId, Vec2, bool, f32)> = None;
        for t in self.tracks.iter().flatten().filter(|t| !t.cancelled && !self.lost(t, now)) {
            // Everything acts on where the target should be by now, not where it was seen.
            let target = self.predicted(t, now);
            let distance = (target - pos).length();
            let confirmed = self.confirmed(t, now);
            if !confirmed && distance > self.cfg.investigate_radius {
                continue;
            }
            if !self.is_responder(t.id, target, t.responders, pos, now) {
                continue;
            }
            // Commitment: the target it's already on wins over any other, however near — a
            // responder that dropped its target for every new one running past would leave the
            // first one's ring short. It still reports the new one; free drones take it.
            let ours = self.assigned == Some(t.id);
            let ours_best = best.is_some_and(|(id, ..)| self.assigned == Some(id));
            if !ours_best && (ours || best.is_none_or(|(_, _, _, bd)| distance < bd)) {
                best = Some((t.id, target, confirmed, distance));
            }
        }

        // Handed over a target it was only watching: not back to it.
        if let Some(watched) = self.custody.filter(|&c| best.is_none_or(|(id, ..)| id != c)) {
            self.left = Some((watched, now));
            self.custody = None;
        }
        let Some((id, target, confirmed, distance)) = best else {
            self.assigned = None;
            self.mode = Mode::Searching;
            return self.search(pos, now);
        };
        if self.assigned != Some(id) {
            self.near_target_since_ms = None;
            self.custody = self.outsider(target, now).then_some(id);
        }
        self.assigned = Some(id);
        if self.near_target_since_ms.is_none() && distance <= self.cfg.scan_radius + SETTLE_MARGIN {
            self.near_target_since_ms = Some(now);
        }
        let reach = self.cfg.link_range + self.cfg.standoff_radius;
        self.target_in_reach = self.route.is_some() && self.anchors_from(target, now).any(|a| (target - a).length() <= reach);
        if confirmed {
            self.mode = Mode::Converging;
            self.toward(id, pos, target, now)
        } else {
            self.mode = Mode::Investigating;
            // Straight to the reported spot (no standoff): the point is to get close enough
            // for our own sensors to confirm or refute it.
            if distance < 1e-3 {
                Vec2::ZERO
            } else {
                (target - pos) * (self.cfg.max_speed / distance)
            }
        }
    }

    /// Records the span the runner created for this tick's `TraceEvent`, so outgoing sightings
    /// of that target carry it as their parent span.
    pub fn attach_span(&mut self, trace_id: [u8; 16], span_id: [u8; 8]) {
        let Some(id) = self.event_track else { return };
        if let Some(t) = self.tracks.iter_mut().flatten().find(|t| t.id == id) {
            t.trace_id = trace_id;
            t.span_id = span_id;
        }
    }

}

/// See `Brain::role`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    None,
    Relay,
    Sweep,
}

fn clamp_length(v: Vec2, max: f32) -> Vec2 {
    let len = v.length();
    if len > max {
        v * (max / len)
    } else {
        v
    }
}

/// Tiny self-contained PRNG so `Brain` doesn't depend on the `rand` crate, whose thread RNG
/// isn't available on the board. Only needs to be good enough to make wander paths irregular.
struct XorShift32(u32);

impl XorShift32 {
    fn new(seed: u32) -> Self {
        // xorshift's state must never be zero, or it stays zero forever.
        Self(if seed == 0 { 0x9E37_79B9 } else { seed })
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Uniform in [0.0, 1.0).
    fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Rejection-sampled from the unit disk, so the direction is uniform without sin/cos
    /// (which need an extra math crate in no_std).
    fn random_unit_vector(&mut self) -> Vec2 {
        loop {
            let v = Vec2::new(self.next_f32() * 2.0 - 1.0, self.next_f32() * 2.0 - 1.0);
            let len_sq = v.x * v.x + v.y * v.y;
            if len_sq > 0.01 && len_sq <= 1.0 {
                return v.normalized();
            }
        }
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests;
