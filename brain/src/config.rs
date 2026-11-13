//! The swarm's configuration, and the plans worked out from it (fleet, sweep, fronts' paths).

use super::*;

#[derive(Clone, Copy)]
pub struct BrainConfig {
    pub max_speed: f32,
    /// The field, centered on the origin: half its width (x) and half its height (y). Past it,
    /// the brain turns its search heading back inward.
    pub field_half: Vec2,
    /// A target unseen for this long is no longer one to fly to: its responders go back to
    /// searching. It's kept as a "last seen here" hint for `last_seen_memory_ms` (see `Track`).
    pub forget_after_ms: u32,
    /// How long a target nobody has seen is remembered, and relayed, as a hint: every drone
    /// that knows of it marks the ground around where it was last seen as unsearched, so the
    /// coverage search sends drones back to look there.
    pub last_seen_memory_ms: u32,
    /// Period of the position (and sighting, if any) broadcast.
    pub beacon_period_ms: u32,
    /// Period of the MAVLink HEARTBEAT, which neighbors will use for failure detection.
    pub heartbeat_period_ms: u32,
    /// Drones hold this far from the target instead of flying onto it; combined with
    /// separation, a converged swarm settles into a ring around the target.
    pub standoff_radius: f32,
    /// Neighbors predicted to pass closer than this push the drone away from them.
    pub separation_radius: f32,
    /// How far ahead to predict each neighbor's closest approach. A real airframe can't stop
    /// instantly, so reacting only once a neighbor is already close is too late.
    pub avoidance_horizon_ms: u32,
    /// Neighbors not heard from for this long are ignored for separation.
    pub neighbor_timeout_ms: u32,
    /// A searching drone that has heard no other drone over the radio for this long stops
    /// wandering and heads back toward where the swarm was last heard (see `rally_point`).
    pub isolation_timeout_ms: u32,
    /// A neighbor with no HEARTBEAT for this long is considered lost and dropped from the
    /// neighbor table. From the inside, a crashed drone and one that flew out of radio range
    /// look exactly the same — both just go silent.
    pub heartbeat_timeout_ms: u32,
    /// How a drone with nothing to respond to looks for targets.
    pub search: SearchStrategy,
    /// Per-tick probability of picking a new random heading (`SearchStrategy::RandomWalk`).
    pub wander_turn_chance: f32,
    /// How far around itself a drone sees targets: flying over a spot counts as searching
    /// everything this close to it (`SearchStrategy::Coverage`).
    pub scan_radius: f32,
    /// Coverage search: how many seconds of staleness one extra meter of travel is worth. Higher
    /// keeps drones searching near where they are; lower sends them across the field to the
    /// stalest spot.
    pub travel_cost_s_per_m: f32,

    // --- Consensus: don't let a single (possibly wrong or lying) drone redirect the swarm ---
    /// Distinct drones that must have seen the target firsthand before the swarm commits to it.
    pub quorum: u8,
    /// Two firsthand observations count as the same target if they're within this distance.
    pub agree_radius: f32,
    /// Two tracks this close are one target under two names (two drones spotted it
    /// independently) and merge into the lower ID. Much tighter than `agree_radius`: a looser
    /// gate also merges two different targets passing each other, after which the swarm keeps
    /// calling both by one name.
    pub merge_radius: f32,
    /// A firsthand observation older than this no longer counts toward quorum.
    pub vote_window_ms: u32,
    /// How many drones respond to each target: the ones closest to it. Everyone else keeps
    /// searching (and keeps relaying), so one target doesn't stop the whole search.
    pub responders: u8,
    /// Adapt to how many targets there are: a drone doesn't join a target that already has its
    /// share of the swarm (the drones the swarm has, over the targets it knows of; at least one,
    /// at most `responders`), so with as many targets as drones, each drone takes one. Without,
    /// every target gets `responders` while there are drones free.
    pub adaptive_responders: bool,
    /// Drones within this distance of an unconfirmed sighting fly over to check it themselves;
    /// those farther away keep searching (but still relay it).
    pub investigate_radius: f32,
    /// An investigating drone this close to the reported spot that still sees nothing there
    /// concludes the report was wrong.
    pub verify_radius: f32,
    /// ...but only if the report is at most this old. An older report may simply be out of date
    /// (the target moved on), which isn't evidence that its observer is unreliable.
    pub refute_max_age_ms: u32,
    /// After refuting a drone's report, ignore everything it reports for this long.
    pub distrust_ms: u32,

    // --- Operator ---
    /// The launch base: where return-to-launch flies to and lands.
    pub home: Vec2,
    /// A target the operator cancelled is remembered (as cancelled) this long without being
    /// seen, so spotting it again doesn't draw drones back to it.
    pub cancelled_forget_after_ms: u32,
    /// Period of this drone's SWARM_DRONE_STATUS for the ground station.
    pub status_period_ms: u32,
    /// Statuses are relayed toward the ground station at most this many hops.
    pub status_max_hops: u8,
    /// Keep the swarm connected to the ground station: each drone stays within `link_range` of
    /// its link "parent" (the neighbor fewest radio hops from the base, or the base itself), so
    /// the swarm spreads out as a tree rooted at the base instead of drifting out of contact.
    pub keep_link: bool,
    /// With `keep_link`, the swarm stays one network: drones form relay chains out to rings
    /// beyond the base's reach (see `relay_spots`) — free drones first, then drones the rings
    /// can spare (all but `RING_MIN` each) — and responders keep the leash too, letting a target
    /// that runs out of reach go (it's then a "last seen here" for the others) rather than
    /// follow it out of contact. Without, a target beats the link, and a free drone only
    /// relays for a responder it hears that's out of contact or routes through it.
    pub relay_chains: bool,
    /// Drones the base launches on this mission (0: not known). The formation's stretches go by
    /// it (see `Brain::slot_of`).
    pub fleet: u16,
    /// With relay chains: how the free drones not relaying look over the field (see
    /// `SweepPattern`).
    pub sweep_pattern: SweepPattern,
    /// How far a drone may stray from its parent: the radio's range, minus a margin for
    /// latency and both drones moving.
    pub link_range: f32,
    /// A free drone that loses its route to the base flies back toward it for up to this long
    /// after it last had one; if the ground station still isn't reachable (closed, or never
    /// there), it searches freely.
    pub link_lost_timeout_ms: u32,

    // --- Navigation without trustworthy GPS ---
    /// How fast position integrated from the drone's own velocity drifts, m/s.
    pub dead_reckoning_drift: f32,
    /// GPS more than this (plus the neighbors' own uncertainty) from where the neighbors place us
    /// is wrong — jammed into nonsense, or spoofed — and is ignored for `gps_reject_ms`.
    pub gps_disagreement_m: f32,
    pub gps_reject_ms: u32,
    /// A neighbor is a usable reference for our position only if it's sure of its own to
    /// within this.
    pub reference_accuracy_m: f32,

    // --- Clocks and prediction ---
    /// Added to a received sighting's age to account for the radio hop and the wait in the
    /// inbox until the next tick, which the sender couldn't include. Drones' clocks aren't
    /// synchronized, so this estimate is all a receiver has.
    pub hop_latency_ms: u32,
    /// Sightings are carried forward along the target's estimated velocity, but no further
    /// than this: past a couple of seconds the target has likely turned.
    pub max_prediction_ms: u32,
}

impl BrainConfig {
    /// The configuration every drone in the swarm flies with — simulated or on the board, so
    /// it lives here rather than in either runner.
    pub const SWARM: BrainConfig = BrainConfig {
        max_speed: 2.0,
        field_half: Vec2::new(15.0, 15.0),
        forget_after_ms: 5_000,
        last_seen_memory_ms: 30_000,
        // 5 Hz position sharing: at 2 m/s, a 500 ms-old neighbor position is already a meter
        // off, which is most of the separation radius.
        beacon_period_ms: 200,
        heartbeat_period_ms: 1_000,
        // A meter inside the 2.5 m sensor range: the ring follows the target's *predicted*
        // position, and a ring right at the edge of sensor range loses sight of a target the
        // moment the prediction is a little off, as when it starts running.
        standoff_radius: 1.5,
        separation_radius: 1.2,
        // About the time an airframe needs to brake from closing at 2x max speed.
        avoidance_horizon_ms: 1_000,
        neighbor_timeout_ms: 2_000,
        isolation_timeout_ms: 3_000,
        // 3 missed 1 Hz heartbeats plus some slack for radio jitter — the usual MAVLink rule of
        // thumb for declaring a link lost. Shorter detects failures faster but raises false
        // alarms from ordinary packet loss.
        heartbeat_timeout_ms: 3_500,
        search: SearchStrategy::Coverage,
        wander_turn_chance: 0.03,
        // The simulated sensor's range.
        scan_radius: 2.5,
        // A cell 10 m further away has to have gone 20 s longer unsearched to be worth it.
        travel_cost_s_per_m: 2.0,
        // Two independent witnesses: enough that one wrong or lying drone can't redirect the
        // swarm alone, small enough that a real sighting confirms within seconds.
        quorum: 2,
        // Wide enough for two drones' GPS errors (each sees a target where *its* GPS says it
        // is), tight enough to keep apart two targets a few meters apart.
        agree_radius: 2.5,
        // Two drones' fixes on one target differ by the difference of their GPS errors, up to
        // a meter or two; any tighter and one target splits into several names.
        merge_radius: 2.0,
        vote_window_ms: 3_000,
        // At the quorum, so the drones on a target keep it confirmed among themselves. One alone
        // loses its mouse more often; more than two hardly watch it better, and leave the fronts
        // that many drones short for searching.
        responders: 2,
        adaptive_responders: false,
        // Investigate radius sits just above radio range, so a report usually draws in the
        // drones that heard it directly and no one from across the field.
        investigate_radius: 10.0,
        // Well inside the 2.5 m sensor range: a drone this close to a real target sees it.
        verify_radius: 1.0,
        refute_max_age_ms: 1_000,
        distrust_ms: 15_000,
        // The corner of the field the swarm launches from.
        home: Vec2::new(-13.0, -13.0),
        cancelled_forget_after_ms: 60_000,
        // Every 2 s: each drone relays about as many statuses as there are drones, so in a big
        // swarm they're most of what's on the air. Relayed 6 hops: the formation's rosters and the
        // count of drones on a target go by them.
        status_period_ms: 2_000,
        // The field is 30 m across and radio reaches 8 m: about 5 hops corner to corner.
        status_max_hops: 6,
        keep_link: true,
        relay_chains: false,
        fleet: 0,
        sweep_pattern: SweepPattern::Cells,
        // Radio reaches 8 m.
        link_range: 6.5,
        // Long enough for a responder that followed a target out of contact to come back.
        link_lost_timeout_ms: 120_000,
        // A small optical-flow bias: about 10 m after a minute without any fix.
        dead_reckoning_drift: 0.15,
        // Two consumer GPS fixes disagree by a meter or two; five is well past that.
        gps_disagreement_m: 5.0,
        gps_reject_ms: 10_000,
        reference_accuracy_m: 4.0,
        // Radio ~30-100 ms plus up to one 50 ms tick in the inbox.
        hop_latency_ms: 80,
        // The target turns every ~2.5 s on average.
        max_prediction_ms: 2_000,
    };
}

/// How a drone with nothing to respond to looks for targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SearchStrategy {
    /// Fly straight, turn at random now and then. No coordination at all: the baseline.
    RandomWalk,
    /// Shared coverage map: head for the spot that's gone longest without anyone flying over
    /// it, unless a free neighbor is closer to it. Every drone marks the map from its own and
    /// its neighbors' broadcast positions, so it needs no extra messages, and the field ends up
    /// split between drones the way a Voronoi diagram would split it — redrawn on its own when
    /// a drone dies or gets busy with a target.
    Coverage,
}

/// The sweep's shape on a field (see `Brain::sweep_nodes`): drones `spacing` apart on rays out
/// from the base, swinging over `span` radians from `from`, the field's farthest corner `reach`
/// away, `per_ray` drones to reach it.
pub(crate) struct SweepGeometry {
    pub(crate) spacing: f32,
    pub(crate) half: Vec2,
    pub(crate) from: f32,
    pub(crate) span: f32,
    pub(crate) reach: f32,
    pub(crate) per_ray: usize,
}

/// How many drones a mission on a field needs (see `BrainConfig::plan`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FleetPlan {
    /// For the sweep: formations as wide as the field (one pass covers it), enough of them to
    /// look over it in about a minute.
    pub sweep: usize,
    /// For the rings: drones per target, for each target expected.
    pub rings: usize,
    /// For relay chains out to the rings, about (targets assumed at an average distance); none
    /// without relay chains.
    pub relays: usize,
    pub total: usize,
    /// How long the formation takes to look over the whole field once (s), with `sweep` drones
    /// free.
    pub pass_s: f32,
    /// How the `sweep` drones do it (none for the radar sweep).
    pub layout: Option<SweepLayout>,
}

/// How a number of free drones sweep the field in formation (see `BrainConfig::sweep_layout`):
/// in `rows` fronts, one to each stretch of the field along its longer side, each a line of
/// drones abreast (`per_row`, one more where they don't divide evenly) mowing its stretch back
/// and forth in `lanes` lanes. A front is cut into `segments` of neighbors who agree among
/// themselves on their places (see `BrainConfig::segment_max`); side by side, the segments keep
/// the front one radio network.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweepLayout {
    pub drones: usize,
    pub rows: usize,
    pub per_row: usize,
    pub segments: usize,
    pub lanes: usize,
    /// How long it takes to look over the whole field once (s), not counting the flight out.
    pub pass_s: f32,
    /// Of the ground the drones' sensors pass over, the share that's new (0..1): the rest goes
    /// on moving across from one lane to the next and on lanes squeezed narrower than the
    /// front.
    pub efficiency: f32,
}

impl SweepLayout {
    /// Places in front `row`.
    pub fn places(&self, row: usize) -> usize {
        self.drones.saturating_sub(row).div_ceil(self.rows)
    }

    /// The segment place `q` of front `row` is in: (which, its first place, how many places).
    pub fn segment(&self, row: usize, q: usize) -> (usize, usize, usize) {
        split(q, self.places(row), self.segments)
    }
}

impl BrainConfig {
    pub(crate) fn sweep_geometry(&self) -> SweepGeometry {
        let spacing = self.link_range - 0.5;
        let half = self.field_half - Vec2::new(SWEEP_EDGE_MARGIN, SWEEP_EDGE_MARGIN);
        let home = self.home;
        // The angles the field spans as seen from the base, and how far its farthest corner is.
        let corners = [Vec2::new(half.x, half.y), Vec2::new(-half.x, half.y), Vec2::new(half.x, -half.y), Vec2::new(-half.x, -half.y)];
        let reach = corners.iter().map(|c| (*c - home).length()).fold(0.0, f32::max);
        // All the way round if the base is well inside the field; otherwise (by an edge or in a
        // corner) just the angles the field spans from it, leaving out any corner the base is
        // in itself, so no ray points out of the field.
        let from_edge = (half.x - home.x.abs()).min(half.y - home.y.abs());
        let (from, span) = if from_edge > spacing {
            (0.0, 2.0 * core::f32::consts::PI)
        } else {
            let to_center = libm::atan2f(-home.y, -home.x);
            let rel = |c: &Vec2| {
                let a = libm::atan2f(c.y - home.y, c.x - home.x) - to_center;
                libm::atan2f(libm::sinf(a), libm::cosf(a))
            };
            let far = || corners.iter().filter(|c| (**c - home).length() > spacing);
            let lo = far().map(rel).fold(f32::INFINITY, f32::min);
            let hi = far().map(rel).fold(f32::NEG_INFINITY, f32::max);
            (to_center + lo, (hi - lo).max(0.1))
        };
        let per_ray = (libm::ceilf(reach / spacing) as usize).max(1);
        SweepGeometry { spacing, half, from, span, reach, per_ray }
    }

    /// Where the drone with rank `rank` (by system ID) of `count`, all of them free, is at
    /// `t_ms`, the fronts all setting out at once (see `Brain::update_front` for how the drones
    /// pick up their front's loop as it changes). Drones side by side, at most
    /// `formation_spacing` apart (each sees the ground out to `scan_radius`, so their strips
    /// meet, and each is in radio range of the next, so the front is one network), mowing the
    /// field in lanes along its longer side — fewer turns. They set out once the farthest can
    /// have flown to its start.
    pub fn formation_slot(&self, rank: usize, count: usize, t_ms: u32) -> Option<Vec2> {
        if count == 0 || rank >= count {
            return None;
        }
        let fronts = self.sweep_layout(count).rows.clamp(1, MAX_STRETCHES);
        let (index, first, n) = front_of(rank, count, fronts);
        let q = t_ms.saturating_sub(self.front_start_ms(count, index)) as f32 / 1000.0 * FORMATION_SPEED;
        Some(self.front_path(fronts, index, n).slot(q, rank - first, n))
    }

    /// When the last of `launched` drones' fronts sets out (mission clock, ms) (see
    /// `front_start_ms`).
    pub fn sweep_start_ms(&self, launched: usize) -> u32 {
        let launched = launched.max(1);
        let fronts = self.sweep_layout(launched).rows.clamp(1, MAX_STRETCHES);
        (0..fronts).map(|k| self.front_start_ms(launched, k)).max().unwrap_or(0)
    }

    /// When front `index` of `launched` drones' fronts sets out (mission clock, ms): once the
    /// last of its drones can be at its place, launched one every `LAUNCH_INTERVAL_MS` in order
    /// of system ID and flown straight there. Each front by itself, so a front doesn't set out
    /// with its far drones still taking off and its line short at the ends.
    pub fn front_start_ms(&self, launched: usize, index: usize) -> u32 {
        let launched = launched.max(1);
        let fronts = self.sweep_layout(launched).rows.clamp(1, MAX_STRETCHES);
        let (_, first, size) = front_of(index * launched / fronts, launched, fronts);
        let path = self.front_path(fronts, index, size);
        let last_there = (0..size)
            .map(|r| {
                let launch = (first + r) as u32 * LAUNCH_INTERVAL_MS;
                let fly = (path.slot(0.0, r, size) - self.home).length() / self.max_speed * 1000.0;
                launch.saturating_add(fly as u32)
            })
            .max()
            .unwrap_or(0);
        // Not for the last few meters: the drones still on their way catch up as it goes. (Its
        // middle can be there by then anyway.)
        let middle_there = ((path.center(0.0) - self.home).length() / self.max_speed * 1000.0) as u32;
        last_there.saturating_sub(START_SLACK_MS).max(middle_there)
    }

    /// The loop front `index` of `fronts` flies with `members` drones: its stretch of the field
    /// (a band along its longer side), in as many lanes as a line of them a spacing apart
    /// takes to span the field's width. One front starts at the end of the field nearer the
    /// base; with several, each two side by side start from opposite ends of their stretches
    /// (see `fronts_flipped`).
    /// Each keeps half a spacing short of a boundary with another front, so where two meet
    /// there they're a spacing apart, not on top of each other.
    pub(crate) fn front_path(&self, fronts: usize, index: usize, members: usize) -> FrontPath {
        let (across, along, along_y) = self.sweep_extent();
        let unit = 2.0 * along / fronts as f32;
        let inset = 0.5 * self.formation_spacing();
        let (a, b) = (-along + index as f32 * unit, -along + (index + 1) as f32 * unit);
        let (inner_low, inner_high) = (index > 0, index + 1 < fronts);
        let home = if along_y { self.home } else { Vec2::new(self.home.y, self.home.x) };
        let lanes = (libm::ceilf(2.0 * across / (members.max(1) as f32 * self.formation_spacing())) as usize).max(1);
        FrontPath {
            left: -across,
            width: 2.0 * across,
            lanes,
            low: if inner_low { a + inset } else { a },
            high: if inner_high { b - inset } else { b },
            down: if fronts == 1 { home.y > 0.0 } else { index.is_multiple_of(2) != self.fronts_flipped(fronts) },
            flip_u: home.x > 0.0,
            along_y,
            band: (if inner_low { a } else { f32::NEG_INFINITY }, if inner_high { b } else { f32::INFINITY }),
        }
    }

    /// Which way round `fronts` fronts set out (see `front_path`): each two side by side from
    /// opposite ends of their stretches, so they meet at the boundary between them every pass,
    /// and of the two ways that goes, the one whose farthest start is nearer the base, so no front
    /// waits long at its start for the others.
    pub(crate) fn fronts_flipped(&self, fronts: usize) -> bool {
        let (_, along, along_y) = self.sweep_extent();
        let home = if along_y { self.home } else { Vec2::new(self.home.y, self.home.x) };
        let unit = 2.0 * along / fronts as f32;
        let inset = 0.5 * self.formation_spacing();
        let farthest = |flipped: bool| {
            (0..fronts)
                .map(|k| {
                    let v = if k.is_multiple_of(2) != flipped {
                        -along + (k + 1) as f32 * unit - if k + 1 < fronts { inset } else { 0.0 }
                    } else {
                        -along + k as f32 * unit + if k > 0 { inset } else { 0.0 }
                    };
                    Vec2::new(home.x, v - home.y).length()
                })
                .fold(0.0, f32::max)
        };
        farthest(true) < farthest(false)
    }

    /// How far apart the formation's drones are: their sensors' strips meeting, and within
    /// radio range of each other.
    pub(crate) fn formation_spacing(&self) -> f32 {
        (2.0 * self.scan_radius).min(self.link_range - 0.5)
    }

    /// The part of the field the fronts sweep (kept off its edge), as half its size across the
    /// passes and along them, and whether the passes run along y: along the longer side, for
    /// the fewest turns.
    pub(crate) fn sweep_extent(&self) -> (f32, f32, bool) {
        let half = self.field_half - Vec2::new(SWEEP_EDGE_MARGIN, SWEEP_EDGE_MARGIN);
        if half.y >= half.x {
            (half.x, half.y, true)
        } else {
            (half.y, half.x, false)
        }
    }

    /// The most drones `spacing` apart in a line that still agree on who's in it. A drone hears
    /// the ones beside it (as many each way as fit in `link_range`) and their statuses
    /// `status_max_hops` relays farther; in a longer line the drones at one end never hear of
    /// the other end, work out different rosters, and so different places.
    pub fn segment_max(&self, spacing: f32) -> usize {
        let beside = ((self.link_range / spacing) as usize).max(1);
        beside * (self.status_max_hops as usize + 1) + 1
    }

    /// `drones` sweeping the field in `rows` fronts (see `SweepLayout`).
    pub(crate) fn layout_of(&self, drones: usize, rows: usize) -> SweepLayout {
        let (across, along, _) = self.sweep_extent();
        let spacing = self.formation_spacing();
        // The smallest front sets the pace.
        let per_row = (drones / rows).max(1);
        let lanes = libm::ceilf(2.0 * across / (per_row as f32 * spacing)).max(1.0);
        let len = 2.0 * along / rows as f32;
        let pass_s = (lanes * len + (lanes - 1.0) * 2.0 * across / lanes) / FORMATION_SPEED;
        // Every drone sweeping a fresh strip as wide as its sensor's, all the time.
        let ideal_s = 4.0 * across * along / (drones.max(1) as f32 * spacing * FORMATION_SPEED);
        // The biggest front, cut so that its segments hold together.
        let biggest = drones.div_ceil(rows);
        let segments = biggest.div_ceil(self.segment_max(2.0 * across / (lanes * biggest as f32)));
        SweepLayout { drones, rows, per_row, segments, lanes: lanes as usize, pass_s, efficiency: (ideal_s / pass_s).min(1.0) }
    }

    /// How `drones` free drones sweep the field: one front, as wide as the field if there are
    /// drones enough, else mowing it in lanes; with drones for more than one front as wide as
    /// the field, as many as there are, each its own stretch of it. Every front stays one radio
    /// network (see `Brain::update_front`). The drones go by how many of them are free now, so
    /// fronts merge as drones leave for mice, and split again as they come back. Fronts span the
    /// field's width rather than cutting it into narrower columns: columns side by side would be
    /// out of radio range of each other for good, islands that never hear of each other's mice.
    pub fn sweep_layout(&self, drones: usize) -> SweepLayout {
        let drones = drones.max(1);
        let one = self.layout_of(drones, 1);
        let mut best = one;
        for rows in 2..=MAX_STRETCHES.min(drones) {
            let l = self.layout_of(drones, rows);
            if l.lanes > 1 || l.per_row <= MIN_SWEEPERS {
                break;
            }
            best = l;
        }
        best
    }

    /// The fleet sizes worth sweeping the field with, up to `max_drones`: each looks over it
    /// sooner than any smaller one does (by more than 2%). Drones beyond one of these and short
    /// of the next only crowd its fronts.
    pub fn sweep_options(&self, max_drones: usize) -> impl Iterator<Item = SweepLayout> + '_ {
        let mut soonest = f32::INFINITY;
        (1..=max_drones).map(move |n| self.sweep_layout(n)).filter(move |l| {
            let sooner = l.pass_s < soonest * 0.98;
            if sooner {
                soonest = l.pass_s;
            }
            sooner
        })
    }

    /// The fewest drones that look over the field once in at most `pass_s`, or, if no fleet the
    /// formations' roster holds does, the quickest of those.
    pub fn sweep_for(&self, pass_s: f32) -> SweepLayout {
        let mut last = self.sweep_layout(1);
        for l in self.sweep_options(MAX_ROSTER) {
            last = l;
            if l.pass_s <= pass_s {
                break;
            }
        }
        last
    }

    /// How long `free` drones in formation take to look over the whole field once (s) (see
    /// `sweep_layout`).
    pub fn cover_time_s(&self, free: usize) -> f32 {
        self.sweep_layout(free).pass_s
    }

    /// How many drones a mission on this field needs, expecting `targets` targets at a time:
    /// the fewest that look over the field in `PLANNED_PASS_S` (see `sweep_for`), `responders`
    /// for each target, and with relay chains, the relays to keep each ring in contact (targets
    /// taken to be at 60% of the way to the field's farthest corner, on average). Fewer free
    /// drones, and the sweep takes longer (see `cover_time_s`).
    pub fn plan(&self, targets: usize) -> FleetPlan {
        if self.sweep_pattern == SweepPattern::Radar {
            return self.plan_radar(targets);
        }
        let layout = self.sweep_for(PLANNED_PASS_S);
        let sweep = layout.drones;
        let rings = targets * self.responders as usize;
        // Relays only while keeping the link: an autonomous swarm keeps none out to its rings.
        let relays = if self.keep_link {
            let half = self.field_half - Vec2::new(SWEEP_EDGE_MARGIN, SWEEP_EDGE_MARGIN);
            let corners = [Vec2::new(half.x, half.y), Vec2::new(-half.x, half.y), Vec2::new(half.x, -half.y), Vec2::new(-half.x, -half.y)];
            let reach = corners.iter().map(|c| (*c - self.home).length()).fold(0.0, f32::max);
            let out = 0.6 * reach - self.standoff_radius;
            let chain_spacing = self.link_range - RELAY_MARGIN_M;
            let relays_each = if out > self.link_range { (libm::ceilf(out / chain_spacing) as usize).saturating_sub(1) } else { 0 };
            targets * relays_each
        } else {
            0
        };
        FleetPlan { sweep, rings, relays, total: sweep + rings + relays, pass_s: layout.pass_s, layout: Some(layout) }
    }

    /// `plan` for the radar sweep: one ray long enough to reach the field's farthest corner, and
    /// how long it takes to swing across the field.
    pub(crate) fn plan_radar(&self, targets: usize) -> FleetPlan {
        let g = self.sweep_geometry();
        let rings = targets * self.responders as usize;
        let out = 0.6 * g.reach - self.standoff_radius;
        let relays_each = if out > self.link_range { (libm::ceilf(out / g.spacing) as usize).saturating_sub(1) } else { 0 };
        let relays = targets * relays_each;
        FleetPlan { sweep: g.per_ray, rings, relays, total: g.per_ray + rings + relays, pass_s: (g.span * g.reach / SWEEP_SPEED).max(1.0), layout: None }
    }
}

/// The two ways the swarm can work (the operator picks one: `MAV_CMD_SWARM_SET_MODE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize), serde(rename_all = "snake_case"))]
pub enum SwarmMode {
    /// An autonomous swarm: off the leash, the free drones sweeping the field in formation
    /// (the quickest way to look over all of it), rings following their mice wherever they run;
    /// in contact with the base only when near it.
    Autonomous,
    /// Always in contact with the base, by relay chains: the free drones sweep the field in rays
    /// out from it, which is slower, and rings let a mouse go rather than leave the chain.
    Connected,
}

impl SwarmMode {
    pub fn to_u8(self) -> u8 {
        match self {
            SwarmMode::Autonomous => 0,
            SwarmMode::Connected => 1,
        }
    }

    pub fn from_u8(v: u8) -> Self {
        if v == 1 { SwarmMode::Connected } else { SwarmMode::Autonomous }
    }
}

impl BrainConfig {
    /// This configuration working in `mode`.
    pub const fn with_mode(self, mode: SwarmMode) -> Self {
        match mode {
            SwarmMode::Autonomous => BrainConfig { keep_link: false, relay_chains: false, sweep_pattern: SweepPattern::Formation, ..self },
            SwarmMode::Connected => BrainConfig { keep_link: true, relay_chains: true, sweep_pattern: SweepPattern::Radar, ..self },
        }
    }

    /// The mode this configuration works in, as near as it comes to one.
    pub fn mode(&self) -> SwarmMode {
        if self.keep_link && self.relay_chains { SwarmMode::Connected } else { SwarmMode::Autonomous }
    }
}

/// How the free drones look over the field (see `SwarmMode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepPattern {
    /// None: each free drone searches the coverage map's cells nearest to it, within its leash
    /// (see `Brain::cover`).
    Cells,
    /// Side by side, mowing the field in lanes (see `BrainConfig::formation_slot`): the
    /// quickest way to look over all of it, the formation in contact with itself throughout but
    /// out of the base's reach much of the time — and so are the free drones a ring chasing its
    /// target needs for its next relay.
    Formation,
    /// In rays out from the base swinging across the field like a radar's beam (see
    /// `Brain::sweep_nodes`): slower, but everyone stays in contact with the base all the time.
    Radar,
}
