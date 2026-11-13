//! OpenTelemetry for the whole simulator, exported over OTLP/HTTP (default
//! http://localhost:4318, override with OTEL_EXPORTER_OTLP_ENDPOINT) to the collector in
//! observability/docker-compose.yml.
//!
//! - Traces: one trace per target sighting, showing how the news spread through the swarm.
//!   The drone that spots the target records a `target_spotted` root span; every drone that
//!   then learns of it records a `sighting_relayed` span parented to the span of whichever
//!   drone relayed it — the parent span ID travels inside the MAVLink frame itself.
//! - Metrics: radio channel health, per-drone traffic, and swarm-level outcomes.
//!
//! All metric and span names live here so the Grafana dashboard has one place to match
//! against. `Brain` stays telemetry-free so it remains portable to the board; its runner task
//! reports on its behalf.
//!
//! If no collector is running, export fails quietly in the background and the simulation runs
//! as normal.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, OnceLock, RwLock};
use std::time::Duration;

use opentelemetry::metrics::{Counter, Gauge, Histogram, MeterProvider as _};
use opentelemetry::trace::{
    Span, SpanContext, SpanId, TraceContextExt, TraceFlags, TraceId, TraceState, Tracer, TracerProvider as _,
};
use opentelemetry::{Context, KeyValue};
use opentelemetry_otlp::{MetricExporter, SpanExporter};
use opentelemetry_sdk::metrics::{PeriodicReader, SdkMeterProvider};
use opentelemetry_sdk::trace::{SdkTracer, SdkTracerProvider};
use opentelemetry_sdk::Resource;

use swarm_brain::TraceEvent;
use swarm_brain::Vec2;

/// What the simulator knows that no drone does, used only to label telemetry correctly.
struct GroundTruth {
    positions: Arc<RwLock<Vec<Vec2>>>,
    radio_range: f32,
}

const SERVICE_NAME: &str = "swarm-sim";

/// Bucket boundaries in seconds, from radio delays (tens of ms) up to sighting ages (seconds).
/// OTel's default boundaries are tuned for millisecond-scale values and would put everything in
/// the first bucket here.
const SECONDS_BUCKETS: &[f64] = &[0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0];

/// Keeps the exporters alive; `shutdown` flushes whatever is still buffered.
pub struct TelemetryGuard {
    tracer_provider: SdkTracerProvider,
    meter_provider: SdkMeterProvider,
}

impl TelemetryGuard {
    pub fn shutdown(self) {
        if let Err(e) = self.tracer_provider.shutdown() {
            eprintln!("telemetry: trace shutdown failed: {e}");
        }
        if let Err(e) = self.meter_provider.shutdown() {
            eprintln!("telemetry: metrics shutdown failed: {e}");
        }
    }
}

/// Ground truth about a drone that was killed — the simulator knows exactly when it died,
/// which is what lets it measure how long the swarm took to notice.
struct Death {
    died_at_ms: u32,
    /// It didn't crash: it was taken out of the mission (landed, flew home).
    left: bool,
    /// The `drone_failed` root span; every detection by a neighbor becomes a child of it.
    span: SpanContext,
}

pub struct Telemetry {
    tracer: SdkTracer,
    deaths: Mutex<HashMap<u8, Death>>,
    ground_truth: OnceLock<GroundTruth>,
    /// Drones the viewer turned into liars — so a refutation can be judged right or wrong.
    liars: Mutex<HashSet<u8>>,

    pub netem_frames_in: Counter<u64>,
    pub netem_delivered: Counter<u64>,
    pub netem_lost: Counter<u64>,
    pub netem_out_of_range: Counter<u64>,
    pub netem_malformed: Counter<u64>,
    pub netem_impostor: Counter<u64>,
    pub netem_delivery_delay: Histogram<f64>,

    agent_frames_received: Counter<u64>,
    agent_inbox_overflow: Counter<u64>,
    agent_frames_unsigned: Counter<u64>,
    agent_became_informed: Counter<u64>,
    relayed_sighting_age: Histogram<f64>,

    drones_failed: Counter<u64>,
    neighbor_lost: Counter<u64>,
    failure_detection_latency: Histogram<f64>,

    sighting_refuted: Counter<u64>,
    sighting_rejected: Counter<u64>,
    order_rejected: Counter<u64>,
    liar_count: Gauge<u64>,
    /// Labelled by `mode` (searching / investigating / converging).
    pub agents_by_mode: Gauge<u64>,

    pub agents: Gauge<u64>,
    pub agents_alive: Gauge<u64>,
    pub agents_informed: Gauge<u64>,
    pub agents_isolated: Gauge<u64>,
    /// Labelled by `target` (the world's target ID).
    pub target_nearest_agent_distance: Gauge<f64>,
    /// Labelled by `target`.
    pub target_drones_near: Gauge<u64>,
    pub targets_covered: Gauge<u64>,
    pub world_missed_reports: Counter<u64>,
    offboard_lost: Counter<u64>,

    pub collisions: Counter<u64>,
    pub wind_speed: Gauge<f64>,
    /// Labelled by `drone.sysid`.
    pub tilt: Gauge<f64>,

    pub search_coverage: Gauge<f64>,
    pub link_connected: Gauge<f64>,
    pub gps_error: Gauge<f64>,
    pub nav_error: Gauge<f64>,
    /// Labelled by `nav` (gps / neighbors / dead_reckoning).
    pub agents_by_nav: Gauge<u64>,
    pub time_to_find: Histogram<f64>,
}

/// Builds the OTLP exporters. Must be called *outside* the tokio runtime: the default OTLP/HTTP
/// client is reqwest's blocking client, which panics if created inside an async context.
pub fn init() -> Result<(TelemetryGuard, Telemetry), Box<dyn std::error::Error>> {
    let resource = Resource::builder().with_service_name(SERVICE_NAME).build();

    let tracer_provider = SdkTracerProvider::builder()
        .with_batch_exporter(SpanExporter::builder().with_http().build()?)
        .with_resource(resource.clone())
        .build();

    // The default 60s export interval would make the dashboard look frozen.
    let reader = PeriodicReader::builder(MetricExporter::builder().with_http().build()?)
        .with_interval(Duration::from_secs(5))
        .build();
    let meter_provider = SdkMeterProvider::builder().with_reader(reader).with_resource(resource).build();

    let meter = meter_provider.meter("swarm");
    let telemetry = Telemetry {
        tracer: tracer_provider.tracer("swarm"),
        deaths: Mutex::new(HashMap::new()),
        ground_truth: OnceLock::new(),
        liars: Mutex::new(HashSet::new()),

        netem_frames_in: meter
            .u64_counter("swarm.netem.frames_in")
            .with_description("MAVLink frames transmitted by drones into the radio channel")
            .build(),
        netem_delivered: meter
            .u64_counter("swarm.netem.delivered")
            .with_description("Frame deliveries that reached a receiver")
            .build(),
        netem_lost: meter
            .u64_counter("swarm.netem.lost")
            .with_description("Frame deliveries lost to random packet loss")
            .build(),
        netem_out_of_range: meter
            .u64_counter("swarm.netem.out_of_range")
            .with_description("Potential receivers too far from the sender to hear a frame")
            .build(),
        netem_malformed: meter
            .u64_counter("swarm.netem.malformed")
            .with_description("Datagrams that were not valid MAVLink v2 frames")
            .build(),
        netem_impostor: meter
            .u64_counter("swarm.netem.impostor")
            .with_description("Frames dropped for claiming a system ID another radio is transmitting as, or posing as a board's simulator link")
            .build(),
        netem_delivery_delay: meter
            .f64_histogram("swarm.netem.delivery_delay")
            .with_unit("s")
            .with_description("Radio latency applied to each delivery")
            .with_boundaries(SECONDS_BUCKETS.to_vec())
            .build(),

        agent_frames_received: meter
            .u64_counter("swarm.agent.frames_received")
            .with_description("MAVLink frames a drone received and parsed")
            .build(),
        agent_frames_unsigned: meter
            .u64_counter("swarm.agent.frames_unsigned")
            .with_description(
                "Radio frames a drone dropped unread: not signed with the mission's link key (MAVLink 2 message signing), \
                 or stale. Someone not of the swarm on the air",
            )
            .build(),
        agent_inbox_overflow: meter
            .u64_counter("swarm.agent.inbox_overflow")
            .with_description("Frames a drone dropped because more arrived between two ticks than it keeps (swarm_brain::MAX_INBOX)")
            .build(),
        agent_became_informed: meter
            .u64_counter("swarm.agent.became_informed")
            .with_description("Times a drone went from knowing nothing about the target to having a sighting")
            .build(),
        relayed_sighting_age: meter
            .f64_histogram("swarm.agent.relayed_sighting_age")
            .with_unit("s")
            .with_description("How old a relayed sighting already was when a drone first adopted it")
            .with_boundaries(SECONDS_BUCKETS.to_vec())
            .build(),

        drones_failed: meter
            .u64_counter("swarm.drones.failed")
            .with_description("Drones killed (by the viewer's fault controls)")
            .build(),
        neighbor_lost: meter
            .u64_counter("swarm.agent.neighbor_lost")
            .with_description(
                "Times a drone declared a neighbor lost after its heartbeats stopped. verdict=dead: it really \
                 failed; verdict=out_of_range: alive but beyond radio range, a correct loss of link; \
                 verdict=false_alarm: alive and in range, heartbeats were just lost to the radio; verdict=left: \
                 taken out of the mission on purpose",
            )
            .build(),
        failure_detection_latency: meter
            .f64_histogram("swarm.failure.detection_latency")
            .with_unit("s")
            .with_description("Time from a drone's death until a given neighbor declared it lost")
            .with_boundaries(SECONDS_BUCKETS.to_vec())
            .build(),

        sighting_refuted: meter
            .u64_counter("swarm.agent.sighting_refuted")
            .with_description(
                "A drone checked a reported sighting in person, found nothing, and stopped trusting the observer. \
                 verdict=liar_caught: the observer really was lying; verdict=honest_distrusted: it wasn't",
            )
            .build(),
        order_rejected: meter
            .u64_counter("swarm.agent.order_rejected")
            .with_description(
                "Orders, stick input and revocations a drone dropped because they weren't signed by the operator \
                 (or the base): someone else on the air trying to give them",
            )
            .build(),
        sighting_rejected: meter
            .u64_counter("swarm.agent.sighting_rejected")
            .with_description(
                "Sightings a drone dropped unread. reason=forged: the signature didn't match (made up, or changed \
                 on the way); reason=unchecked: it waited too long for a signature check or its observer's certificate",
            )
            .build(),
        offboard_lost: meter
            .u64_counter("swarm.drone.offboard_lost")
            .with_description("Drones whose brain sent no command for a while, flown home (or landed) by their autopilot's failsafe")
            .build(),
        liar_count: meter
            .u64_gauge("swarm.liars")
            .with_description("Drones whose radio was compromised to broadcast a fake target")
            .build(),
        agents_by_mode: meter
            .u64_gauge("swarm.agents.mode")
            .with_description("Live drones by what they're doing: searching, investigating a report, or converging on a confirmed target")
            .build(),

        agents: meter.u64_gauge("swarm.agents").with_description("Drones in the swarm").build(),
        agents_alive: meter.u64_gauge("swarm.agents.alive").with_description("Drones still flying").build(),
        agents_informed: meter
            .u64_gauge("swarm.agents.informed")
            .with_description("Drones that currently have a sighting of the target")
            .build(),
        agents_isolated: meter
            .u64_gauge("swarm.agents.isolated")
            .with_description("Drones that currently hear no other drone over the radio")
            .build(),
        target_drones_near: meter
            .u64_gauge("swarm.target.drones_near")
            .with_description("Live drones within 4 m of each target: about `responders` if the division of labor works")
            .build(),
        targets_covered: meter
            .u64_gauge("swarm.targets.covered")
            .with_description("Targets with at least one drone within 4 m of them")
            .build(),
        target_nearest_agent_distance: meter
            .f64_gauge("swarm.target.nearest_agent_distance")
            .with_unit("m")
            .with_description("Distance from each target to the closest drone (true positions)")
            .build(),
        world_missed_reports: meter
            .u64_counter("swarm.world.missed_reports")
            .with_description("Ticks on which a drone's brain failed to answer in time")
            .build(),

        collisions: meter
            .u64_counter("swarm.collisions")
            .with_description("Times two drones' airframes physically touched")
            .build(),
        wind_speed: meter.f64_gauge("swarm.wind.speed").with_unit("m/s").with_description("Wind speed over the field").build(),
        tilt: meter
            .f64_gauge("swarm.drone.tilt")
            .with_unit("deg")
            .with_description("How far each drone's rotor disk is tilted from level: how hard it's accelerating or fighting wind")
            .build(),

        nav_error: meter
            .f64_gauge("swarm.nav.error")
            .with_unit("m")
            .with_description("Mean distance between where drones think they are and where they really are")
            .build(),
        agents_by_nav: meter
            .u64_gauge("swarm.agents.nav")
            .with_description("Flying drones by where their position comes from: gps, neighbors (no usable GPS), dead_reckoning")
            .build(),
        gps_error: meter
            .f64_gauge("swarm.gps.error")
            .with_unit("m")
            .with_description("Mean distance between where drones' GPS says they are and where they really are")
            .build(),
        link_connected: meter
            .f64_gauge("swarm.link.connected")
            .with_description("Fraction of flying drones the ground station at the base can reach by a chain of radio links (true positions)")
            .build(),
        search_coverage: meter
            .f64_gauge("swarm.search.coverage")
            .with_description("Fraction of the field some drone has flown over recently (true positions)")
            .build(),
        time_to_find: meter
            .f64_histogram("swarm.target.time_to_find")
            .with_unit("s")
            .with_description("From a target appearing (mission start, or added from the viewer) until a drone first had it within sensor range")
            .with_boundaries(vec![5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 90.0, 120.0, 180.0, 300.0])
            .build(),
    };

    Ok((TelemetryGuard { tracer_provider, meter_provider }, telemetry))
}

impl Telemetry {
    /// Records one span for a brain's `TraceEvent` and returns its (trace ID, span ID), which
    /// the brain embeds in the sightings it relays.
    pub fn record_event(&self, sysid: u8, event: TraceEvent) -> ([u8; 16], [u8; 8]) {
        let drone = KeyValue::new("drone.sysid", sysid as i64);

        let mut span = match event {
            TraceEvent::Spotted => self
                .tracer
                .span_builder("target_spotted")
                .with_attributes([drone])
                .start(&self.tracer),
            TraceEvent::Adopted { trace_id, parent_span_id, observer_sysid, hops, age_ms } => {
                self.relayed_sighting_age.record(age_ms as f64 / 1000.0, &[]);
                let builder = self.tracer.span_builder("sighting_relayed").with_attributes([
                    drone,
                    KeyValue::new("sighting.observer_sysid", observer_sysid as i64),
                    KeyValue::new("sighting.hops", hops as i64),
                    KeyValue::new("sighting.age_ms", age_ms as i64),
                ]);
                // The sender's span arrives inside the MAVLink frame; continue its trace as a
                // remote parent. A zero trace ID means the sender wasn't tracing — start fresh.
                let parent = SpanContext::new(
                    TraceId::from_bytes(trace_id),
                    SpanId::from_bytes(parent_span_id),
                    TraceFlags::SAMPLED,
                    true,
                    TraceState::default(),
                );
                if parent.is_valid() {
                    builder.start_with_context(&self.tracer, &Context::new().with_remote_span_context(parent))
                } else {
                    builder.start(&self.tracer)
                }
            }
        };

        let ids = span.span_context();
        let result = (ids.trace_id().to_bytes(), ids.span_id().to_bytes());
        span.end();
        result
    }

    /// Called by the world when a drone is killed: opens a `drone_failed` trace that every
    /// neighbor's later detection hangs off of.
    pub fn drone_failed(&self, sysid: u8, time_ms: u32) {
        self.drones_failed.add(1, &[]);
        let mut span = self
            .tracer
            .span_builder("drone_failed")
            .with_attributes([KeyValue::new("drone.sysid", sysid as i64)])
            .start(&self.tracer);
        let ctx = span.span_context().clone();
        span.end();
        self.deaths.lock().unwrap().insert(sysid, Death { died_at_ms: time_ms, left: false, span: ctx });
    }

    /// A drone was taken out of the mission on purpose. Its neighbors find out the same way as
    /// for a crash — its heartbeats stop — but it isn't a failure.
    pub fn drone_left(&self, sysid: u8, time_ms: u32) {
        let mut span = self
            .tracer
            .span_builder("drone_left")
            .with_attributes([KeyValue::new("drone.sysid", sysid as i64)])
            .start(&self.tracer);
        let ctx = span.span_context().clone();
        span.end();
        self.deaths.lock().unwrap().insert(sysid, Death { died_at_ms: time_ms, left: true, span: ctx });
    }

    /// Called when `observer` drops `lost` from its neighbor table. The drone itself can't tell
    /// a crash from flying out of range; the simulator can, and labels the verdict accordingly.
    pub fn neighbor_lost(&self, observer: u8, lost: u8, silent_ms: u32, now_ms: u32) {
        let death = self
            .deaths
            .lock()
            .unwrap()
            .get(&lost)
            .map(|d| (d.died_at_ms, d.left, d.span.clone()));
        let Some((died_at_ms, left, parent)) = death else {
            let verdict = if self.in_radio_range(observer, lost) { "false_alarm" } else { "out_of_range" };
            self.neighbor_lost.add(1, &[KeyValue::new("verdict", verdict)]);
            return;
        };

        self.neighbor_lost.add(1, &[KeyValue::new("verdict", if left { "left" } else { "dead" })]);
        let latency_ms = now_ms.saturating_sub(died_at_ms);
        if !left {
            self.failure_detection_latency.record(latency_ms as f64 / 1000.0, &[]);
        }
        let mut span = self
            .tracer
            .span_builder("failure_detected")
            .with_attributes([
                KeyValue::new("drone.sysid", observer as i64),
                KeyValue::new("failed.sysid", lost as i64),
                KeyValue::new("detection.latency_ms", latency_ms as i64),
                KeyValue::new("heartbeat.silent_ms", silent_ms as i64),
            ])
            .start_with_context(&self.tracer, &Context::new().with_remote_span_context(parent));
        span.end();
    }

    /// Forgets the previous mission's deaths and liars, so a new mission's failure detections
    /// and refutations are judged against its own ground truth. Counters keep counting.
    pub fn reset(&self) {
        self.deaths.lock().unwrap().clear();
        self.liars.lock().unwrap().clear();
        self.liar_count.record(0, &[]);
        self.wind_speed.record(0.0, &[]);
    }

    pub fn drone_compromised(&self, sysid: u8) {
        let mut liars = self.liars.lock().unwrap();
        liars.insert(sysid);
        self.liar_count.record(liars.len() as u64, &[]);
    }

    pub fn sighting_refuted(&self, refuter: u8, observer: u8) {
        let verdict = if self.liars.lock().unwrap().contains(&observer) { "liar_caught" } else { "honest_distrusted" };
        self.sighting_refuted.add(
            1,
            &[KeyValue::new("drone.sysid", refuter as i64), KeyValue::new("verdict", verdict)],
        );
    }

    pub fn offboard_lost(&self, sysid: u8) {
        self.offboard_lost.add(1, &[KeyValue::new("drone.sysid", sysid as i64)]);
    }

    /// More sightings dropped by `sysid` since it last said (see `Brain::sightings_rejected`).
    pub fn sightings_rejected(&self, sysid: u8, forged: u32, unchecked: u32) {
        for (reason, n) in [("forged", forged), ("unchecked", unchecked)] {
            if n > 0 {
                self.sighting_rejected.add(n as u64, &[KeyValue::new("drone.sysid", sysid as i64), KeyValue::new("reason", reason)]);
            }
        }
    }

    /// More forged orders dropped by `sysid` since it last said (see `Brain::orders_rejected`).
    pub fn orders_rejected(&self, sysid: u8, forged: u32) {
        if forged > 0 {
            self.order_rejected.add(forged as u64, &[KeyValue::new("drone.sysid", sysid as i64)]);
        }
    }

    /// Gives telemetry access to true drone positions (indexed by sysid - 1), so it can tell a
    /// real false alarm from a neighbor that simply flew out of range. Call once at startup.
    pub fn set_ground_truth(&self, positions: Arc<RwLock<Vec<Vec2>>>, radio_range: f32) {
        let _ = self.ground_truth.set(GroundTruth { positions, radio_range });
    }

    fn in_radio_range(&self, a: u8, b: u8) -> bool {
        let Some(truth) = self.ground_truth.get() else { return false };
        let positions = truth.positions.read().unwrap();
        let pos = |sysid: u8| positions.get((sysid as usize).wrapping_sub(1)).copied();
        match (pos(a), pos(b)) {
            (Some(pa), Some(pb)) => (pa - pb).length() <= truth.radio_range,
            _ => false,
        }
    }

    pub fn became_informed(&self, sysid: u8, source: &'static str) {
        self.agent_became_informed.add(
            1,
            &[KeyValue::new("drone.sysid", sysid as i64), KeyValue::new("source", source)],
        );
    }

    pub fn frame_received(&self, sysid: u8) {
        self.agent_frames_received.add(1, &[KeyValue::new("drone.sysid", sysid as i64)]);
    }

    pub fn frame_unsigned(&self, sysid: u8) {
        self.agent_frames_unsigned.add(1, &[KeyValue::new("drone.sysid", sysid as i64)]);
    }

    pub fn inbox_overflow(&self, sysid: u8) {
        self.agent_inbox_overflow.add(1, &[KeyValue::new("drone.sysid", sysid as i64)]);
    }
}
