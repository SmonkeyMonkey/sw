//! Hardware-in-the-loop bridge: runs one drone of the swarm on the STM32 board instead of in a
//! tokio task. To the world it looks exactly like `agent::run` (same `AgentLinks`: sensor
//! readings in, reports out); to the rest of the swarm it's just another radio on the emulated
//! network. The only difference is where the brain runs.
//!
//! One serial link carries everything, as MAVLink v2:
//!  - SWARM_HIL_SENSOR (to board) / SWARM_HIL_COMMAND (from board): the drone's own body —
//!    one sensor reading starts one brain tick, one command ends it.
//!  - SWARM_HIL_TRACE_EVENT / SWARM_HIL_TRACE_SPAN: the board has no OpenTelemetry, so it asks
//!    the bridge to record its spans, the same split as `record_event` + `attach_span` in
//!    `agent::run`.
//!  - SWARM_HIL_MISSION (to board): what the launch base tells a drone before it takes off: its
//!    keys, the field, where the base is, the swarm's mode and search. Repeated every second, so
//!    a board that restarts mid-mission gets it again.
//!  - anything else is radio traffic, relayed unchanged between the board and the emulated radio
//!    (netem) — the bridge is the board's radio. Byte for byte: the board signs its own frames
//!    with the mission's link key and checks everyone else's (see `swarm_brain::radio`).

use std::sync::Arc;

use mavlink_swarm::mavlink_core::{
    connect_async, AsyncMavConnection, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkReader, MavlinkVersion, Message,
};
use mavlink_swarm::{MavMessage, SWARM_HIL_TRACE_SPAN_DATA, SWARM_TARGET_SIGHTING_DATA};
use swarm_brain::hil::Mission;
use swarm_brain::{hil, radio, Mode, Nav, TraceEvent, Vec2, COMPONENT_ID};
use tokio::net::UdpSocket;
use tokio::sync::mpsc;

use crate::agent::AgentLinks;
use crate::body::{AgentReport, Fault};

/// System ID the bridge uses for the HIL-only messages it sends the board: MAVLink's
/// conventional ID for a ground station / simulator, never one of the drones'.
const SIMULATOR_SYSID: u8 = 255;

/// How much radio traffic for the board the serial link carries (bytes a second), and how big a
/// burst (in seconds' worth of it); the rest is lost, like frames the radio had no airtime for.
/// Well under the 1 Mbaud line (100 KB/s): what the board can take in and still keep its ticks,
/// with an interrupt for every byte it receives.
const RADIO_BYTES_PER_S: f64 = 30_000.0;
const RADIO_BURST_S: f64 = 0.1;

/// Radio frames waiting to be written to the board, at most: about 40 ms of the link. Past
/// that the link is behind, and a frame that doesn't fit is lost like one the radio had no
/// airtime for, rather than holding up the next sensor reading.
const RADIO_QUEUE: usize = 32;

/// The mission goes to the board again every this many sensor readings (a second's worth).
const MISSION_EVERY: u32 = 20;

type Connection = Arc<Box<dyn AsyncMavConnection<MavMessage> + Sync + Send>>;

/// `address` is a mavlink-core connection address: `serial:<port>:<baud>` for the board, or
/// e.g. `tcpin:127.0.0.1:5760` for the host-side fake board (examples/fake_board.rs).
pub async fn run(mission: Mission, mut links: AgentLinks, address: &str, beacon_period_ms: u32) -> std::io::Result<()> {
    let keys = mission.keys;
    let sysid = keys.sysid;
    println!("hil: drone {sysid} waits for its brain on {address}");
    let board: Connection = Arc::new(connect_async::<MavMessage>(address).await?);
    let radio = UdpSocket::bind("127.0.0.1:0").await?;
    let telemetry = links.telemetry.clone();
    println!("hil: drone {sysid}'s brain connected");

    // A dedicated reader task: `recv` holds the port's read lock while it waits, so it gets its
    // own task instead of being raced (and cancelled) in the select below.
    let (from_board_tx, mut from_board) = mpsc::channel::<MAVLinkV2MessageRaw>(256);
    let reader = board.clone();
    // Aborted when this function returns (drone killed, or the mission reset), so the serial
    // port is actually closed and the next mission's bridge can open it again.
    let _reader = AbortOnDrop(tokio::spawn(async move {
        loop {
            match reader.recv_raw().await {
                Ok(MAVLinkMessageRaw::V2(frame)) => {
                    if from_board_tx.send(frame).await.is_err() {
                        return;
                    }
                }
                Ok(MAVLinkMessageRaw::V1(_)) => {}
                Err(e) => {
                    eprintln!("hil: serial read failed: {e}");
                    return;
                }
            }
        }
    }));

    // And a dedicated writer: what the board's body needs (sensor readings, its mission, spans)
    // always goes first; radio frames wait in a short queue behind it.
    let (body, body_rx) = mpsc::channel::<MavMessage>(16);
    let (radio_out, radio_rx) = mpsc::channel::<MAVLinkV2MessageRaw>(RADIO_QUEUE);
    let slowest_write = Arc::new(std::sync::atomic::AtomicU64::new(0));
    let _writer = AbortOnDrop(tokio::spawn(write_to_board(board.clone(), body_rx, radio_rx, slowest_write.clone())));

    let mut recv_buf = [0u8; 512];
    // Only for what the bridge injects itself, as a compromised radio: it has the link key.
    let mut liar_radio = radio::Signer::new(keys.link, sysid);
    let mut informed = false;
    // Whether the board reported spotting the target itself during the current tick.
    let mut spotted_this_tick = false;
    let mut lie: Option<(Vec2, u32)> = None;
    let mut lie_sequence: u8 = 0;
    let mut readings: u32 = 0;
    let mut stats = LinkStats::new();
    let mut airtime = Airtime::new(RADIO_BYTES_PER_S);

    loop {
        tokio::select! {
            reading = links.sensor_rx.recv() => {
                // Killed: stop relaying. The board simply stops getting ticks and its radio
                // goes silent, the same as a simulated drone's task ending.
                let Some(reading) = reading else { return Ok(()) };
                while let Ok(Fault::Lie { fake_pos }) = links.fault_rx.try_recv() {
                    lie = Some((fake_pos, reading.time_ms));
                }
                spotted_this_tick = false;
                // Before the first reading: the board flies only once it has its mission.
                if readings.is_multiple_of(MISSION_EVERY) {
                    send_to_board(&body, hil::mission_msg(&mission)).await?;
                }
                readings = readings.wrapping_add(1);
                stats.to_board += send_to_board(&body, hil::sensor_msg(&reading, sysid)).await?;
                stats.tick_sent(reading.time_ms);
                stats.report_now(sysid, &slowest_write);

                // A compromised radio: same behavior as in `agent::run`, injected here because
                // the bridge *is* this drone's radio.
                if let Some((fake_pos, next_ms)) = lie {
                    if reading.time_ms >= next_ms {
                        lie = Some((fake_pos, reading.time_ms + beacon_period_ms));
                        let mut fake = SWARM_TARGET_SIGHTING_DATA {
                            // Looks like a track of its own, as a real first sighting would.
                            target_id: ((sysid as u16) << 8) | 0xFF,
                            age_ms: 0,
                            x: fake_pos.x,
                            y: fake_pos.y,
                            vx: 0.0,
                            vy: 0.0,
                            observer_sysid: sysid,
                            cancelled: 0,
                            responders: 0,
                            hops: 0,
                            trace_id: [0; 16],
                            parent_span_id: [0; 8],
                            observed_ms: 0,
                            signature: [0; 64],
                        };
                        // With the drone's own key, which the bridge has as its launch base.
                        keys.sign_sighting(&mut fake, reading.time_ms);
                        let fake = MavMessage::SWARM_TARGET_SIGHTING(fake);
                        let header = MavHeader { system_id: sysid, component_id: COMPONENT_ID, sequence: lie_sequence };
                        lie_sequence = lie_sequence.wrapping_add(1);
                        radio.send_to(liar_radio.frame(header, &fake, reading.time_ms).raw_bytes(), links.netem_addr).await?;
                    }
                }
            }
            received = radio.recv_from(&mut recv_buf) => {
                let (len, _) = received?;
                let mut reader = MavlinkReader::new(&recv_buf[..len]);
                while let Ok(raw) = reader.read_raw_message::<MavMessage>(MavlinkVersion::V2) {
                    let MAVLinkMessageRaw::V2(raw) = raw else { continue };
                    // Read, not checked: the board checks the signature itself.
                    let Some((_, msg)) = radio::parse(&raw) else { continue };
                    // The board's body link shares its UART with the radio: nothing that
                    // belongs to it gets in from the air (see `hil::is_body_link`).
                    if hil::is_body_link(&msg) {
                        continue;
                    }
                    // Over the link's share for radio: lost, like a frame the radio had no
                    // airtime for. The TCP fake board gets the same, to hear what the board would.
                    let len = raw.raw_bytes().len();
                    if !airtime.take(len) {
                        stats.radio_dropped += 1;
                        continue;
                    }
                    let kind = msg.message_name();
                    match radio_out.try_send(raw) {
                        Ok(()) => {
                            telemetry.frame_received(sysid);
                            stats.to_board += len;
                            stats.radio_frames += 1;
                            *stats.by_kind.entry(kind).or_default() += 1;
                        }
                        Err(mpsc::error::TrySendError::Full(_)) => stats.radio_dropped += 1,
                        Err(mpsc::error::TrySendError::Closed(_)) => return Err(std::io::Error::other("serial link to the board closed")),
                    }
                }
            }
            frame = from_board.recv() => {
                let Some(raw) = frame else {
                    return Err(std::io::Error::other("serial link to the board closed"));
                };
                stats.from_board += raw.raw_bytes().len();
                let Some((_, msg)) = radio::parse(&raw) else { continue };
                match msg {
                    MavMessage::SWARM_HIL_COMMAND(cmd) => {
                        stats.tick_answered(cmd.time_ms);
                        for (&lost, &silent_ms) in cmd.lost_sysid.iter().zip(&cmd.lost_silent_ms) {
                            if lost != 0 {
                                telemetry.neighbor_lost(sysid, lost, silent_ms, cmd.time_ms);
                            }
                        }
                        if cmd.refuted_sysid != 0 {
                            telemetry.sighting_refuted(sysid, cmd.refuted_sysid);
                        }
                        let now_informed = cmd.informed != 0;
                        if !informed && now_informed {
                            telemetry.became_informed(sysid, if spotted_this_tick { "firsthand" } else { "relayed" });
                        }
                        informed = now_informed;
                        let report = AgentReport {
                            index: links.index,
                            time_ms: cmd.time_ms,
                            velocity: Vec2::new(cmd.vx, cmd.vy),
                            informed: now_informed,
                            // The board doesn't report what it knows.
                            known: [None; swarm_brain::MAX_TRACKS],
                            role: swarm_brain::Role::None,
                            isolated: cmd.isolated != 0,
                            mode: Mode::from_u8(cmd.mode),
                            target: (cmd.target_id != 0).then_some(cmd.target_id),
                            land: cmd.land != 0,
                            to_charge: cmd.land == 2,
                            position: Vec2::new(cmd.est_x, cmd.est_y),
                            nav: Nav::from_u8(cmd.nav),
                        };
                        if links.report_tx.send(report).await.is_err() {
                            return Ok(());
                        }
                    }
                    MavMessage::SWARM_HIL_TRACE_EVENT(e) => {
                        let Some(event) = hil::trace_event(&e) else { continue };
                        spotted_this_tick |= matches!(event, TraceEvent::Spotted);
                        let (trace_id, span_id) = telemetry.record_event(sysid, event);
                        let span = MavMessage::SWARM_HIL_TRACE_SPAN(SWARM_HIL_TRACE_SPAN_DATA { trace_id, span_id });
                        send_to_board(&body, span).await?;
                    }
                    // The rest of the body link stays off the air.
                    body if hil::is_body_link(&body) => {}
                    // While lying, the board's own (honest) sightings are replaced by the fake.
                    MavMessage::SWARM_TARGET_SIGHTING(_) if lie.is_some() => {}
                    // The board's own frame, signed by the board: on the air as it is.
                    _ => {
                        radio.send_to(raw.raw_bytes(), links.netem_addr).await?;
                    }
                }
            }
        }
    }
}

/// A frame's size on the wire.
fn frame_len(header: MavHeader, msg: &MavMessage) -> usize {
    radio::unsigned(header, msg).raw_bytes().len()
}

const SIMULATOR_HEADER: MavHeader = MavHeader { system_id: SIMULATOR_SYSID, component_id: COMPONENT_ID, sequence: 0 };

/// Queues a message of the board's body link, ahead of any radio frames; its size on the wire.
async fn send_to_board(body: &mpsc::Sender<MavMessage>, msg: MavMessage) -> std::io::Result<usize> {
    let len = frame_len(SIMULATOR_HEADER, &msg);
    body.send(msg).await.map_err(|_| std::io::Error::other("serial link to the board closed"))?;
    Ok(len)
}

/// Writes to the board until the link fails or the bridge is done: the body link first.
async fn write_to_board(
    board: Connection,
    mut body: mpsc::Receiver<MavMessage>,
    mut radio: mpsc::Receiver<MAVLinkV2MessageRaw>,
    slowest_write: Arc<std::sync::atomic::AtomicU64>,
) {
    loop {
        // The body link unsigned (it's a wire), radio frames as they came off the air.
        let frame = tokio::select! {
            biased;
            Some(msg) = body.recv() => radio::unsigned(SIMULATOR_HEADER, &msg),
            Some(frame) = radio.recv() => frame,
            else => return,
        };
        let started = std::time::Instant::now();
        if let Err(e) = board.send_raw(&MAVLinkMessageRaw::V2(frame)).await {
            eprintln!("hil: serial write failed: {e}");
            return;
        }
        slowest_write.fetch_max(started.elapsed().as_micros() as u64, std::sync::atomic::Ordering::Relaxed);
    }
}

/// A token bucket: `rate` bytes a second, up to `RADIO_BURST_S` of it at once.
struct Airtime {
    rate: f64,
    tokens: f64,
    last: std::time::Instant,
}

impl Airtime {
    fn new(rate: f64) -> Self {
        Self { rate, tokens: rate * RADIO_BURST_S, last: std::time::Instant::now() }
    }

    /// Whether `bytes` more fit now (and if so, they're spent).
    fn take(&mut self, bytes: usize) -> bool {
        let now = std::time::Instant::now();
        self.tokens = (self.tokens + now.duration_since(self.last).as_secs_f64() * self.rate).min(self.rate * RADIO_BURST_S);
        self.last = now;
        if self.tokens < bytes as f64 {
            return false;
        }
        self.tokens -= bytes as f64;
        true
    }
}

/// The serial link's load and how quickly the board answers, printed every `LinkStats::EVERY`:
/// the link is `HIL_BAUD`, a tenth of that in bytes a second each way, and a tick is 50 ms.
struct LinkStats {
    since: std::time::Instant,
    to_board: usize,
    from_board: usize,
    radio_frames: u32,
    radio_dropped: u32,
    /// Radio frames for the board by message.
    by_kind: std::collections::HashMap<&'static str, u32>,
    /// The reading the board hasn't answered yet: its time, and when it went out.
    waiting: Option<(u32, std::time::Instant)>,
    slowest: std::time::Duration,
    unanswered: u32,
    /// The longest the world took between two sensor readings (a tick is 50 ms; the board's
    /// watchdog resets it after 500).
    last_reading: Option<std::time::Instant>,
    longest_gap: std::time::Duration,
    /// Readings in a row the board hasn't answered, and how often that reached 10 (half a
    /// second: the board stalled or reset).
    unanswered_run: u32,
    gone: u32,
}

impl LinkStats {
    const EVERY: std::time::Duration = std::time::Duration::from_secs(10);

    fn new() -> Self {
        Self {
            since: std::time::Instant::now(),
            to_board: 0,
            from_board: 0,
            radio_frames: 0,
            radio_dropped: 0,
            by_kind: std::collections::HashMap::new(),
            waiting: None,
            slowest: std::time::Duration::ZERO,
            unanswered: 0,
            last_reading: None,
            longest_gap: std::time::Duration::ZERO,
            unanswered_run: 0,
            gone: 0,
        }
    }

    fn tick_sent(&mut self, time_ms: u32) {
        if self.waiting.is_some() {
            self.unanswered_run += 1;
            if self.unanswered_run == 10 {
                self.gone += 1;
            }
        } else {
            self.unanswered_run = 0;
        }
        let now = std::time::Instant::now();
        if let Some(last) = self.last_reading {
            self.longest_gap = self.longest_gap.max(now - last);
        }
        self.last_reading = Some(now);
        if self.waiting.is_some() {
            self.unanswered += 1;
        }
        self.waiting = Some((time_ms, std::time::Instant::now()));
    }

    fn tick_answered(&mut self, time_ms: u32) {
        if let Some((_, at)) = self.waiting.filter(|(t, _)| *t == time_ms) {
            self.slowest = self.slowest.max(at.elapsed());
            self.waiting = None;
        }
    }

    fn report_now(&mut self, sysid: u8, slowest_write: &std::sync::atomic::AtomicU64) {
        let elapsed = self.since.elapsed();
        if elapsed < Self::EVERY {
            return;
        }
        let per_s = |n: usize| n as f64 / elapsed.as_secs_f64() / 1000.0;
        println!(
            "hil: drone {sysid}'s link: to the board {:.1} KB/s ({:.0} radio frames/s, {} dropped for want of room), from it {:.1} KB/s; slowest tick {} ms, {} unanswered, gone {} times; longest gap between readings {} ms",
            per_s(self.to_board),
            self.radio_frames as f64 / elapsed.as_secs_f64(),
            self.radio_dropped,
            per_s(self.from_board),
            self.slowest.as_millis(),
            self.unanswered,
            self.gone,
            self.longest_gap.as_millis()
        );
        println!("hil:   slowest write to the serial port {} ms", slowest_write.swap(0, std::sync::atomic::Ordering::Relaxed) / 1000);
        let mut kinds: Vec<_> = self.by_kind.iter().collect();
        kinds.sort_by(|a, b| b.1.cmp(a.1));
        let secs = elapsed.as_secs_f64();
        let kinds: Vec<String> = kinds.iter().take(6).map(|(k, n)| format!("{k} {:.0}", **n as f64 / secs)).collect();
        println!("hil:   radio frames/s by kind: {}", kinds.join(", "));
        *self = Self { waiting: self.waiting, last_reading: self.last_reading, unanswered_run: self.unanswered_run, ..Self::new() };
    }
}

/// Forwards a frame to the emulated radio under the header the board sent it with (its own, or
/// for a relay the original sender's), the way a MAVLink router relays traffic between links.
struct AbortOnDrop(tokio::task::JoinHandle<()>);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}
