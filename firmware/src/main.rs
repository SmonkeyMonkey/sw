//! One drone of the swarm, on real hardware. Runs `swarm_brain::Brain` — the exact code every
//! simulated drone runs — and talks to the rest of the swarm only through MAVLink v2 frames on
//! the UART. The backend's HIL bridge (backend/src/hil.rs) is, from the board's point of view,
//! both its body (sensor readings in, velocity commands out) and its radio.
//!
//! The board never initiates a tick: each SWARM_HIL_SENSOR from the simulator is one tick, the
//! same 20 Hz the simulated drones run at. Radio frames that arrive between two readings are
//! queued and handed to the brain on the next tick, exactly like `agent::run` on the PC.
//!
//! One task reads the UART and hands whole frames to the main loop over a channel. Waiting on
//! the channel with a timeout is safe to cancel; waiting on the UART reader itself isn't: a
//! timeout mid-frame would lose the rest of it, and often the next frame too.
//!
//! An independent watchdog resets the board if a tick doesn't come within `WATCHDOG` (it's
//! started with the first one), and so does a panic in release builds.
//!
//! The brain flies only once it has its mission (SWARM_HIL_MISSION, which the bridge sends
//! every second): its keys, to sign its sightings and check everyone else's (see
//! `swarm_brain::auth`), and the field, the base, the swarm's mode. A different one means a new
//! mission, and a fresh brain.

#![no_std]
#![no_main]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::USART3;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_stm32::bind_interrupts;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration, Instant};
use embedded_io_async::Write;
use mavlink_swarm::mavlink_core::{AsyncMavlinkReader, MAVLinkMessageRaw, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion};
use mavlink_swarm::{MavMessage, SWARM_HIL_SENSOR_DATA};
use static_cell::StaticCell;
use swarm_brain::hil::{self, command_msg, sensor_reading, trace_event_msg};
use swarm_brain::hil::Mission;
use swarm_brain::{radio, Brain, COMPONENT_ID, MAX_INBOX};
use defmt_rtt as _;
#[cfg(debug_assertions)]
use panic_probe as _;

/// Release builds: say why over RTT, then reset. A drone stopped dead in a panic is worse than
/// one that's back a moment later with a fresh brain.
#[cfg(not(debug_assertions))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic: {}", defmt::Display2Format(info));
    // A moment for the debug probe to read the log out of RAM, or the reset takes it along.
    cortex_m::asm::delay(216_000 * 100);
    cortex_m::peripheral::SCB::sys_reset()
}

/// The Cortex-M7's instruction and data caches, which embassy leaves off. Without them every
/// instruction comes from flash, 7 wait states at 216 MHz: checking a signature takes 21 ms
/// instead of 6, and three in a tick no longer fit in its 50 ms. Nothing here uses DMA, so the
/// data cache can't go stale under a peripheral.
fn enable_caches() {
    // Safe: only this, at startup, touches SCB and CPUID.
    let mut core = unsafe { cortex_m::Peripherals::steal() };
    core.SCB.enable_icache();
    core.SCB.enable_dcache(&mut core.CPUID);
}

/// Why the board last reset, from the reset flags (then cleared, for next time).
fn reset_cause() -> &'static str {
    let rcc = embassy_stm32::pac::RCC;
    let csr = rcc.csr().read();
    let cause = if csr.wdgrstf() {
        "watchdog: no tick for 500 ms"
    } else if csr.sftrstf() {
        "software: a panic in a release build, or the debug probe (flashing resets this way too)"
    } else if csr.borrstf() || csr.porrstf() {
        "power on"
    } else if csr.padrstf() {
        "reset pin (button or debug probe)"
    } else {
        "other"
    };
    rcc.csr().modify(|w| w.set_rmvf(true));
    cause
}

defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

bind_interrupts!(struct Irqs {
    USART3 => BufferedInterruptHandler<USART3>;
});

/// Must match backend/src/main.rs HIL_BAUD.
const BAUD: u32 = 1_000_000;

/// How long to wait for the simulator to answer a SWARM_HIL_TRACE_EVENT before broadcasting
/// without trace context. A round trip is ~2 ms; the world waits 25 ms for our command.
const SPAN_TIMEOUT: Duration = Duration::from_millis(10);

/// No tick for this long and the watchdog resets the board: ten ticks' worth.
const WATCHDOG_US: u32 = 500_000;

/// Radio frames buffered between two ticks (see `swarm_brain::MAX_INBOX`): ~27 KB.
type Inbox = heapless::Vec<(MavHeader, MavMessage), MAX_INBOX>;

/// Whole frames from the UART reader task to the main loop. A few ticks' worth of radio
/// traffic, behind the UART's own 8 KB ring buffer (80 ms of the link flat out).
static FRAMES: Channel<CriticalSectionRawMutex, MAVLinkV2MessageRaw, 8> = Channel::new();

/// What the UART reader couldn't make a frame of (bad CRC, cut short, a UART error), since the
/// last report: bytes lost on the link.
static BAD_FRAMES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// How often the reader found the main loop's channel full and had to wait, since the last
/// report (the UART's ring buffer fills meanwhile).
static CHANNEL_FULL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// What the main loop is doing right now, kept across a reset (`.uninit` isn't zeroed at boot):
/// after the watchdog fires, where it got stuck. The high bits mark it as ours, not power-on
/// garbage.
#[link_section = ".uninit.STAGE"]
static STAGE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
const STAGE_MARK: u32 = 0x5A6E_0000;

#[derive(Clone, Copy)]
#[repr(u32)]
enum Stage {
    WaitingForFrame = 1,
    Thinking,
    SendingTraceEvent,
    WaitingForSpan,
    Building,
    SendingRadio,
    SendingCommand,
    Bookkeeping,
}

fn stage(s: Stage) {
    STAGE.store(STAGE_MARK | s as u32, core::sync::atomic::Ordering::Relaxed);
}

/// Where the main loop was when the board last reset.
fn last_stage() -> &'static str {
    let v = STAGE.load(core::sync::atomic::Ordering::Relaxed);
    if v & 0xFFFF_0000 != STAGE_MARK {
        return "unknown";
    }
    match v & 0xFFFF {
        1 => "waiting for a frame from the UART",
        2 => "thinking (on_tick)",
        3 => "sending the trace event",
        4 => "waiting for the span",
        5 => "building the outgoing frames",
        6 => "sending radio frames",
        7 => "sending the command",
        8 => "bookkeeping and logging",
        _ => "unknown",
    }
}

// Big state lives in statics rather than in the main task's future: the brain is ~69 KB (its
// fixed-size neighbor, track, vote and certificate tables) and each MavMessage takes 280 bytes.
static TX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 8192]> = StaticCell::new();
static BRAIN: StaticCell<Option<Brain>> = StaticCell::new();
static INBOX: StaticCell<Inbox> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(clock_config());
    info!("booted; last reset: {}, in the middle of {}", reset_cause(), last_stage());
    enable_caches();

    // LD1 (green) toggles every 10 ticks: the brain is running. LD2 (blue) is on while the
    // brain knows where the target is.
    let mut tick_led = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut informed_led = Output::new(p.PB7, Level::Low, Speed::Low);

    // USART3 on PD8/PD9 is wired to the ST-Link, which shows it on the host as a virtual COM
    // port over the same USB cable used for flashing.
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = BAUD;
    let uart = BufferedUart::new(
        p.USART3,
        p.PD9,
        p.PD8,
        TX_BUF.init([0; 4096]),
        RX_BUF.init([0; 8192]),
        Irqs,
        uart_config,
    )
    .unwrap();
    let (mut tx, rx) = uart.split();
    spawner.spawn(read_frames(rx).unwrap());
    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_US);
    let mut watching = false;

    let brain_slot = BRAIN.init(None);
    let inbox = INBOX.init(heapless::Vec::new());
    let mut sequence: u8 = 0;
    let mut ticks: u32 = 0;
    let mut dropped: u32 = 0;
    let mut last_time_ms: u32 = 0;
    // A reading that came in while we were still waiting on the last tick's span: the next tick.
    let mut next_sensor: Option<SWARM_HIL_SENSOR_DATA> = None;
    // This mission, and the last one just in from the bridge.
    let mut mission: Option<Mission> = None;
    let mut new_mission: Option<Mission> = None;
    let mut idle_ticks: u32 = 0;
    let mut times = TickTimes::default();
    let mut last_tick_ms: u32 = 0;
    // Signs our radio frames with the mission's link key (see `swarm_brain::radio`); a new one
    // with every new brain.
    let mut link: Option<radio::Signer> = None;
    // Radio frames dropped for not being signed with it, since the start.
    let mut unsigned: u32 = 0;

    loop {
        if let Some(m) = new_mission.take() {
            if mission != Some(m) {
                info!("mission {}: field {}x{} m, base ({}, {})", m.keys.mission, m.field_half.x * 2.0, m.field_half.y * 2.0, m.home.x, m.home.y);
                mission = Some(m);
                *brain_slot = None;
                inbox.clear();
            }
        }
        let sensor = match next_sensor.take() {
            Some(sensor) => sensor,
            None => {
                stage(Stage::WaitingForFrame);
                let raw = FRAMES.receive().await;
                stage(Stage::Bookkeeping);
                let Some((header, msg)) = radio::parse(&raw) else { continue };
                match msg {
                    MavMessage::SWARM_HIL_SENSOR(sensor) => sensor,
                    // A span that arrived after we gave up waiting for it.
                    MavMessage::SWARM_HIL_TRACE_SPAN(_) => continue,
                    MavMessage::SWARM_HIL_MISSION(m) => {
                        new_mission = hil::mission(&m).or(new_mission);
                        continue;
                    }
                    msg => {
                        let radio_in = RadioIn { inbox: &mut *inbox, dropped: &mut dropped, unsigned: &mut unsigned };
                        radio_in.take(&raw, header, msg, mission.as_ref(), last_time_ms);
                        continue;
                    }
                }
            }
        };
        if !watching {
            watchdog.unleash();
            watching = true;
        }
        watchdog.pet();

        // The simulator's clock restarts with every mission: time going backwards means the
        // swarm was reset, so start over with a fresh brain instead of last mission's memory.
        if sensor.time_ms < last_time_ms {
            info!("new mission");
            *brain_slot = None;
            inbox.clear();
        }
        last_time_ms = sensor.time_ms;
        // No mission for this drone yet: sit the tick out. (No command back: the simulator
        // holds the airframe where it is.)
        let Some(mission) = mission.filter(|m| m.keys.sysid == sensor.sysid) else {
            if idle_ticks.is_multiple_of(20) {
                warn!("no mission for drone {} yet", sensor.sysid);
            }
            idle_ticks = idle_ticks.wrapping_add(1);
            inbox.clear();
            continue;
        };
        if brain_slot.is_none() {
            link = Some(radio::Signer::new(mission.keys.link, sensor.sysid));
        }
        let brain = brain_slot.get_or_insert_with(|| {
            info!("flying as drone {}", sensor.sysid);
            // The ground station can still switch the mode over the radio.
            Brain::new(sensor.sysid, mission.config(), seed(), mission.keys)
        });
        let reading = sensor_reading(&sensor);
        // Readings come every 50 ms of simulator time: a bigger step is readings that never
        // made it here.
        if last_tick_ms > 0 && reading.time_ms > last_tick_ms + 75 {
            times.lost_readings += (reading.time_ms - last_tick_ms) / 50 - 1;
        }
        last_tick_ms = reading.time_ms;

        let started = Instant::now();
        let work_before = brain.crypto_work();
        times.inbox = times.inbox.max(inbox.len());
        stage(Stage::Thinking);
        let out = brain.on_tick(&reading, inbox);
        inbox.clear();
        let thought = Instant::now();

        if let Some(event) = out.event {
            stage(Stage::SendingTraceEvent);
            send(&mut tx, brain.sysid(), &mut sequence, &trace_event_msg(event)).await;
            stage(Stage::WaitingForSpan);
            let radio_in = RadioIn { inbox: &mut *inbox, dropped: &mut dropped, unsigned: &mut unsigned };
            match with_timeout(SPAN_TIMEOUT, wait_for_span(radio_in, &mut new_mission, &mission, reading.time_ms)).await {
                Ok(Span::Got(trace_id, span_id)) => brain.attach_span(trace_id, span_id),
                Ok(Span::NextTick(sensor)) => {
                    warn!("next tick before the span came back; broadcasting without trace context");
                    next_sensor = Some(sensor);
                }
                Err(_) => warn!("no span from the simulator; broadcasting without trace context"),
            }
        }

        let span_done = Instant::now();
        stage(Stage::Building);
        let frames = brain.outgoing_frames(reading.time_ms);
        let built = Instant::now();
        stage(Stage::SendingRadio);
        let link = link.as_mut().expect("made with the brain");
        for (relayed, msg) in frames.into_iter().flatten() {
            // A relay goes out under the header it came with (see `swarm_brain::Frame`).
            let header = relayed.unwrap_or_else(|| own_header(brain.sysid(), &mut sequence));
            send_frame(&mut tx, &link.frame(header, &msg, reading.time_ms)).await;
        }
        let command = command_msg(brain, &reading, &out);
        stage(Stage::SendingCommand);
        send(&mut tx, brain.sysid(), &mut sequence, &command).await;
        stage(Stage::Bookkeeping);
        let work_after = brain.crypto_work();
        times.add(started, thought, span_done, built, Instant::now(), work_after.0 - work_before.0, work_after.1 - work_before.1);

        ticks = ticks.wrapping_add(1);
        if ticks.is_multiple_of(10) {
            tick_led.toggle();
        }
        informed_led.set_level(if brain.informed() { Level::High } else { Level::Low });
        // Every 5 s, not every second: writing the log masks interrupts long enough for the UART
        // to drop a byte at 1 Mbaud (an "Overrun error" right after each line).
        if ticks.is_multiple_of(100) {
            times.report();
        }
        if ticks.is_multiple_of(100) {
            info!(
                "t={}ms pos=({},{}) mode={} informed={} dropped={} unsigned={} outbox_dropped={} sightings forged={} unchecked={} orders forged={}",
                reading.time_ms,
                reading.own_pos.x,
                reading.own_pos.y,
                brain.mode().to_u8(),
                brain.informed(),
                dropped,
                unsigned,
                brain.outbox_dropped(),
                brain.sightings_rejected().0,
                brain.sightings_rejected().1,
                brain.orders_rejected()
            );
        }
    }
}

/// Where a tick's time goes, the slowest of each over 5 s of ticks (a tick has 50 ms).
#[derive(Default)]
struct TickTimes {
    think_us: u64,
    span_us: u64,
    build_us: u64,
    send_us: u64,
    total_us: u64,
    over_budget: u32,
    inbox: usize,
    checks: u32,
    signatures: u32,
    lost_readings: u32,
}

impl TickTimes {
    #[allow(clippy::too_many_arguments)]
    fn add(&mut self, start: Instant, thought: Instant, span: Instant, built: Instant, sent: Instant, checks: u32, signatures: u32) {
        self.think_us = self.think_us.max((thought - start).as_micros());
        self.span_us = self.span_us.max((span - thought).as_micros());
        self.build_us = self.build_us.max((built - span).as_micros());
        self.send_us = self.send_us.max((sent - built).as_micros());
        let total = (sent - start).as_micros();
        self.total_us = self.total_us.max(total);
        self.over_budget += (total > 50_000) as u32;
        self.checks += checks;
        self.signatures += signatures;
    }

    fn report(&mut self) {
        info!(
            "tick max {} ms (think {} span {} build {} send {}), {} over 50 ms; inbox max {}; checks {} signatures {}; bad frames {}; readings lost {}; channel full {}",
            self.total_us / 1000,
            self.think_us / 1000,
            self.span_us / 1000,
            self.build_us / 1000,
            self.send_us / 1000,
            self.over_budget,
            self.inbox,
            self.checks,
            self.signatures,
            BAD_FRAMES.swap(0, core::sync::atomic::Ordering::Relaxed),
            self.lost_readings,
            CHANNEL_FULL.swap(0, core::sync::atomic::Ordering::Relaxed)
        );
        *self = Self::default();
    }
}

/// 216 MHz SYSCLK from the ST-Link's 8 MHz clock output (HSE bypass). Mostly so a brain tick —
/// a few passes over 256-entry tables — takes microseconds rather than milliseconds; it also
/// gives USART3 a 54 MHz kernel clock, which divides 1 Mbaud exactly.
fn clock_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Bypass,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL216,
        divp: Some(PllPDiv::DIV2), // 8 / 4 * 216 / 2 = 216 MHz
        divq: None,
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4; // 54 MHz, the APB1 maximum
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 108 MHz, the APB2 maximum
    config
}

/// Reads the UART for good: whole frames into `FRAMES`. Bad CRCs and unknown messages are
/// skipped, like lost packets.
#[embassy_executor::task]
async fn read_frames(rx: BufferedUartRx<'static>) {
    let mut reader = AsyncMavlinkReader::new(rx);
    loop {
        // Whole frames, signatures and all: the main loop checks radio frames' (see
        // `RadioIn::take`).
        match reader.read_raw_message::<MavMessage>(MavlinkVersion::V2).await {
            Ok(MAVLinkMessageRaw::V1(_)) => {}
            Ok(MAVLinkMessageRaw::V2(frame)) => {
                if let Err(embassy_sync::channel::TrySendError::Full(frame)) = FRAMES.try_send(frame) {
                    CHANNEL_FULL.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    FRAMES.send(frame).await;
                }
            }
            Err(_) => {
                BAD_FRAMES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}

enum Span {
    Got([u8; 16], [u8; 8]),
    /// The next tick's reading came first: kept for it, not lost with its tick.
    NextTick(SWARM_HIL_SENSOR_DATA),
}

/// Takes frames until the simulator's SWARM_HIL_TRACE_SPAN arrives; radio frames that come in
/// meanwhile belong to the next tick.
async fn wait_for_span(mut radio_in: RadioIn<'_>, new_mission: &mut Option<Mission>, mission: &Mission, now_ms: u32) -> Span {
    loop {
        let raw = FRAMES.receive().await;
        let Some((header, msg)) = radio::parse(&raw) else { continue };
        match msg {
            MavMessage::SWARM_HIL_TRACE_SPAN(span) => return Span::Got(span.trace_id, span.span_id),
            // The next tick started before the span came back — shouldn't happen at 20 Hz.
            MavMessage::SWARM_HIL_SENSOR(sensor) => return Span::NextTick(sensor),
            MavMessage::SWARM_HIL_MISSION(m) => *new_mission = hil::mission(&m).or(*new_mission),
            msg => radio_in.reborrow().take(&raw, header, msg, Some(mission), now_ms),
        }
    }
}

/// Where radio frames go: the inbox for the next tick, and the counts of those lost.
struct RadioIn<'a> {
    inbox: &'a mut Inbox,
    dropped: &'a mut u32,
    unsigned: &'a mut u32,
}

impl RadioIn<'_> {
    fn reborrow(&mut self) -> RadioIn<'_> {
        RadioIn { inbox: self.inbox, dropped: self.dropped, unsigned: self.unsigned }
    }

    /// A radio frame, if it's signed with this mission's link key, within a second of our clock
    /// (see `swarm_brain::radio`): anything else on the air isn't the swarm's. No mission yet,
    /// no key, and nothing's heard.
    fn take(self, raw: &MAVLinkV2MessageRaw, header: MavHeader, msg: MavMessage, mission: Option<&Mission>, now_ms: u32) {
        if !mission.is_some_and(|m| radio::signed(raw, &m.keys.link, now_ms)) {
            *self.unsigned += 1;
            return;
        }
        if self.inbox.push((header, msg)).is_err() {
            *self.dropped += 1;
        }
    }
}

fn own_header(sysid: u8, sequence: &mut u8) -> MavHeader {
    let header = MavHeader { system_id: sysid, component_id: COMPONENT_ID, sequence: *sequence };
    *sequence = sequence.wrapping_add(1);
    header
}

/// A frame of the body link (to the simulator): a wire, not the radio, so unsigned.
async fn send(tx: &mut BufferedUartTx<'static>, sysid: u8, sequence: &mut u8, msg: &MavMessage) {
    let header = own_header(sysid, sequence);
    send_frame(tx, &radio::unsigned(header, msg)).await;
}

/// How long one try at writing a frame gets, and how many tries before it's dropped.
const WRITE_TRY: Duration = Duration::from_millis(2);
const WRITE_TRIES: u32 = 50;

/// Writes a frame, trying again and again while the UART's transmit buffer has no room for it.
///
/// Not just `tx.write_all(...).await`: in embassy-stm32 0.6, `BufferedUartTx::write`
/// answers "pending" without registering a waker when the frame doesn't fit in the buffer's
/// free space, and then nothing ever wakes it: the main loop would hang right there, mid-send,
/// until the watchdog reset the board (with many drones around, a tick's relays fill the buffer
/// often). A write that times out wrote nothing (it copies the whole frame or none of it), so it
/// can simply be tried again once the interrupt has sent some of the buffer out.
async fn send_frame(tx: &mut BufferedUartTx<'static>, frame: &MAVLinkV2MessageRaw) {
    for _ in 0..WRITE_TRIES {
        match with_timeout(WRITE_TRY, tx.write_all(frame.raw_bytes())).await {
            Ok(Ok(_)) => return,
            Ok(Err(_)) => {
                warn!("uart write failed");
                return;
            }
            Err(_) => {}
        }
    }
    warn!("uart: no room to send for 100 ms, frame dropped");
}

/// Seed for the brain's wander PRNG, from the chip's factory-unique 96-bit ID, so two boards
/// flying in the same swarm wouldn't wander identically.
fn seed() -> u32 {
    let uid = embassy_stm32::uid::uid();
    let mut seed = 0x811C_9DC5u32; // FNV-1a
    for byte in uid {
        seed = (seed ^ byte as u32).wrapping_mul(0x0100_0193);
    }
    seed
}
