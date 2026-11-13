# Swarm

A simulator for drone-swarm algorithms, where one of the drones can be a real microcontroller.

> This is a very basic swarming visualizer: a top-down 2D view for watching and debugging the
> algorithms, not a realistic flight simulation or 3D rendering.

A swarm of quadcopters searches a field for moving targets ("find the mice in the field"). The
drones spread the news by radio, agree on whether a report is real, and split up so each target
gets a couple of drones holding a ring around it while the rest keep searching. With batteries
on, they go back to the base to charge on their own, and charged airframes take over from them,
so the swarm can keep searching with nobody at the controls.

There is no central controller. Every drone runs the same brain, and drones talk to each other
only through **MAVLink v2 frames over UDP**, routed by an emulated radio with range, latency,
jitter and loss. One of the drones can be an **STM32F767 board running that brain on bare
metal**, over UART; the rest of the swarm can't tell it apart from the simulated drones.

## Architecture

```
                ┌──────────────────────── backend (tokio) ─────────────────────────┐
                │                                                                  │
 viewer (Bevy) ◄┼─ ws ─ world: ground truth, physics, autopilots, batteries ───┐   │
  faults,       │         │ sensor reading ▲ velocity command (20 Hz)          │   │
  simulation ──►│         ▼                │                                   │   │
                │   drone 2..N: swarm-brain in a tokio task ◄─ UDP ─► netem ◄──┘   │
                │   drone 1:   HIL bridge ◄─ UART, MAVLink ─► STM32: swarm-brain   │
                │                                       (radio range, loss and     │
                │   base: launch pads, charging,         latency from true         │
                │         keys, operator link ◄──┐       positions)                │
                └────────────────────────────────┼─── OTLP ─► Grafana/Prometheus/Tempo
                                                 │
            ground station (Bevy) ── wire (UDP) ─┘   + radio (through netem) ─► QGroundControl
```

The pieces, and what each one is allowed to know:

- **The brain** ([`brain/`](brain/src/lib.rs)) is one drone's decision-making. It's a `no_std`,
  allocation-free crate that compiles unchanged for the PC and for the Cortex-M7. Each tick it
  takes the drone's own sensor reading (`SensorReading`: GPS fix, own velocity, targets in view,
  drones in ranging distance, battery charge) and the MAVLink frames heard since the last tick,
  and returns a horizontal velocity and the frames to send. It never sees ground truth.
- **The backend** ([`backend/`](backend/src/main.rs)) is everything around the drones:
  - **the world** ([`world.rs`](backend/src/world.rs)): ground truth for drones and targets, the
    sensors (GPS with realistic error, ranging, the target sensor), batteries, the base's pads;
  - **physics** ([`physics.rs`](backend/src/physics.rs)): a Rapier rigid body per airframe and a
    PX4-style autopilot under each brain;
  - **the radio** ([`netem.rs`](backend/src/netem.rs)): delivers each frame only to drones in
    range of its sender's true position, with latency, jitter and loss;
  - **the drones** ([`agent.rs`](backend/src/agent.rs)): one tokio task and UDP socket per brain;
    or the **HIL bridge** ([`hil.rs`](backend/src/hil.rs)) for the drone the board flies;
  - **the base** ([`base.rs`](backend/src/base.rs)): the launcher and dock, the fleet's
    certificate authority, and the wired link to the ground station;
  - telemetry (OTLP).
- **The firmware** ([`firmware/`](firmware/src/main.rs)) runs the same brain on a Nucleo-F767ZI
  (Embassy), talking MAVLink over the ST-Link's virtual COM port.
- **The ground station** ([`gcs/`](gcs/src/main.rs)) is the operator's console. Like
  QGroundControl, it knows only what reaches it over MAVLink: the swarm's statuses and sightings
  by radio, the base by wire. It bridges the swarm to QGroundControl.
- **The viewer** ([`viz/`](viz/src/main.rs)) shows the simulator's ground truth, and controls the
  simulation itself: faults, weather, and how the world is simulated.
- **The MAVLink dialect** ([`mavlink-swarm/`](mavlink-swarm/dialect/swarm.xml)): `common` plus the
  swarm's own messages (sightings, certificates, signed orders, statuses, the base's messages, and
  the `SWARM_HIL_*` body link between the board and the simulator).

Everything but the firmware is one Cargo workspace ([`Cargo.toml`](Cargo.toml)), with one
`target/`, one lock file and one version of each shared dependency. The firmware builds for the
board (`thumbv7em`) with its own profiles, outside it.

## What each drone does

The brain is split by topic: [`lib.rs`](brain/src/lib.rs) has the tick (`on_tick`, `decide`), and
the rest is in [`config`](brain/src/config.rs), [`tracks`](brain/src/tracks.rs),
[`respond`](brain/src/respond.rs), [`formation`](brain/src/formation.rs),
[`search`](brain/src/search.rs), [`relay`](brain/src/relay.rs), [`nav`](brain/src/nav.rs),
[`avoid`](brain/src/avoid.rs), [`battery`](brain/src/battery.rs),
[`operator`](brain/src/operator.rs), [`outgoing`](brain/src/outgoing.rs),
[`auth`](brain/src/auth.rs) and [`radio`](brain/src/radio.rs).

### Searching

- **Two modes** (`SwarmMode`, picked in the ground station and sent to every drone):
  - **Autonomous** (default): off the leash. The free drones sweep the field in formation, rings
    follow their targets wherever they run, and the swarm is in contact with the base only near
    it.
  - **Connected**: always in contact with the base. Drones keep within reach of a neighbor closer
    to the base, relay chains run out to every ring, and the free drones sweep the field in rays
    out from the base, like a radar's beam. Slower, but the operator reaches every drone.
- **Formation** (autonomous). The free drones fly side by side, 5 m apart (their sensors' strips
  meet, and each is in radio range of the next), mowing the field in lanes along its longer side.
  - The drones are split into fronts by system ID, each with its own stretch of the field. A long
    front is cut into segments of drones close enough to hear of each other, lined up from the
    middle of the front outwards so it stays one line however many are busy. A front with few
    drones free closes up into a shorter line and mows its stretch in more lanes.
  - Where a front is goes by the shared mission clock alone, so every drone works out the same
    places without negotiating. Who's in a front is each drone's own roster, from whom it hears
    free; it changes slowly (joining after 1 s, leaving after 5 s).
  - Adjacent fronts set out from opposite ends, so they meet at their boundary every pass.
  - A drone that loses contact keeps flying its own place by the clock. One that strayed from its
    front (out of range of all of it) rejoins it. Under jamming, with nobody heard but drones in
    ranging distance, it takes the radio for gone, not its front, and holds its place.
- **Coverage map** (the other sweeps). Every drone keeps a map of when each part of the field was
  last flown over, marked from its own position and its neighbors' broadcasts, and heads for the
  stalest part no free neighbor is closer to.
- **Last seen here.** A target nobody has seen for 5 s is remembered and relayed for another 30 s
  as a hint: drones search where it has probably got to.

### Targets

- **Sightings** are flooded drone to drone (`SWARM_TARGET_SIGHTING`). The freshest *observation*
  wins, not the freshest relay, and freshness travels as an age, since the drones' clocks aren't
  synchronized. The observer's velocity estimate rides along, so receivers act on where the
  target should be by now.
- **Tracks.** A sensor sees *a* target, not which one. Each drone keeps tracks and associates
  every observation with one by predicted position. Every track has a swarm-wide name (the first
  spotter's system ID plus a counter); two tracks of one target merge into the lower name. Two
  targets seen at the same moment more than 4 m apart stay two, however close they get later.
- **Consensus.** A drone commits to a target once two different drones have seen it firsthand,
  or it sees it itself. A single report only draws nearby drones to check it; one that finds
  nothing there distrusts the reporter for a while.
- **Who goes.** About `responders` drones per target (2 by default; the operator sets it), the
  ones best placed for it, each deciding from its own neighbor table with no negotiation. In the
  formation, the drones whose places are nearest it go, and each front keeps its two highest
  system IDs searching. A drone that sees a target nobody is on, outside its own stretch or as
  its front's reserve, watches it until one whose target it is arrives.
- **Rings.** Responders hold a 1.5 m ring around the target, inside sensor range. With more
  drones than fit, they stack outward. Too few drones for every target: drones leave a full ring
  for one that's two short. Optionally ("adapt to the number of mice"), each target gets a share
  of the drones known, leaving some for the targets not found yet.

### Staying safe

- **Collision avoidance.** Each drone predicts every neighbor's closest approach up to 1 s ahead
  and steers clear, and never closes on a neighbor faster than it could brake (both of a pair
  take half the correction, as in ORCA). It goes by the drones it ranges within 5 m, or by their
  broadcast positions without ranging.
- **Where am I.** The brain doesn't simply believe its GPS: it checks the fix against the
  neighbors it ranges, falls back on them (cooperative localization) or on dead reckoning, and
  so catches a spoofed GPS that its neighbors place elsewhere.
- **Failure detection.** `HEARTBEAT` at 1 Hz; a neighbor silent for 3.5 s is lost.

### Security

- **Signed sightings** ([`auth.rs`](brain/src/auth.rs)). The base is the fleet's certificate
  authority: each mission, every drone gets a fresh Ed25519 key and a certificate. Drones sign
  their own observations, so a captured drone can lie only in its own name: one witness, never a
  quorum. Certificates travel with the sightings.
- **Signed orders.** Only the operator gives orders: drones carry out and relay only
  `SWARM_SIGNED_COMMAND` / `SWARM_SIGNED_STICKS` signed with the mission's operator key, dated on
  the mission clock (no replays). The operator can revoke a captured drone; the swarm then
  ignores it.
- **Link signing** ([`radio.rs`](brain/src/radio.rs)). Every frame on the air carries a MAVLink 2
  signature with the mission's link key, so nobody outside the swarm can inject or replay frames.
- Signature checks are budgeted (3 a tick: 6 ms each on the board).

### The battery

Each drone knows its charge, and works out how fast it's going down from what it measures. It
goes home when what's left is what the flight home takes, with a margin, and a little more by
system ID so drones launched together don't all come back together. To the swarm, a drone going
to charge is one sent home: the line closes up without it and its target gets another drone.

### The radio

Beacons stretch in a crowd, and statuses and sightings are relayed only if few neighbors already
did, so what each drone sends grows little with the size of the swarm.

## The base

- **Launching.** Drones take off one a second from pads in rows along the field's edge by the
  base, outside the field, and climb before moving sideways.
- **Docking.** A drone back from a return-to-launch asks to land 5 m from the base, and the base
  guides it onto the nearest free pad.
- **Charging.** Airframes on the pads charge. A charged airframe takes off in the place of each
  drone that came back to charge, flying as it (its system ID and keys), so as many stay in the
  air as were launched. A drone the operator sent home isn't replaced.
- **Authority.** Each mission, the base issues the drones' keys, derives the operator key and
  hands it to the ground station over the wire only, and signs revocations.

## The ground station

[`gcs/`](gcs/src/main.rs) sits at the base, on the same radio as the drones (system ID 255); the
swarm relays for it. Every drone broadcasts a `SWARM_DRONE_STATUS`, relayed up to 6 hops.

- **Field.** Width and height, set before launching (sliders or dragging the map), starting a new
  mission on it.
- **Fleet.** How many drones the field needs (the same sums the drones do), launching them, the
  mode, drones per target, replacing lost drones.
- **Charging.** What's on the base's pads, the lowest charge in the air, how many spares the base
  keeps, and whether charged airframes take over from drones down to charge.
- **Orders** to a drone: go to a spot, hold, return to base, hand back to the swarm; cancel a
  target; revoke a drone. Commands repeat until acknowledged; a drone acknowledges in its status,
  since a relayed `COMMAND_ACK` loses its sender.
- **The remote** ([`remote.rs`](gcs/src/remote.rs)), styled after a DJI controller: a light per
  drone, flying one by hand (`MANUAL_CONTROL`, relayed by the swarm), with a hover-then-rejoin
  failsafe when the sticks' signal is lost.
- **QGroundControl** ([`qgc.rs`](gcs/src/qgc.rs)). The ground station feeds the drones it hears to
  QGC on UDP 14550, as generic MAVLink vehicles with positions in latitude/longitude around an
  arbitrary origin; go-to, hold, return to launch and resume reach the drone through the swarm.

| Command | MAVLink |
|---|---|
| Go to a spot | `MAV_CMD_DO_REPOSITION` (field x/y in params 5/6) |
| Hold | `MAV_CMD_NAV_LOITER_UNLIM` |
| Return to base | `MAV_CMD_NAV_RETURN_TO_LAUNCH` |
| Hand back to the swarm | `MAV_CMD_MISSION_START` |
| Cancel a target | `MAV_CMD_SWARM_CANCEL_TARGET` |
| Drones per target | `MAV_CMD_SWARM_SET_RESPONDERS` |
| Mode | `MAV_CMD_SWARM_SET_MODE` |
| Launch N drones | `MAV_CMD_SWARM_LAUNCH` (base link) |
| New field | `MAV_CMD_SWARM_SET_FIELD` (base link) |
| Spares on the pads | `MAV_CMD_SWARM_SET_SPARES` (base link) |
| Swap in charged drones | `MAV_CMD_SWARM_SET_ROTATION` (base link) |
| Revoke a drone | `MAV_CMD_SWARM_REVOKE` (base link) |

## The simulator

- **GPS** ([`gps.rs`](backend/src/gps.rs)): slowly drifting error, most of it shared by all drones,
  about 2 m on average; 5 Hz.
- **Sensors**: targets within 2.5 m, seen relative to the drone; other drones within 5 m (ranging,
  as UWB or cameras would).
- **Radio**: 8 m range, latency, jitter, 10% loss. No bandwidth limit or collisions.
- **Physics** ([`physics.rs`](backend/src/physics.rs)): a Mavic-class rigid body per airframe; a
  PX4-style cascade (velocity → tilt and thrust → rates → four rotors) at 400 Hz; drag relative to
  the air, so wind pushes; collisions between airframes. A brain that goes silent for 3 s gets its
  drone flown home by the autopilot, as PX4's offboard-loss failsafe does.
- **Batteries**: drain in the air (more flying fast), charge on the pads; a drone that runs out
  falls.

**The viewer** draws the drones, targets, radio links, coverage, battery gauges and the pads, and
has two panels:

- **Faults**: kill a drone, make one lie (broadcast a fake signed sighting and forged orders),
  jam or spoof GPS, add and remove drones and mice, wind.
- **Simulation**, for the next mission: GPS error, ranging between drones, batteries (minutes a
  charge flies, minutes to charge), and a random-walk search to compare against.

## Observability

Everything exports over OTLP to the `grafana/otel-lgtm` stack ([`observability/`](observability/docker-compose.yml)),
with a provisioned dashboard:

- **Traces.** A sighting's spread through the swarm is one distributed trace: every drone that
  learns of it adds a span whose parent is the drone it heard it from (the trace context rides in
  the MAVLink frame). A drone's death is a trace too, with a span per neighbor that notices.
- **Metrics.** Radio (frames, loss, latency), news freshness, failure detection (latency, false
  alarms), consensus (liars caught, honest drones wrongly distrusted), security (forgeries
  refused), flight (collisions, tilt, wind), coverage and targets.

## Running it

`make` builds and starts the backend, the viewer and the ground station together; Ctrl-C stops
them all. Each one's output is in the terminal, prefixed with its name, and in `logs/`.

```sh
make                                       # backend + viewer + ground station
make run ARGS="--field 100x30 --drones 16" # with flags for the backend
make hil                                   # flash the board, fly it as drone 1
make fake                                  # the host-side fake board as drone 1
make flash                                 # just flash the board and watch its log
make obs                                   # Grafana on http://localhost:3000 (make obs-down)
make build | test | smoke
```

The backend's flags:

| Flag | What | Default |
|---|---|---|
| `--field 40x30` | Field size, m; the base is in its lower left corner | `30x30` |
| `--drones N` | Drones launched at the start (0: launch from the ground station) | 12 |
| `--mode autonomous\|connected` | The swarm's mode | `autonomous` |
| `--sweep formation\|cells\|radar` | Override the mode's sweep, to compare | the mode's |
| `--battery-min 20\|off` | Batteries: minutes a full charge flies | 12 |
| `--charge-min 60` | Minutes to charge from empty | 2 |
| `--spares 8` | Charged airframes on the pads besides those launched | 0 |
| `--gps on\|off` | GPS error | `off` |
| `--proximity on\|off` | Ranging between drones | `on` |
| `--search coverage\|random` | Random-walk search, to compare | `coverage` |
| `--hil <port>` | Fly drone 1 on the board (or `tcpin:127.0.0.1:5760` for the fake board) | |

`--battery-min`, `--charge-min`, `--gps`, `--proximity` and `--search` set the first mission; the viewer's
Simulation panel changes them for the next. Kept up for good, each drone in the air needs about
`charge / flight` more airframes charging on the pads; the backend prints how many.

### With the board as drone 1

```sh
cd firmware && cargo run --release -- --no-catch-reset   # flash (probe-rs), watch defmt logs
cd backend && cargo run -- --hil /dev/tty.usbmodem103   # the board's port: ls /dev/tty.usbmodem*
```

- One USB cable: the board talks to the backend over the ST-Link's virtual COM port, at 1 Mbaud.
  The bridge sends the board's body link (sensor readings, its mission) ahead of radio frames and
  caps radio traffic, dropping the rest like a radio out of airtime.
- The board gets its mission (keys, field, base, mode) from the bridge (`SWARM_HIL_MISSION`,
  never from the air), and flies only once it has it. `SWARM_HIL_*` frames that come in from the
  air are dropped.
- A watchdog resets the board if no tick comes for 500 ms; a release build resets on a panic too.
  LD1 blinks while the brain ticks.
- Two things that cost a day: Embassy leaves the Cortex-M7's caches off (a signature check took
  21 ms instead of 6); and `BufferedUartTx::write` in embassy-stm32 0.6 can return pending
  without registering a waker, hanging the task. The firmware turns the caches on and retries a
  write on a 2 ms timer.
- No board? [`fake_board`](backend/examples/fake_board.rs) runs the firmware's loop on the host:
  `make fake`.

## Layout

| Crate | What |
|---|---|
| [`brain/`](brain/src/lib.rs) | One drone's decision-making, `no_std`; the tests in [`src/tests/`](brain/src/tests/mod.rs), with the swarm harness in [`harness.rs`](brain/src/tests/harness.rs). |
| [`backend/`](backend/src/main.rs) | World, physics, radio, drone runners, HIL bridge, base, telemetry. |
| [`firmware/`](firmware/src/main.rs) | Embassy firmware for the Nucleo-F767ZI. |
| [`gcs/`](gcs/src/main.rs) | The operator's ground station and QGroundControl bridge. |
| [`viz/`](viz/src/main.rs) | The simulator's viewer and control panels. |
| [`mavlink-swarm/`](mavlink-swarm/dialect/swarm.xml) | The MAVLink dialect, `std` and `embedded`. |
| [`observability/`](observability/docker-compose.yml) | Grafana, Prometheus and Tempo. |

## Tests

```sh
make test     # brain (with std), backend, ground station
make smoke    # protocol checks against a running backend: gcs_smoke, remote_smoke
```

- `cd brain && cargo test --features std`: consensus, signatures, tracks, responders, rings,
  formation, relay chains, navigation, orders, battery, and small swarms end to end on the
  harness (brains flown together over an emulated radio, no network).
- Surveys (ignored by default, run with `--ignored --nocapture`) fly the formation over many
  fields and seeds and print how it does. `formation_survey` takes `FIELDS=80x80x20,100x30x12`,
  `SEEDS=8`, `LOSS=10` (% of frames lost), `GPS=150` (cm), `PROX=50` (ranging, dm), `JAM=60-120`
  (radio jammed, s) and `DUR=360` (s); `SURVEY=100x30x12x1 ... formation_trace` traces one flight
  drone by drone. Others: `lone_mouse_survey`, `strip_survey`, `gps_survey`, `relay_chains_survey`.
- `cargo run --example sweep_plan -- 180x175` in `brain/` prints the fleets worth flying on a field.
- In `backend/`: `gcs_smoke` and `remote_smoke` play the ground station and the remote without the
  GUI; `field_smoke` checks a new field; `roles_watch` prints who does what every 10 s. In `gcs/`:
  `mock_qgc` stands in for QGroundControl, `headless` runs the ground station without a window.

## Known limitations

- **Radio.** Statuses still reach the whole swarm, about as many relayed a second as there are
  drones: fine for tens, a lot for 150 on a real modem. The emulated radio has no bandwidth limit
  or collisions.
- **Connectivity.** In autonomous mode, the ground station reaches only the drones near the base.
  Fronts are out of radio range of each other, so a front drained by targets gets no help from
  the others. A long front on a big field splits into several radio networks part of the time.
- **Formation.** The strip along the field's long edges is looked over less (the outer drones fly
  3.5 m in, and GPS error shifts the line). A line too short for the field mows lanes, and a
  target can run into lanes already looked over. Few drones and many targets leave nobody
  searching. Everything hangs on a common clock (GPS time in the field).
- **Jamming.** Without ranging between drones, they have nothing to keep apart by once the radio
  is gone, and a drone can't always tell being jammed from having strayed.
- **Security.** Statuses, beacons and relayed frames are protected only by the shared link key,
  which a captured drone has: it can't give orders, but can confuse the formation. Nothing is
  encrypted. A flood of forged orders can delay real ones. Spoofing a drone with no neighbors in
  ranging distance goes unnoticed. The board has no read-out protection; its keys are good for
  one mission and can be revoked.
- **Board.** The body link is the simulator's own (`SWARM_HIL_*`) on the same UART as the radio;
  a real drone needs a flight controller speaking the standard offboard messages and a separate
  radio modem.
- **Simulator.** Time runs in real time. The coverage map is 10 × 10 cells whatever the field's
  size.
