//! Surveys over many flights: how the formation does on fields of all shapes.

use super::*;

/// Three mice wandering a `w` x `h` m field at 0.8 m/s, bouncing off its edges, from spots
/// `seed` picks.
fn wandering_mice(w: f32, h: f32, seed: u32) -> impl Fn(u32) -> std::vec::Vec<Vec2> {
    let mut rng = XorShift32::new(seed.wrapping_mul(2_654_435_761) | 1);
    let mice: std::vec::Vec<(Vec2, Vec2)> = (0..3)
        .map(|_| {
            let at = Vec2::new((rng.next_f32() - 0.5) * (w - 4.0), (rng.next_f32() - 0.5) * (h - 4.0));
            (at, rng.random_unit_vector() * 0.8)
        })
        .collect();
    // Reflected off the edges: position along a line, folded back into the field.
    let fold = |x: f32, half: f32| {
        let span = 2.0 * half;
        let m = (x + half).rem_euclid(2.0 * span);
        (if m > span { 2.0 * span - m } else { m }) - half
    };
    // MICE_X=0.5: they keep to the part of the field from that far along x (0 to 1: 0.5 is
    // the right half). MICE_EVERY=60: they turn up one by one, that many seconds apart.
    let from_x: f32 = std::env::var("MICE_X").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    let every: f32 = std::env::var("MICE_EVERY").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0);
    // MICE_EDGE=4: how near the field's edge they come (m).
    let edge: f32 = std::env::var("MICE_EDGE").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0);
    move |tick| {
        let t = tick as f32 * 0.05;
        let span_x = (w / 2.0 - edge) * (1.0 - from_x);
        let mid_x = (w / 2.0 - edge) * from_x;
        mice.iter()
            .enumerate()
            .map(|(k, (at, v))| {
                if t < k as f32 * every {
                    return Vec2::new(1.0e4, 1.0e4);
                }
                Vec2::new(mid_x + fold(at.x * (1.0 - from_x) + v.x * t, span_x), fold(at.y + v.y * t, h / 2.0 - edge))
            })
            .collect()
    }
}

/// One flight for `formation_survey`: `n` drones launched three at a time from the base
/// corner of a `w` x `h` m field, three mice wandering it, for three minutes. Prints what
/// went wrong in the formation along the way.
fn formation_flight(w: f32, h: f32, n: usize, seed: u32, trace: bool) -> [f32; 14] {
    // ADAPT=1: as many drones to a mouse as there are drones for (see `adaptive_responders`).
    let adaptive = std::env::var("ADAPT").is_ok_and(|v| v == "1");
    let cfg = BrainConfig { fleet: n as u16, adaptive_responders: adaptive, ..field(w, h) };
    // RESP=2: drones to a mouse (see `responders`).
    let cfg = match std::env::var("RESP").ok().and_then(|v| v.parse().ok()) {
        Some(responders) => BrainConfig { responders, ..cfg },
        None => cfg,
    };
    // HOPS=3: how far statuses are relayed; STATUS_MS=2000: how often each drone sends its own.
    let cfg = match std::env::var("HOPS").ok().and_then(|v| v.parse().ok()) {
        Some(status_max_hops) => BrainConfig { status_max_hops, ..cfg },
        None => cfg,
    };
    let cfg = match std::env::var("STATUS_MS").ok().and_then(|v| v.parse().ok()) {
        Some(status_period_ms) => BrainConfig { status_period_ms, ..cfg },
        None => cfg,
    };
    // BEACON_MS=400: how often each drone beacons its position.
    let cfg = match std::env::var("BEACON_MS").ok().and_then(|v| v.parse().ok()) {
        Some(beacon_period_ms) => BrainConfig { beacon_period_ms, ..cfg },
        None => cfg,
    };
    // MERGE=3: sightings within that many meters are one target (see `merge_radius`).
    let cfg = match std::env::var("MERGE").ok().and_then(|v| v.parse().ok()) {
        Some(merge_radius) => BrainConfig { merge_radius, ..cfg },
        None => cfg,
    };
    // PAD=1.2: how far apart the base's pads are (m), 2.5 as the simulator's.
    let pad: f32 = std::env::var("PAD").ok().and_then(|v| v.parse().ok()).unwrap_or(2.5);
    let starts: std::vec::Vec<Vec2> = (0..n).map(|i| cfg.home + Vec2::new((i % 3) as f32 * pad, (i / 3 % 3) as f32 * pad)).collect();
    let mice = wandering_mice(w, h, seed);
    let (cw, ch) = (w as usize, h as usize);
    let mut seen = std::vec![false; cw * ch];
    // When each cell was last looked at (s), and the share looked at within the last minute,
    // summed each second from the second minute on.
    let mut last_seen = std::vec![-1000.0f32; cw * ch];
    let (mut fresh, mut fresh_n) = (0.0f32, 0u32);
    // DUR=360: a longer flight (s).
    let ticks: u32 = std::env::var("DUR").ok().and_then(|v| v.parse::<u32>().ok()).unwrap_or(180) * 20;
    let (mut t90, mut found) = (None, [None::<u32>; 3]);
    let (mut disagree, mut isolated, mut close, mut crowded, mut samples, mut together) = (0u32, 0u32, 0u32, 0u32, 0u32, 0u32);
    let mut notes: std::vec::Vec<std::string::String> = std::vec::Vec::new();
    // Each drone's own slot a second ago, and how often it moved more than 5 m in a second.
    let mut last_slot = std::vec![None::<Vec2>; n + 1];
    let mut jumps = 0u32;
    let (mut watched, mut mouse_s) = (0u32, 0u32);
    // Drone-seconds on the sweep, investigating a sighting, on a target, and none of those;
    // and the target names drones went for.
    let mut doing = [0u32; 4];
    // Of the free pairs within a meter, those with either drone still on its way to its place.
    let mut close_transit = 0u32;
    // When each drone was last on a target (ms, 0: never).
    let mut last_busy = std::vec![0u32; n + 2];
    // Pairs of a drone's tracks of one real mouse marked apart (distinct), and drone-seconds
    // holding more than one track of one real mouse.
    let mut false_apart = std::collections::BTreeSet::<(u8, TargetId, TargetId)>::new();
    let mut doubled = 0u32;
    let mut names = std::collections::BTreeSet::<TargetId>::new();
    LAUNCH_EVERY_TICKS.with(|v| v.set(60));
    RADIO.with(|r| r.set(RadioStats::default()));
    FRAMES_SENT.with(|f| f.borrow_mut().clear());
    // LOSS=10 GPS=150: a lossy radio (% of frames lost), GPS error (cm, the shared part).
    let knob = |name: &str| std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    RADIO_LOSS_PERCENT.with(|v| v.set(knob("LOSS")));
    GPS_SIGMA_CM.with(|v| v.set(knob("GPS")));
    // PROX=50: ranging the drones within 5 m (dm), as the simulator does by default.
    PROXIMITY_DM.with(|v| v.set(knob("PROX")));
    // JAM=60-120: no frame gets through from 60 s to 120 s.
    let jam: Option<(u32, u32)> = std::env::var("JAM").ok().and_then(|v| {
        let (a, b) = v.split_once('-')?;
        Some((a.parse::<u32>().ok()? * 20, b.parse::<u32>().ok()? * 20))
    });
    // JAMFAKE=1: the same phases measured, the radio left alone, to compare with.
    let fake = std::env::var("JAMFAKE").is_ok_and(|v| v == "1");
    RADIO_JAM_TICKS.with(|v| v.set(jam.filter(|_| !fake).unwrap_or((0, 0))));
    // Before, during and after the jamming: fronts in one network (of samples), pairs of any
    // drones within a meter (pair-s), and the cells first looked at.
    let (mut jam_together, mut jam_samples, mut jam_close, mut jam_cells) = ([0u32; 3], [0u32; 3], [0u32; 3], [0u32; 3]);
    // Of those pairs, ones with either drone on a mouse or looking into a sighting.
    let mut jam_busy = [0u32; 3];
    fly_swarm_moving(cfg, seed, &starts, &mice, 8.0, ticks, None, |tick, drones| {
        let now = tick * 50;
        let phase = jam.map_or(0, |(a, b)| if tick < a { 0 } else if tick < b { 1 } else { 2 });
        let launched = |k: usize| tick >= (k as u32 / 3) * 60;
        for (k, m) in mice(tick).iter().enumerate() {
            if found[k].is_none() && drones.iter().any(|(_, p)| (*p - *m).length() <= 2.5) {
                found[k] = Some(tick / 20);
            }
        }
        if tick % 20 != 0 {
            return false;
        }
        for (b, _) in drones.iter().enumerate().filter(|(k, _)| launched(*k)).map(|(_, d)| d) {
            let k = if b.assigned.is_some() {
                2
            } else if b.mode == Mode::Investigating {
                1
            } else if b.on_sweep {
                0
            } else {
                3
            };
            doing[k] += 1;
            names.extend(b.assigned);
        }
        let real = mice(tick);
        let mouse_of = |p: Vec2| real.iter().enumerate().map(|(k, m)| ((*m - p).length(), k)).filter(|(d, _)| *d <= 6.0).min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, k)| k);
        for (b, _) in drones {
            let tracks: std::vec::Vec<(TargetId, Option<usize>, Track)> = b.tracks.iter().flatten().map(|t| (t.id, mouse_of(b.predicted(t, now)), *t)).collect();
            let mut per = [0u32; 8];
            for (_, k, _) in &tracks {
                if let Some(k) = k {
                    per[*k % 8] += 1;
                }
            }
            doubled += per.iter().any(|&c| c > 1) as u32;
            if trace && per.iter().any(|&c| c > 1) && notes.len() < 30 && tick % 100 == 0 {
                let desc: std::vec::Vec<std::string::String> = tracks
                    .iter()
                    .map(|(id, k, t)| {
                        let p = b.predicted(t, now);
                        std::format!("{id:#x}->m{k:?} at ({:.1},{:.1}) obs {} age {}ms seen {} apart {:?} wit {}", p.x, p.y, t.observer_sysid, age_at(now, t.observed_at_ms), t.seen_now, t.apart_from, t.witnesses.iter().flatten().count())
                    })
                    .collect();
                let mice_now: std::vec::Vec<std::string::String> = real.iter().map(|m| std::format!("({:.1},{:.1})", m.x, m.y)).collect();
                notes.push(std::format!("{}s drone {} at mice {:?}: {}", now / 1000, b.sysid, mice_now, desc.join(" | ")));
            }
            for (ia, ka, ta) in &tracks {
                for (ib, kb, _) in &tracks {
                    if ia < ib && ka.is_some() && ka == kb && ta.apart(*ib) {
                        false_apart.insert((b.sysid, *ia, *ib));
                    }
                }
            }
        }
        // Of the mice found, how many have a drone on them.
        for (_, m) in mice(tick).iter().enumerate().filter(|(k, _)| found[*k].is_some()) {
            mouse_s += 1;
            watched += drones.iter().any(|(b, p)| b.assigned.is_some() && (*p - *m).length() <= 4.0) as u32;
        }
        if trace && n > 40 && tick % 400 == 0 {
            // A big swarm: front by front, how many fly with it, where they are.
            for index in 0..MAX_STRETCHES {
                let ds: std::vec::Vec<&(Brain, Vec2)> = drones.iter().filter(|(d, _)| d.front.is_some_and(|f| f.index == index)).collect();
                if ds.is_empty() {
                    continue;
                }
                let (lo, hi) = ds.iter().fold((Vec2::new(f32::MAX, f32::MAX), Vec2::new(f32::MIN, f32::MIN)), |(lo, hi), (_, p)| (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y))));
                let f = ds[0].0.front.unwrap();
                std::println!("  {:3}s front {index}/{}: {} free here ({} places, {} segments, closed {}), x {:.0}..{:.0}, y {:.0}..{:.0}", tick / 20, f.fronts, ds.len(), f.size, f.segments, f.closed, lo.x, hi.x, lo.y, hi.y);
            }
            let busy = drones.iter().filter(|(d, _)| d.assigned.is_some()).count();
            let none = drones.iter().filter(|(d, _)| d.front.is_none() && d.assigned.is_none()).count();
            std::println!("  {:3}s on mice {busy}, in no front {none}", tick / 20);
        }
        if trace && n <= 40 && tick % 60 == 0 {
            let row: std::vec::Vec<std::string::String> = drones
                .iter()
                .map(|(d, p)| std::format!("{}{}{}({:.0},{:.0}){}t{}", d.sysid, if d.assigned.is_some() { "*" } else { "" }, if d.isolated(now) { "~" } else { "" }, p.x, p.y, d.front.map_or(std::string::String::new(), |f| std::format!("{}{}{}", f.count, if f.closed { "c" } else { "s" }, f.segments)), d.tracks.iter().flatten().count()))
                .collect();
            std::println!("  {:3}s mice {:?}: {}", tick / 20, mice(tick).iter().map(|m| (m.x as i32, m.y as i32)).collect::<std::vec::Vec<_>>(), row.join(" "));
        }
        for (_, (_, p)) in drones.iter().enumerate().filter(|(k, _)| launched(*k)) {
            for x in (p.x - 2.5 + w / 2.0).max(0.0) as usize..((p.x + 2.5 + w / 2.0) as usize).min(cw) {
                for y in (p.y - 2.5 + h / 2.0).max(0.0) as usize..((p.y + 2.5 + h / 2.0) as usize).min(ch) {
                    let c = Vec2::new(x as f32 + 0.5 - w / 2.0, y as f32 + 0.5 - h / 2.0);
                    if (c - *p).length() <= cfg.scan_radius {
                        jam_cells[phase] += !seen[x * ch + y] as u32;
                        seen[x * ch + y] = true;
                        last_seen[x * ch + y] = tick as f32 / 20.0;
                    }
                }
            }
        }
        let share = seen.iter().filter(|&&s| s).count() as f32 / seen.len() as f32;
        if tick >= 2_400 {
            fresh += last_seen.iter().filter(|&&at| tick as f32 / 20.0 - at <= 60.0).count() as f32 / last_seen.len() as f32;
            fresh_n += 1;
        }
        if t90.is_none() && share >= 0.9 {
            t90 = Some(tick / 20);
        }
        for (i, (a, pa)) in drones.iter().enumerate().filter(|(k, _)| launched(*k)) {
            for (_, (b, pb)) in drones.iter().enumerate().skip(i + 1).filter(|(k, _)| launched(*k)) {
                if (*pa - *pb).length() < 1.0 {
                    jam_close[phase] += 1;
                    let busy = |d: &Brain| d.assigned.is_some() || d.mode == Mode::Investigating;
                    jam_busy[phase] += (busy(a) || busy(b)) as u32;
                    if trace && phase == 1 && notes.len() < 60 {
                        notes.push(std::format!(
                            "JAMCLOSE {}s {} {:?} {:?} slot {:?} at ({:.1},{:.1}) / {} {:?} {:?} slot {:?} at ({:.1},{:.1})",
                            now / 1000, a.sysid, a.mode, a.assigned, a.slot_of(a.sysid, now), pa.x, pa.y, b.sysid, b.mode, b.assigned, b.slot_of(b.sysid, now), pb.x, pb.y
                        ));
                    }
                }
            }
        }
        // From 40 s on: the fronts have had time to get to their stretches.
        if tick < 800 {
            return false;
        }
        for (d, _) in drones.iter().filter(|(d, _)| d.assigned.is_some()) {
            last_busy[d.sysid as usize] = now;
        }
        let free: std::vec::Vec<&(Brain, Vec2)> = drones.iter().filter(|(b, _)| b.assigned.is_none() && b.front.is_some()).collect();
        // Each front's free drones one radio network.
        for index in 0..MAX_STRETCHES {
            let front: std::vec::Vec<Vec2> = free.iter().filter(|(b, _)| b.front.unwrap().index == index).map(|(_, p)| *p).collect();
            if front.len() < 2 {
                continue;
            }
            samples += 1;
            jam_samples[phase] += 1;
            let mut reached = std::vec![false; front.len()];
            let mut frontier = std::vec![0];
            reached[0] = true;
            while let Some(i) = frontier.pop() {
                for j in 0..front.len() {
                    if !reached[j] && (front[i] - front[j]).length() <= 8.0 {
                        reached[j] = true;
                        frontier.push(j);
                    }
                }
            }
            together += reached.iter().all(|&r| r) as u32;
            jam_together[phase] += reached.iter().all(|&r| r) as u32;
        }
        for (d, _) in drones {
            let slot = d.slot_of(d.sysid, now).filter(|_| d.assigned.is_none());
            if let (Some(was), Some(is)) = (last_slot[d.sysid as usize], slot) {
                if (is - was).length() > 5.0 {
                    jumps += 1;
                    if notes.len() < 12 {
                        notes.push(std::format!("{}s: {}'s place jumped {:.0} m, ({:.0},{:.0}) to ({:.0},{:.0}), places {:?}", now / 1000, d.sysid, (is - was).length(), was.x, was.y, is.x, is.y, d.front_places()));
                    }
                }
            }
            last_slot[d.sysid as usize] = slot;
        }
        for (a, pa) in &free {
            let fa = a.front.unwrap();
            let alone = !free.iter().any(|(b, pb)| b.sysid != a.sysid && b.front.unwrap().index == fa.index && (*pb - *pa).length() <= 8.0);
            isolated += alone as u32;
            if alone && notes.len() < 12 {
                notes.push(std::format!("{}s: {} alone at ({:.0},{:.0}), front {} places {:?}", now / 1000, a.sysid, pa.x, pa.y, fa.index, a.front_places()));
            }
            for (b, pb) in &free {
                if b.sysid <= a.sysid {
                    continue;
                }
                close += ((*pb - *pa).length() < 1.0) as u32;
                if (*pb - *pa).length() < 1.0 {
                    let far = |d: &Brain, p: Vec2| d.slot_of(d.sysid, now).is_none_or(|s| (s - p).length() > 10.0);
                    close_transit += (far(a, *pa) || far(b, *pb)) as u32;
                    if trace && !(far(a, *pa) || far(b, *pb)) && notes.len() < 2000 {
                        let (sa, sb) = (a.slot_of(a.sysid, now).unwrap(), b.slot_of(b.sysid, now).unwrap());
                        notes.push(std::format!(
                            "ONSTATION {} {} {} slots {:.1} apart; off own {:.1} {:.1}; back from a mouse {} {}",
                            now / 1000, a.sysid, b.sysid, (sa - sb).length(), (sa - *pa).length(), (sb - *pb).length(),
                            now.saturating_sub(last_busy[a.sysid as usize]) < 20_000 && last_busy[a.sysid as usize] > 0,
                            now.saturating_sub(last_busy[b.sysid as usize]) < 20_000 && last_busy[b.sysid as usize] > 0
                        ));
                    }
                }
                if (*pb - *pa).length() < 1.0 && notes.len() < 40 {
                    let sa = a.slot_of(a.sysid, now).unwrap_or(Vec2::ZERO);
                    let sb = b.slot_of(b.sysid, now).unwrap_or(Vec2::ZERO);
                    notes.push(std::format!("{}s: CLOSE {} ({:.1},{:.1}) slot ({:.1},{:.1}) and {} ({:.1},{:.1}) slot ({:.1},{:.1}); fronts {} {}", now / 1000, a.sysid, pa.x, pa.y, sa.x, sa.y, b.sysid, pb.x, pb.y, sb.x, sb.y, fa.index, b.front.unwrap().index));
                }
                if b.front.unwrap().index != fa.index {
                    continue;
                }
                // Drones near each other in a front putting either of them somewhere else.
                if (*pb - *pa).length() > 16.0 {
                    continue;
                }
                let apart = |m: u8| Some((a.slot_of(m, now)? - b.slot_of(m, now)?).length());
                let off = [a.sysid, b.sysid].map(|m| apart(m).unwrap_or(99.0));
                if off[0].max(off[1]) > 2.0 {
                    disagree += 1;
                    if notes.len() < 12 {
                        let all: std::vec::Vec<std::string::String> = drones
                            .iter()
                            .filter(|(d, _)| d.front.is_some_and(|f| f.index == fa.index))
                            .map(|(d, p)| std::format!("{}{}@({:.0},{:.0}){:?}", d.sysid, if d.assigned.is_some() { "*" } else { "" }, p.x, p.y, d.front_places()))
                            .collect();
                        notes.push(std::format!("{}s: {} and {} {:.0}/{:.0} m off: {}", now / 1000, a.sysid, b.sysid, off[0], off[1], all.join(" ")));
                    }
                }
            }
        }
        let mut per = std::collections::BTreeMap::<TargetId, u32>::new();
        for (b, _) in drones {
            if let Some(id) = b.assigned {
                *per.entry(id).or_default() += 1;
            }
        }
        crowded += per.values().filter(|&&c| c > cfg.responders as u32).count() as u32;
        false
    });
    LAUNCH_EVERY_TICKS.with(|v| v.set(0));
    if trace {
        std::println!("CLOSE {w}x{h} {n} seed {seed}: free pairs within a meter {close} pair-s, {close_transit} of them on the way to their places");
    }
    // RADIOLOG=1: what the flight put on the air.
    if trace || std::env::var("RADIOLOG").is_ok_and(|v| v == "1") {
        let r = RADIO.with(|r| r.get());
        let drone_s = r.drone_ticks as f32 / 20.0;
        let bytes_per_frame = r.sent_bytes as f32 / r.sent_frames.max(1) as f32;
        let heard_per_s = r.heard_frames as f32 / drone_s.max(1.0);
        std::println!(
            "RADIO {w}x{h} {n} seed {seed}: each drone sends {:.1} frames/s, {:.2} kbit/s ({bytes_per_frame:.0} B a frame); hears {heard_per_s:.0} frames/s on the air around it ({:.0} kbit/s), {} in the busiest 50 ms",
            r.sent_frames as f32 / drone_s.max(1.0),
            r.sent_bytes as f32 * 8.0 / 1000.0 / drone_s.max(1.0),
            heard_per_s * bytes_per_frame * 8.0 / 1000.0,
            r.busiest_tick
        );
        let mut by_kind: std::vec::Vec<(&'static str, u32)> = FRAMES_SENT.with(|f| f.borrow().iter().map(|(k, v)| (*k, *v)).collect());
        by_kind.sort_by_key(|(_, v)| core::cmp::Reverse(*v));
        std::println!("    by kind, per drone per s: {}", by_kind.iter().map(|(k, v)| std::format!("{k} {:.2}", *v as f32 / drone_s.max(1.0))).collect::<std::vec::Vec<_>>().join(", "));
    }
    FRAMES_SENT.with(|f| f.borrow_mut().clear());
    if let Some((a, b)) = jam {
        let cells = (cw * ch) as f32 / 100.0;
        let span = [a.min(ticks), b.min(ticks).saturating_sub(a), ticks.saturating_sub(b)].map(|t| (t as f32 / 20.0).max(1.0));
        let part = |k: usize| {
            std::format!(
                "together {:.0}%, any two within a meter {} pair-s ({} at mice), {:.2}% of the field newly seen a second",
                100.0 * jam_together[k] as f32 / jam_samples[k].max(1) as f32,
                jam_close[k],
                jam_busy[k],
                jam_cells[k] as f32 / cells / span[k]
            )
        };
        std::println!("JAM {w}x{h} {n} seed {seed}: before: {} | jammed: {} | after: {}", part(0), part(1), part(2));
    }
    RADIO_JAM_TICKS.with(|v| v.set((0, 0)));
    RADIO_LOSS_PERCENT.with(|v| v.set(0));
    GPS_SIGMA_CM.with(|v| v.set(0));
    PROXIMITY_DM.with(|v| v.set(0));
    let share = seen.iter().filter(|&&s| s).count() as f32 / seen.len() as f32;
    std::println!(
        "{w}x{h} {n} drones seed {seed}: free together {:.0}% | covered {:.0}% (90% at {t90:?} s) | mice found at {found:?} s | disagree {disagree} | alone {isolated} | <1 m {close} | crowded mice {crowded} | jumps {jumps}",
        100.0 * together as f32 / samples.max(1) as f32,
        100.0 * share
    );
    if trace {
        for note in notes {
            std::println!("    {note}");
        }
    }
    [100.0 * together as f32 / samples.max(1) as f32, 100.0 * share, t90.map_or(180.0, |t| t as f32), isolated as f32, disagree as f32, 100.0 * watched as f32 / mouse_s.max(1) as f32, found.iter().map(|f| f.map_or(180.0, |t| t as f32)).fold(0.0, f32::max), close as f32, doing[0] as f32, doing[1] as f32, doing[2] as f32, doing[3] as f32 + 1000.0 * names.len() as f32 + 1_000_000.0 * false_apart.len() as f32, 100.0 * fresh / fresh_n.max(1) as f32, doubled as f32]
}

/// Not a pass/fail test: anomalies in the formation on fields of different shapes.
/// `cargo test --release --features std formation_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn formation_survey() {
    let all = [(100.0, 30.0, 12), (30.0, 30.0, 6), (40.0, 40.0, 12), (60.0, 30.0, 8), (150.0, 40.0, 16), (200.0, 20.0, 12), (80.0, 80.0, 20), (50.0, 50.0, 4)];
    // FIELDS=200x20x12,220x30x22 for just those.
    let fields: std::vec::Vec<(f32, f32, usize)> = match std::env::var("FIELDS") {
        Ok(f) => f
            .split(',')
            .map(|f| {
                let v: std::vec::Vec<f32> = f.split('x').map(|x| x.parse().unwrap()).collect();
                (v[0], v[1], v[2] as usize)
            })
            .collect(),
        Err(_) => all.to_vec(),
    };
    for &(w, h, n) in &fields {
        let seeds: u32 = std::env::var("SEEDS").ok().and_then(|s| s.parse().ok()).unwrap_or(8);
        let runs: std::vec::Vec<[f32; 14]> = (1..=seeds).map(|seed| formation_flight(w, h, n, seed, false)).collect();
        let mean = |k: usize| runs.iter().map(|r| r[k]).sum::<f32>() / runs.len() as f32;
        let worst = runs.iter().map(|r| r[0]).fold(f32::MAX, f32::min);
        let other = runs.iter().map(|r| r[11] % 1000.0).sum::<f32>() / runs.len() as f32;
        let names = runs.iter().map(|r| libm::floorf(r[11] % 1_000_000.0 / 1000.0)).sum::<f32>() / runs.len() as f32;
        let false_apart = runs.iter().map(|r| libm::floorf(r[11] / 1_000_000.0)).sum::<f32>() / runs.len() as f32;
        let doubled = runs.iter().map(|r| r[13]).sum::<f32>() / runs.len() as f32;
        std::println!(
            "DOING {w}x{h} {n}: sweep {:.0} | investigating {:.0} | on targets {:.0} | other {other:.0} drone-s | target names {names:.1} | one mouse's tracks marked apart {false_apart:.1} | drone-s with a mouse twice {doubled:.0}",
            mean(8),
            mean(9),
            mean(10)
        );
        std::println!(
            "MEAN {w}x{h} {n}: together {:.0}% (worst {worst:.0}%) | covered {:.0}% | 90% at {:.0} s | alone {:.0} | disagree {:.0} | mice watched {:.0}% | all found at {:.0} s | <1 m {:.1} | seen in the last minute {:.0}%",
            mean(0),
            mean(1),
            mean(2),
            mean(3),
            mean(4),
            mean(5),
            mean(6),
            mean(7),
            mean(12)
        );
    }
}

/// `formation_survey`'s flight on one field, drone by drone every 3 s.
/// `SURVEY=80x80x20x2 cargo test --release --features std formation_trace -- --ignored --nocapture`
#[test]
#[ignore]
fn formation_trace() {
    let v: std::vec::Vec<u32> = std::env::var("SURVEY").unwrap().split('x').map(|x| x.parse().unwrap()).collect();
    formation_flight(v[0] as f32, v[1] as f32, v[2] as usize, v[3], true);
}

/// Not a pass/fail test: a mouse nobody is on (no drone on a target within 4 m of it), seen
/// by a free drone (within 2.5 m) that doesn't take it on — and what kept it from it (the
/// checks of `is_responder`, in order). Drone-seconds over a flight on each field.
/// `FIELDS=100x30x12 SEEDS=4 cargo test --release --features std lone_mouse_survey -- --ignored --nocapture`
#[test]
#[ignore]
fn lone_mouse_survey() {
    let fields: std::vec::Vec<(f32, f32, usize)> = std::env::var("FIELDS")
        .unwrap_or_else(|_| "100x30x12,80x80x20,40x40x12,200x20x12".into())
        .split(',')
        .map(|f| {
            let v: std::vec::Vec<f32> = f.split('x').map(|x| x.parse().unwrap()).collect();
            (v[0], v[1], v[2] as usize)
        })
        .collect();
    let seeds: u32 = std::env::var("SEEDS").ok().and_then(|s| s.parse().ok()).unwrap_or(4);
    for (w, h, n) in fields {
        let mut tally = std::collections::BTreeMap::<&'static str, u32>::new();
        let (mut lone, mut lone_seen) = (0u32, 0u32);
        for seed in 1..=seeds {
            let cfg = BrainConfig { fleet: n as u16, ..field(w, h) };
            let starts: std::vec::Vec<Vec2> = (0..n).map(|i| cfg.home + Vec2::new((i % 3) as f32 * 2.5, (i / 3 % 3) as f32 * 2.5)).collect();
            let mice = wandering_mice(w, h, seed);
            LAUNCH_EVERY_TICKS.with(|v| v.set(60));
            fly_swarm_moving(cfg, seed, &starts, &mice, 8.0, 180 * 20, None, |tick, drones| {
                if tick % 20 != 0 {
                    return false;
                }
                let now = tick * 50;
                for m in mice(tick) {
                    if drones.iter().any(|(b, p)| b.assigned.is_some() && (*p - m).length() <= 4.0) {
                        continue;
                    }
                    lone += 1;
                    let seers: std::vec::Vec<&(Brain, Vec2)> = drones.iter().filter(|(b, p)| b.assigned.is_none() && (*p - m).length() <= 2.5).collect();
                    lone_seen += !seers.is_empty() as u32;
                    for (b, _) in seers {
                        let Some(t) = b.tracks.iter().flatten().find(|t| (b.predicted(t, now) - m).length() <= 3.0) else {
                            *tally.entry("no track of it").or_default() += 1;
                            continue;
                        };
                        let why = if b.order.is_some() {
                            "under an order (going home)"
                        } else if b.left.is_some_and(|(left, at)| left == t.id && now.saturating_sub(at) < LEFT_MEMORY_MS) {
                            "left it lately"
                        } else if t.responders >= b.join_share {
                            "reported full"
                        } else if t.full_at_ms != 0 && now.saturating_sub(t.full_at_ms) < FULL_MEMORY_MS {
                            "full a moment ago"
                        } else if b.slot_of(b.sysid, now).is_some() && {
                            let (index, fronts) = b.home_stretch(b.sysid);
                            !b.cfg.front_path(fronts, index, 1).holds(m)
                        } {
                            "another front's stretch"
                        } else if b.slot_of(b.sysid, now).is_some() && b.in_reserve(now) {
                            "its front's reserve"
                        } else if b.formation_ahead(t.id, m, now).is_some_and(|ahead| ahead >= b.join_share) {
                            "others' places nearer"
                        } else {
                            "none of those (taking it on)"
                        };
                        *tally.entry(why).or_default() += 1;
                    }
                }
                false
            });
            LAUNCH_EVERY_TICKS.with(|v| v.set(0));
        }
        std::println!("LONE {w}x{h} {n}: mouse-seconds with nobody on it {lone}, seen by a free drone {lone_seen}; why the drones seeing it didn't take it on (drone-seconds): {tally:?}");
    }
}
