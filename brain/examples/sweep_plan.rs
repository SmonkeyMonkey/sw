//! How many drones a field needs to sweep it, and how they'd do it (see
//! `BrainConfig::sweep_layout`). `cargo run --example sweep_plan -- 180x175 [pass seconds]`

use swarm_brain::{BrainConfig, SwarmMode, Vec2};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let Some((w, h)) = args.get(1).and_then(|f| f.split_once('x')).and_then(|(w, h)| Some((w.parse::<f32>().ok()?, h.parse::<f32>().ok()?))) else {
        eprintln!("usage: sweep_plan <width>x<height> [pass seconds, default 60]");
        std::process::exit(2);
    };
    let pass_s: f32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(60.0);
    let half = Vec2::new(w / 2.0, h / 2.0);
    let cfg = BrainConfig { field_half: half, home: Vec2::new(2.0 - half.x, 2.0 - half.y), ..BrainConfig::SWARM }.with_mode(SwarmMode::Autonomous);

    println!("{w} x {h} m: fleets worth sweeping it with (each looks it over sooner than any smaller one)\n");
    println!("drones  fronts  abreast  segments  lanes  one pass  efficiency");
    for l in cfg.sweep_options(128) {
        println!("{:6}  {:6}  {:7}  {:8}  {:5}  {:6.0} s  {:9.0}%", l.drones, l.rows, l.per_row, l.segments, l.lanes, l.pass_s, l.efficiency * 100.0);
    }
    let l = cfg.sweep_for(pass_s);
    let plan = cfg.plan(1);
    println!(
        "\nFor a pass in {pass_s:.0} s: {} drones ({} front(s) of {} abreast), {:.0} s a pass, {:.0}% efficient.",
        l.drones,
        l.rows,
        l.per_row,
        l.pass_s,
        l.efficiency * 100.0
    );
    println!("With a mouse to ring ({} drones), the mission takes {}.", plan.rings, l.drones + plan.rings + plan.relays);
}
