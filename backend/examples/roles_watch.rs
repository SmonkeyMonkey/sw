//! Who does what in the swarm, every 10 s, from the simulator's ground truth (the viewer feed):
//! drones on mice, relaying in a chain, on the sweep, searching otherwise, and how many have a
//! radio path (8 m hops) to the base.
//!
//!   cargo run                                    # backend
//!   cargo run --example roles_watch -- 120       # this, for 120 s
use futures_util::StreamExt;
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    let secs: u64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(120);
    let (mut ws, _) = tokio_tungstenite::connect_async("ws://127.0.0.1:9002").await.unwrap();
    let start = Instant::now();
    let mut next = 10.0f32;
    while start.elapsed() < Duration::from_secs(secs) {
        let Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text))) = ws.next().await else { break };
        let t = start.elapsed().as_secs_f32();
        if t < next { continue; }
        next += 10.0;
        let s: serde_json::Value = serde_json::from_str(&text).unwrap();
        let pos = |v: &serde_json::Value| (v["pos"]["x"].as_f64().unwrap(), v["pos"]["y"].as_f64().unwrap());
        let agents: Vec<_> = s["agents"].as_array().unwrap().iter().filter(|a| a["alive"] == true && a["alt"].as_f64().unwrap_or(0.0) > 1.0).collect();
        let base = (s["base"]["x"].as_f64().unwrap(), s["base"]["y"].as_f64().unwrap());
        let d = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).hypot(a.1 - b.1);
        let p: Vec<_> = agents.iter().map(|a| pos(a)).collect();
        let mut reached = vec![false; p.len()];
        let mut q: Vec<usize> = (0..p.len()).filter(|&i| d(p[i], base) <= 8.0).collect();
        for &i in &q { reached[i] = true; }
        while let Some(i) = q.pop() { for j in 0..p.len() { if !reached[j] && d(p[i], p[j]) <= 8.0 { reached[j] = true; q.push(j); } } }
        let count = |f: &dyn Fn(&serde_json::Value) -> bool| agents.iter().filter(|a| f(a)).count();
        println!("{t:4.0}s: {} flying, on mice {}, relays {}, sweep {}, other searching {}; in contact with the base {}; mice {}",
            agents.len(),
            count(&|a| !a["target_id"].is_null()),
            count(&|a| a["role"] == "relay"),
            count(&|a| a["role"] == "sweep"),
            count(&|a| a["target_id"].is_null() && a["role"] == ""),
            reached.iter().filter(|&&r| r).count(),
            s["targets"].as_array().unwrap().len());
        let line: Vec<String> = agents.iter().map(|a| { let (x, y) = pos(a); format!("{}{}@({x:.0},{y:.0})", a["id"], if !a["target_id"].is_null() { "T" } else if a["role"] == "relay" { "R" } else if a["role"] == "sweep" { "W" } else { "s" }) }).collect();
        println!("      {}", line.join(" "));
    }
}
