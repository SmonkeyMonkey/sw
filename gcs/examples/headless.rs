//! The ground station without its window: just the radio and base links and the QGroundControl
//! bridge. For running the swarm with QGC as the only ground station UI, or for tests.
//!
//!   cargo run --release --example headless

use std::time::Duration;

use swarm_gcs::{link, qgc};

fn main() {
    let (heard, _to_send) = link::spawn("127.0.0.1:14600".parse().unwrap(), "127.0.0.1:14650".parse().unwrap(), qgc::QGC_ADDR.parse().unwrap());
    println!("ground station running headless; QGroundControl bridge on udp://{}", qgc::QGC_ADDR);
    loop {
        // The link thread does the work; just keep its inbox from piling up.
        while heard.try_recv().is_ok() {}
        std::thread::sleep(Duration::from_millis(100));
    }
}
