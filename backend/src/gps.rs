//! Each drone's GNSS receiver: what the drone's brain is told its position and velocity are,
//! as opposed to where physics says the airframe really is.
//!
//! The error is modeled the way GNSS error behaves: it drifts slowly rather than jumping
//! around, and most of it is shared by receivers close together (same satellites, same
//! atmosphere), so drones disagree about where they *all* are far more than about where they
//! are relative to each other. Each axis is a first-order Gauss-Markov process (a random walk
//! pulled back toward zero with time constant `tau`), the standard way to model GNSS drift:
//!  - a common error, the same for every drone,
//!  - each drone's own error,
//!  - plus white noise on every fix.
//!
//! Fixes come at `rate_hz`; between fixes the brain keeps getting the last one, as from a real
//! receiver. The autopilot underneath still flies on the true state: a real one fuses GNSS
//! with its IMU in an EKF, which this simulator doesn't model, so only the swarm's own
//! decisions see the error.

use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use swarm_brain::Vec2;

#[derive(Clone, Copy)]
pub struct GpsConfig {
    /// Off: the brain gets the true position.
    pub enabled: bool,
    /// Standard deviation (m, per axis) and correlation time (s) of the error all drones share.
    pub common_sigma: f32,
    pub common_tau_s: f32,
    /// ...and of each drone's own error.
    pub own_sigma: f32,
    pub own_tau_s: f32,
    /// White noise on each fix, m.
    pub white_sigma: f32,
    /// Noise on the reported velocity, m/s.
    pub velocity_sigma: f32,
    pub rate_hz: f32,
}

impl GpsConfig {
    /// A consumer GNSS receiver in the open: about 1.5-2 m of error, mostly shared.
    pub const CONSUMER: GpsConfig = GpsConfig {
        enabled: true,
        common_sigma: 1.5,
        common_tau_s: 120.0,
        own_sigma: 0.7,
        own_tau_s: 30.0,
        white_sigma: 0.2,
        velocity_sigma: 0.1,
        rate_hz: 5.0,
    };
}

/// One axis-pair of a Gauss-Markov process.
#[derive(Clone, Copy)]
struct Drift(Vec2);

impl Drift {
    fn step(&mut self, sigma: f32, tau_s: f32, dt: f32, rng: &mut StdRng) {
        let keep = (-dt / tau_s).exp();
        // Scaled so the process's long-run standard deviation stays `sigma`.
        let kick = sigma * (1.0 - keep * keep).sqrt();
        self.0 = self.0 * keep + gaussian2(rng) * kick;
    }
}

pub struct Gps {
    cfg: GpsConfig,
    rng: StdRng,
    common: Drift,
    own: Vec<Drift>,
    /// Per drone: the latest fix (position, velocity) and when it was taken.
    fixes: Vec<Option<(Vec2, Vec2, f32)>>,
}

impl Gps {
    pub fn new(cfg: GpsConfig) -> Self {
        let mut rng = StdRng::from_rng(&mut rand::rng());
        // Start from a random point of the error's range, not from zero: the receivers have
        // been on for a while.
        let common = Drift(gaussian2(&mut rng) * cfg.common_sigma);
        Self { cfg, rng, common, own: Vec::new(), fixes: Vec::new() }
    }

    /// Advances every error process by `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        let cfg = self.cfg;
        self.common.step(cfg.common_sigma, cfg.common_tau_s, dt, &mut self.rng);
        for own in self.own.iter_mut() {
            own.step(cfg.own_sigma, cfg.own_tau_s, dt, &mut self.rng);
        }
    }

    /// What drone `i`'s receiver reports at time `t_s`, given where it truly is.
    pub fn reading(&mut self, i: usize, true_pos: Vec2, true_vel: Vec2, t_s: f32) -> (Vec2, Vec2) {
        if !self.cfg.enabled {
            return (true_pos, true_vel);
        }
        while self.own.len() <= i {
            let start = Drift(gaussian2(&mut self.rng) * self.cfg.own_sigma);
            self.own.push(start);
            self.fixes.push(None);
        }
        let due = self.fixes[i].is_none_or(|(_, _, at)| t_s - at >= 1.0 / self.cfg.rate_hz);
        if due {
            let error = self.common.0 + self.own[i].0 + gaussian2(&mut self.rng) * self.cfg.white_sigma;
            let vel = true_vel + gaussian2(&mut self.rng) * self.cfg.velocity_sigma;
            self.fixes[i] = Some((true_pos + error, vel, t_s));
        }
        let (pos, vel, _) = self.fixes[i].expect("just set");
        (pos, vel)
    }

    /// How far drone `i`'s last fix is from where it truly is.
    pub fn error(&self, i: usize, true_pos: Vec2) -> Option<f32> {
        let (pos, _, _) = (*self.fixes.get(i)?)?;
        Some((pos - true_pos).length())
    }
}

/// Two independent standard normal samples (Box-Muller).
fn gaussian2(rng: &mut StdRng) -> Vec2 {
    let u1: f32 = rng.random_range(f32::EPSILON..1.0);
    let u2: f32 = rng.random_range(0.0..1.0);
    let r = (-2.0 * u1.ln()).sqrt();
    let a = std::f32::consts::TAU * u2;
    Vec2::new(r * a.cos(), r * a.sin())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn errors_have_the_configured_size_and_are_mostly_shared() {
        let mut gps = Gps::new(GpsConfig::CONSUMER);
        let (mut abs, mut rel, mut n) = (0.0f32, 0.0f32, 0u32);
        // Two drones standing still for an hour of simulated time.
        for tick in 0..72_000 {
            let t = tick as f32 * 0.05;
            gps.advance(0.05);
            let (a, _) = gps.reading(0, Vec2::ZERO, Vec2::ZERO, t);
            let (b, _) = gps.reading(1, Vec2::new(5.0, 0.0), Vec2::ZERO, t);
            if tick % 20 == 0 {
                abs += a.length();
                rel += ((b - a) - Vec2::new(5.0, 0.0)).length();
                n += 1;
            }
        }
        let (abs, rel) = (abs / n as f32, rel / n as f32);
        // Mean of a 2D error with ~1.67 m per axis is ~2 m; relative error is only the
        // drones' own parts, ~1.2 m.
        assert!((1.2..3.0).contains(&abs), "mean absolute error {abs}");
        assert!(rel < abs, "relative error {rel} should be below absolute {abs}");
    }

    #[test]
    fn off_means_the_truth() {
        let mut gps = Gps::new(GpsConfig { enabled: false, ..GpsConfig::CONSUMER });
        assert_eq!(gps.reading(0, Vec2::new(3.0, 4.0), Vec2::ZERO, 0.0).0, Vec2::new(3.0, 4.0));
    }
}
