//! The drones' bodies: one Rapier rigid body per airframe, each pushed around only by its four
//! rotors, gravity and air drag, plus each drone's own low-level autopilot flying it.
//!
//! The split mirrors a real drone with a companion computer: the swarm brain (on a simulated
//! drone, or on the STM32 board) is the companion computer and only ever asks for a horizontal
//! velocity, at 20 Hz — the way a mission computer drives PX4/ArduPilot in offboard mode. The
//! autopilot here is the flight controller underneath: a PX4-style cascade (velocity → tilt and
//! thrust → body rates → motor thrusts) running at the physics rate, with altitude hold. So a
//! drone can't stop or turn instantly any more, wind blows it off course until the velocity
//! loop's integrator catches up, a killed drone's motors stop and it falls, and drones that get
//! too close physically collide.
//!
//! Frames: Rapier world is Y-up. The field's 2D (x, y) maps to world (x, z); altitude is y.
//! Body frame: +Y up through the rotor disk, rotors at (±arm, 0, ±arm).

use std::collections::{HashMap, HashSet};

use glam::{Mat3, Quat, Vec3};
use rapier3d::prelude::*;
use swarm_brain::Vec2;

pub const PHYSICS_DT: f32 = 1.0 / 400.0;
/// Every drone holds this altitude; the swarm searches a flat field in one layer, which is
/// also what makes separation (and collisions) matter.
pub const CRUISE_ALTITUDE: f32 = 5.0;
/// Below this, the autopilot only climbs or descends (takeoff and landing).
const SAFE_ALTITUDE: f32 = 1.5;
const GRAVITY: f32 = 9.81;

/// A DJI Mavic-class quad: mass, size and thrust in the right ballpark, not any exact model.
#[derive(Clone, Copy)]
pub struct Airframe {
    pub mass: f32,
    /// Center to each rotor, along both body axes (an X frame).
    pub arm: f32,
    /// Half-size of the collision box, props included.
    pub half_extent: f32,
    pub half_height: f32,
    pub max_rotor_thrust: f32,
    /// First-order lag from commanded to actual rotor thrust (ESC + motor + prop spin-up).
    pub rotor_time_constant: f32,
    /// Rotor reaction torque per newton of thrust (prop drag), which is what yaws a quad.
    pub yaw_torque_per_thrust: f32,
    /// Air drag, relative to the air (so wind pushes): F = -(linear + quadratic*|v|) * v.
    pub drag_linear: f32,
    pub drag_quadratic: f32,
}

pub const MAVIC_CLASS: Airframe = Airframe {
    mass: 0.9,
    arm: 0.18,
    half_extent: 0.2,
    half_height: 0.04,
    // Thrust-to-weight ~2.2: 4 * 4.9 N vs 0.9 kg * 9.81.
    max_rotor_thrust: 4.9,
    rotor_time_constant: 0.025,
    yaw_torque_per_thrust: 0.016,
    drag_linear: 0.1,
    drag_quadratic: 0.03,
};

impl Airframe {
    /// Principal moments of inertia of the collision box (Rapier derives the same ones from
    /// the collider's mass), which the rate controller scales its torques by.
    fn inertia(&self) -> Vec3 {
        let (w, h) = (2.0 * self.half_extent, 2.0 * self.half_height);
        let side = self.mass / 12.0 * (w * w + h * h);
        Vec3::new(side, self.mass / 12.0 * (2.0 * w * w), side)
    }
}

/// Rotor positions (in units of `arm`) and spin direction (+1/-1, which way its reaction
/// torque yaws the frame). Diagonal rotors spin the same way so yaw torques cancel in hover.
const ROTORS: [(f32, f32, f32); 4] = [(-1.0, 1.0, 1.0), (1.0, 1.0, -1.0), (-1.0, -1.0, -1.0), (1.0, -1.0, 1.0)];

/// Autopilot tuning. Each loop is several times slower than the one inside it, so each sees
/// the inner one as (nearly) instant — the usual way a cascade is kept stable.
mod gains {
    /// Body rate loop, 1/s. ~20 rad/s bandwidth, under the rotors' 1/0.025 = 40 rad/s lag.
    pub const RATE: f32 = 25.0;
    /// Attitude loop, 1/s.
    pub const ATTITUDE: f32 = 8.0;
    pub const MAX_RATE: f32 = 4.0;
    /// Velocity loop: P (1/s) and I (1/s²), with the integrator capped (m/s²).
    pub const VELOCITY_P: f32 = 2.5;
    pub const VELOCITY_I: f32 = 1.0;
    pub const MAX_INTEGRAL: f32 = 4.0;
    /// Altitude hold, 1/s, and its climb-rate limit (m/s).
    pub const ALTITUDE: f32 = 1.5;
    pub const MAX_CLIMB: f32 = 1.5;
    /// 35°: limits horizontal acceleration to g*tan(35°) ≈ 6.9 m/s².
    pub const MAX_TILT: f32 = 35.0 * core::f32::consts::PI / 180.0;
}

/// Roughly what an IMU + GPS-aided state estimator would hand the autopilot. True state for
/// now; sensor noise would go here.
struct State {
    velocity: Vec3,
    altitude: f32,
    rotation: Quat,
    /// Body frame, as a gyro measures it.
    body_rates: Vec3,
}

struct Autopilot {
    /// Altitude hold's target: cruise altitude, or below the ground to land.
    altitude_sp: f32,
    velocity_integral: Vec3,
    /// Actual (lagged) thrust of each rotor.
    rotor_thrust: [f32; 4],
}

impl Autopilot {
    fn hovering(airframe: &Airframe) -> Self {
        Self {
            altitude_sp: CRUISE_ALTITUDE,
            velocity_integral: Vec3::ZERO,
            rotor_thrust: [airframe.mass * GRAVITY / 4.0; 4],
        }
    }

    /// One control step: from a horizontal velocity setpoint to commanded rotor thrusts.
    fn control(&mut self, airframe: &Airframe, state: &State, setpoint: Vec2, dt: f32) -> [f32; 4] {
        // Velocity → acceleration. Vertical velocity comes from altitude hold. Close to the
        // ground (taking off or landing) it only moves vertically, like a real autopilot's
        // takeoff and landing phases, so it doesn't drag its skids across the field.
        let climb = (gains::ALTITUDE * (self.altitude_sp - state.altitude)).clamp(-gains::MAX_CLIMB, gains::MAX_CLIMB);
        let horizontal = if state.altitude < SAFE_ALTITUDE || self.altitude_sp < SAFE_ALTITUDE { Vec2::ZERO } else { setpoint };
        let velocity_sp = Vec3::new(horizontal.x, climb, horizontal.y);
        let error = velocity_sp - state.velocity;
        self.velocity_integral = (self.velocity_integral + error * gains::VELOCITY_I * dt).clamp_length_max(gains::MAX_INTEGRAL);
        let mut accel = error * gains::VELOCITY_P + self.velocity_integral;
        let max_horizontal = GRAVITY * gains::MAX_TILT.tan();
        let horizontal = Vec3::new(accel.x, 0.0, accel.z).clamp_length_max(max_horizontal);
        accel.x = horizontal.x;
        accel.z = horizontal.z;

        // Acceleration → desired thrust vector: tilt the rotor disk toward it, heading fixed.
        let thrust_vector = airframe.mass * (accel + Vec3::Y * GRAVITY);
        let up = thrust_vector.normalize_or(Vec3::Y);
        let side = Vec3::X.cross(up).normalize_or(Vec3::Z);
        let desired = Quat::from_mat3(&Mat3::from_cols(up.cross(side), up, side));
        // Only the part of the thrust vector along the *current* disk axis does useful work.
        let collective = thrust_vector.dot(state.rotation * Vec3::Y).max(0.0);

        // Attitude → body rates, from the quaternion error (no Euler-angle singularities).
        let mut q_err = state.rotation.inverse() * desired;
        if q_err.w < 0.0 {
            q_err = -q_err;
        }
        let rate_sp = (2.0 * gains::ATTITUDE * Vec3::new(q_err.x, q_err.y, q_err.z)).clamp_length_max(gains::MAX_RATE);

        // Body rates → torques.
        let torque = airframe.inertia() * (gains::RATE * (rate_sp - state.body_rates));
        mix(airframe, collective, torque)
    }

    /// Rotors spin up and down with a lag; returns the thrust each one actually produces.
    fn spin(&mut self, airframe: &Airframe, commanded: [f32; 4], dt: f32) -> [f32; 4] {
        let k = (dt / airframe.rotor_time_constant).min(1.0);
        for (actual, cmd) in self.rotor_thrust.iter_mut().zip(commanded) {
            *actual += (cmd - *actual) * k;
        }
        self.rotor_thrust
    }
}

/// Collective thrust plus body torques → per-rotor thrust, the inverse of `rotor_wrench`. When a
/// rotor would saturate, the collective is shifted to make room, so attitude control keeps its
/// authority (a drone that can't hold attitude can't do anything else either).
fn mix(airframe: &Airframe, collective: f32, torque: Vec3) -> [f32; 4] {
    let a = airframe.arm;
    let km = airframe.yaw_torque_per_thrust;
    let mut diff = [0.0f32; 4];
    for (i, &(x, z, spin)) in ROTORS.iter().enumerate() {
        // Torque from rotor i's thrust f at (x*a, 0, z*a): (-z*a*f, spin*km*f, x*a*f).
        diff[i] = (-z * torque.x / a + spin * torque.y / km + x * torque.z / a) / 4.0;
    }
    let max = airframe.max_rotor_thrust;
    let mut base = collective / 4.0;
    let (lo, hi) = diff.iter().fold((f32::MAX, f32::MIN), |(lo, hi), &d| (lo.min(d), hi.max(d)));
    if base + hi > max {
        base = max - hi;
    }
    if base + lo < 0.0 {
        base = -lo;
    }
    diff.map(|d| (base + d).clamp(0.0, max))
}

struct Drone {
    body: RigidBodyHandle,
    autopilot: Autopilot,
    armed: bool,
    /// Removed from the simulation (`remove_drone`).
    gone: bool,
}

pub struct Physics {
    world: PhysicsWorld,
    airframe: Airframe,
    drones: Vec<Drone>,
    owner: HashMap<ColliderHandle, usize>,
    /// Drone pairs in contact after the last step, so a collision is counted once, not every
    /// step the two stay touching.
    touching: HashSet<(usize, usize)>,
    pub wind: Vec2,
}

impl Physics {
    /// One drone per start position, hovering at cruise altitude.
    pub fn new(airframe: Airframe, starts: &[Vec2]) -> Self {
        let mut world = PhysicsWorld::new();
        world.integration_parameters.dt = PHYSICS_DT;
        world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
        // Ground, top surface at y = 0.
        world.insert_collider(ColliderBuilder::cuboid(500.0, 0.5, 500.0).translation(Vector::new(0.0, -0.5, 0.0)), None);

        let mut physics = Self { world, airframe, drones: Vec::new(), owner: HashMap::new(), touching: HashSet::new(), wind: Vec2::ZERO };
        for &start in starts {
            physics.add_drone(start, false);
        }
        physics
    }

    /// A new airframe over `at`, either hovering at cruise altitude or sitting on the ground
    /// about to take off; returns its index.
    pub fn add_drone(&mut self, at: Vec2, on_ground: bool) -> usize {
        let airframe = self.airframe;
        let altitude = if on_ground { airframe.half_height + 0.01 } else { CRUISE_ALTITUDE };
        let body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(at.x, altitude, at.y))
            // Drones are never at rest while flying; don't let Rapier put one to sleep.
            .can_sleep(false);
        let collider = ColliderBuilder::cuboid(airframe.half_extent, airframe.half_height, airframe.half_extent)
            .mass(airframe.mass)
            .friction(0.8);
        let (body, collider) = self.world.insert(body, collider);
        let i = self.drones.len();
        self.owner.insert(collider, i);
        self.drones.push(Drone { body, autopilot: Autopilot::hovering(&airframe), armed: true, gone: false });
        i
    }

    /// Another airframe in drone `i`'s place, on the ground over `at` about to take off: one
    /// charged on the base's pads, flying as the drone that went to charge (see world.rs).
    pub fn relaunch(&mut self, i: usize, at: Vec2) {
        let old = self.drones[i].body;
        self.world.remove_body(old);
        self.owner.retain(|_, &mut owner| owner != i);
        self.touching.retain(|&(a, b)| a != i && b != i);
        let added = self.add_drone(at, true);
        let fresh = self.drones.pop().expect("just added");
        self.owner.retain(|_, owner| *owner != added);
        let collider = self.world.bodies[fresh.body].colliders()[0];
        self.owner.insert(collider, i);
        self.drones[i] = fresh;
    }

    /// Takes a drone out of the simulation altogether (it left the mission); its index stays
    /// reserved so other drones' indices don't shift.
    pub fn remove_drone(&mut self, i: usize) {
        self.drones[i].gone = true;
        self.drones[i].armed = false;
        self.world.bodies[self.drones[i].body].set_enabled(false);
    }

    /// Flies every drone for `duration` seconds toward its horizontal velocity setpoint.
    /// Returns the drone pairs that newly collided.
    pub fn step(&mut self, duration: f32, setpoints: &[Vec2]) -> Vec<(usize, usize)> {
        let steps = (duration / PHYSICS_DT).round().max(1.0) as usize;
        let mut collisions = Vec::new();
        for _ in 0..steps {
            self.substep(setpoints);
            collisions.extend(self.new_contacts());
        }
        collisions
    }

    /// Stops a drone's rotors for good: it falls wherever it is.
    pub fn disarm(&mut self, i: usize) {
        self.drones[i].armed = false;
    }

    /// Starts a controlled descent; `landed` turns true once it's down and its rotors stop.
    pub fn land(&mut self, i: usize) {
        // Aim below the ground, so the descent doesn't flare out just above it.
        self.drones[i].autopilot.altitude_sp = -1.0;
    }

    /// On the ground after `land`, rotors stopped.
    pub fn landed(&mut self, i: usize) -> bool {
        let drone = &self.drones[i];
        if drone.autopilot.altitude_sp >= 0.0 {
            return false;
        }
        let rb = &self.world.bodies[drone.body];
        let down = rb.translation().y < self.airframe.half_height + 0.05 && rb.linvel().y.abs() < 0.2;
        if down {
            self.drones[i].armed = false;
        }
        down
    }

    pub fn position(&self, i: usize) -> Vec2 {
        let t = self.world.bodies[self.drones[i].body].translation();
        Vec2::new(t.x, t.z)
    }

    /// Horizontal ground velocity, field coordinates.
    pub fn velocity(&self, i: usize) -> Vec2 {
        let v = self.world.bodies[self.drones[i].body].linvel();
        Vec2::new(v.x, v.z)
    }

    pub fn altitude(&self, i: usize) -> f32 {
        self.world.bodies[self.drones[i].body].translation().y
    }

    /// How fast two drones are moving toward or past each other, m/s.
    pub fn relative_speed(&self, a: usize, b: usize) -> f32 {
        let va = self.world.bodies[self.drones[a].body].linvel();
        let vb = self.world.bodies[self.drones[b].body].linvel();
        (va - vb).length()
    }

    /// Angle between the rotor disk axis and vertical, degrees.
    pub fn tilt_degrees(&self, i: usize) -> f32 {
        let up = *self.world.bodies[self.drones[i].body].rotation() * Vec3::Y;
        up.y.clamp(-1.0, 1.0).acos().to_degrees()
    }

    fn substep(&mut self, setpoints: &[Vec2]) {
        let airframe = self.airframe;
        let wind = Vec3::new(self.wind.x, 0.0, self.wind.y);
        for (i, drone) in self.drones.iter_mut().enumerate().filter(|(_, d)| !d.gone) {
            let rb = &mut self.world.bodies[drone.body];
            // Rapier keeps applying user forces until they're cleared; these are per-step.
            rb.reset_forces(true);
            rb.reset_torques(true);
            let rotation = *rb.rotation();
            let state = State {
                velocity: rb.linvel(),
                altitude: rb.translation().y,
                rotation,
                body_rates: rotation.inverse() * rb.angvel(),
            };
            let commanded = if drone.armed {
                drone.autopilot.control(&airframe, &state, setpoints.get(i).copied().unwrap_or(Vec2::ZERO), PHYSICS_DT)
            } else {
                [0.0; 4]
            };
            let thrust = drone.autopilot.spin(&airframe, commanded, PHYSICS_DT);

            let center = rb.translation();
            let mut yaw_torque = 0.0;
            for (&(x, z, spin), f) in ROTORS.iter().zip(thrust) {
                let point = center + rotation * Vec3::new(x * airframe.arm, 0.0, z * airframe.arm);
                rb.add_force_at_point(rotation * Vec3::new(0.0, f, 0.0), point, true);
                yaw_torque += spin * airframe.yaw_torque_per_thrust * f;
            }
            rb.add_torque(rotation * Vec3::new(0.0, yaw_torque, 0.0), true);

            let air = state.velocity - wind;
            rb.add_force(-(airframe.drag_linear + airframe.drag_quadratic * air.length()) * air, true);
        }
        self.world.step();
    }

    fn new_contacts(&mut self) -> Vec<(usize, usize)> {
        let mut now = HashSet::new();
        for pair in self.world.narrow_phase.contact_graph().interactions() {
            if !pair.has_any_active_contact() {
                continue;
            }
            if let (Some(&a), Some(&b)) = (self.owner.get(&pair.collider1), self.owner.get(&pair.collider2)) {
                now.insert((a.min(b), a.max(b)));
            }
        }
        let new = now.difference(&self.touching).copied().collect();
        self.touching = now;
        new
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fly(physics: &mut Physics, seconds: f32, setpoint: Vec2) {
        let n = physics.drones.len();
        physics.step(seconds, &vec![setpoint; n]);
    }

    fn velocity(physics: &Physics, i: usize) -> Vec3 {
        physics.world.bodies[physics.drones[i].body].linvel()
    }

    #[test]
    fn hovers_in_place() {
        let mut p = Physics::new(MAVIC_CLASS, &[Vec2::new(1.0, 2.0)]);
        fly(&mut p, 10.0, Vec2::ZERO);
        assert!((p.position(0) - Vec2::new(1.0, 2.0)).length() < 0.05, "drifted to {:?}", p.position(0));
        assert!((p.altitude(0) - CRUISE_ALTITUDE).abs() < 0.05, "altitude {}", p.altitude(0));
        assert!(p.tilt_degrees(0) < 1.0);
    }

    #[test]
    fn tracks_a_velocity_step_without_losing_altitude() {
        let mut p = Physics::new(MAVIC_CLASS, &[Vec2::ZERO]);
        let mut max_tilt: f32 = 0.0;
        let mut min_alt = f32::MAX;
        for _ in 0..60 {
            fly(&mut p, 0.05, Vec2::new(2.0, 0.0));
            max_tilt = max_tilt.max(p.tilt_degrees(0));
            min_alt = min_alt.min(p.altitude(0));
        }
        let v = velocity(&p, 0);
        assert!((v.x - 2.0).abs() < 0.1 && v.z.abs() < 0.1, "velocity after 3 s: {v:?}");
        assert!(max_tilt <= 36.0, "tilted {max_tilt}°");
        assert!(min_alt > CRUISE_ALTITUDE - 0.3, "sank to {min_alt}");
    }

    #[test]
    fn holds_position_against_steady_wind() {
        let mut p = Physics::new(MAVIC_CLASS, &[Vec2::ZERO]);
        p.wind = Vec2::new(5.0, 0.0);
        fly(&mut p, 15.0, Vec2::ZERO);
        let v = velocity(&p, 0);
        assert!(v.x.abs() < 0.05, "still drifting at {} m/s", v.x);
        // Leaning into the wind to hold still.
        assert!(p.tilt_degrees(0) > 2.0);
    }

    #[test]
    fn disarmed_drone_falls() {
        let mut p = Physics::new(MAVIC_CLASS, &[Vec2::ZERO]);
        p.disarm(0);
        fly(&mut p, 3.0, Vec2::ZERO);
        assert!(p.altitude(0) < 0.2, "still at {} m", p.altitude(0));
    }

    #[test]
    fn drones_flown_into_each_other_collide_once() {
        let mut p = Physics::new(MAVIC_CLASS, &[Vec2::new(-2.0, 0.0), Vec2::new(2.0, 0.0)]);
        let mut collisions = Vec::new();
        for _ in 0..60 {
            collisions.extend(p.step(0.05, &[Vec2::new(2.0, 0.0), Vec2::new(-2.0, 0.0)]));
            if !collisions.is_empty() {
                break;
            }
        }
        assert_eq!(collisions, vec![(0, 1)]);
    }

    #[test]
    fn takes_off_straight_up_then_flies_and_lands_on_command() {
        let mut p = Physics::new(MAVIC_CLASS, &[]);
        p.add_drone(Vec2::ZERO, true);
        // Ordered to fly east right away, it climbs first.
        fly(&mut p, 0.8, Vec2::new(2.0, 0.0));
        assert!(p.position(0).x.abs() < 0.05, "moved sideways on the ground: {:?}", p.position(0));
        fly(&mut p, 6.0, Vec2::new(2.0, 0.0));
        assert!((p.altitude(0) - CRUISE_ALTITUDE).abs() < 0.3, "altitude {}", p.altitude(0));
        assert!(p.position(0).x > 3.0);

        p.land(0);
        let mut landed = false;
        for _ in 0..200 {
            fly(&mut p, 0.05, Vec2::ZERO);
            if p.landed(0) {
                landed = true;
                break;
            }
        }
        assert!(landed, "still at {} m", p.altitude(0));
        assert!(p.tilt_degrees(0) < 5.0, "landed level");
    }
}
