//! Minimal 2D vector type. Not pulling in `glam` here on purpose — the drone HIL project
//! already hit a real version-compatibility snag tying its own glam usage to whatever version
//! rapier3d happened to depend on internally; this project has no physics engine dictating a
//! math-library version, so a few small hand-written methods keep that class of problem out
//! of the picture entirely. It also has to work in `no_std`, where `f32::sqrt` isn't available.

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn length(self) -> f32 {
        libm::sqrtf(self.x * self.x + self.y * self.y)
    }

    /// Neither coordinate NaN or infinite. Anything that came over the radio or from a sensor
    /// is checked with this before the brain uses it: one NaN in a position spreads to every
    /// distance, sum and velocity worked out from it.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    pub fn dot(self, other: Vec2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn normalized(self) -> Self {
        let len = self.length();
        if len > 0.0 {
            self * (1.0 / len)
        } else {
            Vec2::ZERO
        }
    }
}

impl core::ops::Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl core::ops::Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl core::ops::Mul<f32> for Vec2 {
    type Output = Vec2;
    fn mul(self, rhs: f32) -> Vec2 {
        Vec2::new(self.x * rhs, self.y * rhs)
    }
}
