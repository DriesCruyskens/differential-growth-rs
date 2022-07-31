use std::{fmt, ops::{AddAssign, MulAssign, Sub}};

use kd_tree::KdPoint;
use nalgebra::{Point2, Vector2};

#[derive(Copy, Clone)]
pub struct Node {
    pub position: Point2<f64>,
    pub velocity: Vector2<f64>,
    pub acceleration: Vector2<f64>,
    pub max_force: f64,
    pub max_speed: f64,
}

impl Node {
    pub fn new(position: Point2<f64>, max_speed: f64, max_force: f64) -> Node {
        Node {
            position,
            velocity: Vector2::default(),
            acceleration: Vector2::default(),
            max_speed,
            max_force,
        }
    }

    pub fn apply_force(&mut self, force: &Vector2<f64>) {
        self.acceleration.add_assign(force);
    }

    pub fn update(&mut self) {
        self.velocity.add_assign(self.acceleration);
        self.velocity = self.velocity.cap_magnitude(self.max_speed);
        self.position.add_assign(self.velocity);
        self.acceleration.mul_assign(0.0);
    }

    pub fn seek(&self, target: &Vector2<f64>) -> Vector2<f64> {
        let mut desired: Vector2<f64> = target.sub(self.position.coords);
        if desired.magnitude() != 0.0 {
            desired.set_magnitude(self.max_speed);
        }
        let steer: Vector2<f64> = desired.sub(self.velocity);
        return steer.cap_magnitude(self.max_force);
    }
}

impl fmt::Debug for Node {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Point")
            .field("pos", &self.position.coords)
            .field("vel", &self.velocity)
            .field("acc", &self.acceleration)
            .finish()
    }
}

// Somehow the nalgebra feature of kd-tree doesn't work so doing it manually.
impl KdPoint for Node {
    type Scalar = f64;
    type Dim = typenum::U2; // 2 dimensional tree.
    fn at(&self, k: usize) -> f64 {
        self.position[k]
    }
}
