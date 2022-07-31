use nalgebra::Point2;
use std::{f64::consts::PI, ops::AddAssign};

/// Helper function that returns points on a circle.
/// 
/// # Examples
/// 
/// ```rust
/// let points = differential_growth::generate_points_on_circle(0.0, 0.0, 10.0, 10);
/// ```
/// 
pub fn generate_points_on_circle(
    origin_x: f64,
    origin_y: f64,
    radius: f64,
    amount_of_points: usize,
) -> Vec<Point2<f64>> {
    // https://www.mathopenref.com/coordcirclealgorithm.html

    let mut points: Vec<Point2<f64>> = Vec::new();

    let h: f64 = origin_x;
    let k: f64 = origin_y;

    let two_pi: f64 = 2.0 * PI as f64;
    let step: f64 = two_pi / amount_of_points as f64;
    let mut theta: f64 = 0.0;

    while theta < two_pi {
        let x: f64 = h + radius * f64::cos(theta);
        let y: f64 = k + radius * f64::sin(theta);
        points.push(Point2::new(x, y));
        theta.add_assign(step);
    }

    return points;
}
