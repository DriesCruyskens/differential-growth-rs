//! [![github]](https://github.com/driescruyskens/differential-growth-rust)&ensp;[![crates-io]](https://crates.io/crates/differential-growth)&ensp;[![docs-rs]](https://docs.rs/differential-growth)
//! 
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [docs-rs]: https://img.shields.io/badge/docs.rs-66c2a5?style=for-the-badge&labelColor=555555&logo=docs.rs
//! 
//! # Overview
//! 
//! ![example1](https://github.com/DriesCruyskens/differential-growth-rs/raw/main/images/example1.gif)
//! 
//! This crate provides a very easy to use Differential Growth algorithm as well as an example sketch using the excellent [nannou](https://crates.io/crates/nannou) crate.
//! It's as simple as providing a Vec of starting points, calling [`DifferentialGrowth::tick()`] to advance one iteration of the algorithm and obtaining the new state
//! using [`DifferentialGrowth::get_points()`].
//! 
//! Get up and running quickly by using an included point generator function like [`generate_points_on_circle()`] to generate your starting points.
//! 
//! Having a basic knowledge of the differential growth algorithm is highly recommended:
//! - <https://inconvergent.net/generative/differential-line/>
//! - <https://inconvergent.net/2016/shepherding-random-growth/>
//! 
//! # Example
//! 
//! The best way to understand this crate is by taking a look at `/example` folder. You can run it on any platform running `cargo run --example example`.
//! 
//! Otherwise, below is a quick reference on how to use this crate.
//! 
//! ```rust
//! # use differential_growth::{generate_points_on_circle, DifferentialGrowth};
//! # use nalgebra::Point2; 
//! # use nannou::{event::Update, prelude::*, window, App, Frame};

//! // Generate a set of starting points.
//! // You do not have to use the included helper function,
//! // you could for example pass points drawn by the mouse.
//! let starting_points: Vec<Point2<f64>> = generate_points_on_circle(0.0, 0.0, 10.0, 10);
//!
//! // Instantiate DifferentialGrowth.
//! // the choice of input parameters is very import to the
//! // succesful working of the algorithm. A value to big or too small
//! // or a wrong combination of values can make the algorithm behave
//! // like it doesn't work.
//! // Here I've provided values that I tested and like.
//! let mut dg = DifferentialGrowth::new(starting_points, 1.5, 1.0, 14.0, 1.1, 5.0);
//! 
//! // Advance the algorithm 1 iteration.
//! dg.tick();
//! 
//! // Get the newly calculated points.
//! let points_to_draw: Vec<Point2<f64>> = dg.get_points();
//! 
//! // draw the result by
//! // - drawing a line between consecutive Vec elements.
//! // - drawing a line between the first and the last element.
//! ```
//! 

mod differential_growth;
mod node;
#[cfg(feature = "point_generators")]
mod point_generators;

pub use crate::differential_growth::*;
#[cfg(feature = "point_generators")]
pub use crate::point_generators::*;
