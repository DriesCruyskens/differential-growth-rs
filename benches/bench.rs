use std::time::Duration;

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use differential_growth::{generate_points_on_circle, DifferentialGrowth};
use nalgebra::Point2;

pub fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("Run Differential Line", |b| {
        b.iter(|| {
            let starting_points: Vec<Point2<f64>> =
                generate_points_on_circle(100.0, 100.0, 10.0, 10);

            let mut line = DifferentialGrowth::new(
                starting_points,
                black_box(100.0),
                black_box(100.0),
                black_box(10.0),
                black_box(10.0),
                black_box(1.5),
            );
            for _ in 0..10 {
                line.tick();
            }
        })
    });
}

// Decreasing sample size since the default takes way to long with high ticks.
// High number of ticks are preferred so we test the system with a high number of nodes.
// https://bheisler.github.io/criterion.rs/book/user_guide/advanced_configuration.html#configuring-sample-count--other-statistical-settings
criterion_group! {
    name = benches;
    // This can be any expression that returns a `Criterion` object.
    config = Criterion::default().significance_level(0.1).sample_size(200).measurement_time(Duration::from_secs(10));
    targets = criterion_benchmark
}
criterion_main!(benches);
