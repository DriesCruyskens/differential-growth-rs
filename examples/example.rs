

use differential_growth::{generate_points_on_circle, DifferentialGrowth};
use nalgebra::Point2;
use nannou::{event::Update, prelude::*, window, App, Frame};

fn main() {
    nannou::app(model)
        .update(update)
        .run();
}

struct Model {
    _window: window::Id,
    differential_growth: DifferentialGrowth,
}

fn model(app: &App) -> Model {
    let _window = app.new_window().view(view).build().unwrap();

    // Generate a set of starting points.
    // You do not have to use the included helper function,
    // you could for example pass points drawn by the mouse.
    // nannou has the origin in the center of the window.
    let starting_points: Vec<Point2<f64>> = generate_points_on_circle(0.0, 0.0, 10.0, 10);

    // Instantiate DifferentialGrowth.
    // the choice of input parameters is very import to the
    // succesful working of the algorithm. A value to big or too small
    // or a wrong combination of values can make the algorithm behave
    // like it doesn't work.
    // Here I've provided values that I tested and like.
    let differential_growth: DifferentialGrowth =
        DifferentialGrowth::new(starting_points, 1.5, 1.0, 14.0, 1.1, 5.0);

    Model {
        _window,
        differential_growth,
    }
}

fn update(_app: &App, _model: &mut Model, _update: Update) {
    // Execute one iteration of the algorithm.
    _model.differential_growth.tick();
}

// colors: https://docs.rs/nannou/0.11.1/nannou/color/index.html#constants
fn view(app: &App, _model: &Model, frame: Frame) {
    // Get a Vector of points
    let points: Vec<Point2<f64>> = _model.differential_growth.get_points();

    let draw: Draw = app.draw();
    draw.background().color(MINTCREAM);

    // Drawing line between consecutive elements
    for window in points.windows(2) {
        let point1: Point2<f64> = window[0];
        let point2: Point2<f64> = window[1];

        draw.line()
            .start(Vec2::new(point1.x as f32, point1.y as f32))
            .end(Vec2::new(point2.x as f32, point2.y as f32))
            .color(NAVY);
    }

    // Drawing line between first and last element
    {
        let point1: Point2<f64> = points[0];
        let point2: Point2<f64> = points[points.len() - 1];

        draw.line()
            .start(Vec2::new(point1.x as f32, point1.y as f32))
            .end(Vec2::new(point2.x as f32, point2.y as f32))
            .color(NAVY);
    }

    draw.to_frame(app, &frame).unwrap();
}
