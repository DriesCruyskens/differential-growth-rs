use std::ops::{MulAssign, AddAssign, SubAssign, DivAssign, Add, Div, Sub};

use kd_tree::KdTree2;
use nalgebra::{Point2, Vector2, distance};

use crate::node::Node;

/// The differential growth algorithm.
pub struct DifferentialGrowth {
    /// A Vec of Node objects.
    pub nodes: Vec<Node>,
    /// The maximum force nodes can exert on eachother.
    pub max_force: f64,
    ///  The maximum magnitude of a node's velocity.
    pub max_speed: f64,
    /// The desired separation between nodes.
    pub desired_separation: f64,
    /// The ratio between separation and cohesion forces.
    pub separation_cohesion_ration: f64,
    /// The maximum length between two connected nodes.
    pub max_edge_length: f64,
}

impl DifferentialGrowth {
    /// Returns a DifferentialGrowth instance with the given parameters.
    /// 
    /// # Arguments
    /// 
    /// * `input_nodes` - A Vec of starting points. These are converted into Nodes.
    /// * `max_force` - The maximum force nodes can exert on eachother.
    /// * `max_speed` - The maximum magnitude of a node's velocity.
    /// * `desired_separation` - The desired separation between nodes.
    /// * `separation_cohesion_ratio` - The ratio between separation and cohesion forces.
    /// * `max_edge_len` - The maximum length between two connected nodes.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// let starting_points = differential_growth::generate_points_on_circle(0.0, 0.0, 10.0, 10);
    /// 
    /// let differential_growth = differential_growth::DifferentialGrowth::new(starting_points, 1.5, 1.0, 14.0, 1.1, 5.0);
    /// ```
    /// 
    pub fn new(
        input_points: Vec<Point2<f64>>,
        max_force: f64,
        max_speed: f64,
        desired_separation: f64,
        separation_cohesion_ratio: f64,
        max_edge_len: f64,
    ) -> DifferentialGrowth {
        // Convert points to Nodes.
        let nodes: Vec<Node> =
            input_points
                .into_iter()
                .map(|point: Point2<f64>| Node::new(point, max_speed, max_force))
                .collect();

        DifferentialGrowth {
            nodes,
            max_force,
            max_speed,
            desired_separation,
            separation_cohesion_ration: separation_cohesion_ratio,
            max_edge_length: max_edge_len,
        }
    }

    /// Advanced the algorithm by 1 iteration.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// let starting_points = differential_growth::generate_points_on_circle(0.0, 0.0, 10.0, 10);
    /// let mut differential_growth = differential_growth::DifferentialGrowth::new(starting_points, 1.5, 1.0, 14.0, 1.1, 5.0);
    /// differential_growth.tick();
    /// ```
    /// 
    pub fn tick(&mut self) {
        self.differentiate();
        self.growth();
    }

    /// Get the positions of the current state of the nodes.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// let starting_points = differential_growth::generate_points_on_circle(0.0, 0.0, 10.0, 10);
    /// let mut differential_growth = differential_growth::DifferentialGrowth::new(starting_points, 1.5, 1.0, 14.0, 1.1, 5.0);
    /// differential_growth.tick();
    /// let points_to_draw = differential_growth.get_points();
    /// 
    /// // draw the result by
    /// // - drawing a line between consecutive Vec elements.
    /// // - drawing a line between the first and the last element.
    /// ```
    /// 
    pub fn get_points(&self) -> Vec<Point2<f64>> {
        let mut result = Vec::new();

        for node in &self.nodes {
            result.push(Point2::new(node.position.x, node.position.y));
        }

        result
    }

    fn insert_node_at(&mut self, node: Node, index: usize) {
        self.nodes.insert(index, node);
    }

    fn growth(&mut self) {
        let mut new_nodes: Vec<(Node, usize)> = Vec::with_capacity(self.nodes.len());
        let mut amount_nodes_added = 0;

        for i in 0..self.nodes.len() {
            let n1: &Node = &self.nodes[i];
            // Wrapping around to 0 if we are on last i.
            let n2: &Node = if i == self.nodes.len() - 1 {
                &self.nodes[0]
            } else {
                &self.nodes[i + 1]
            };

            let distance: f64 = distance(&n1.position, &n2.position);

            if distance > self.max_edge_length {
                // Inserting new nodes shifts the index of the original nodes.
                // To compensate we shift the index with it.
                let index: usize = i + 1 + amount_nodes_added;
                amount_nodes_added.add_assign(1);
                let middle_node: Vector2<f64> = n1.position.coords.add(n2.position.coords).div(2.0);
                new_nodes.push((
                    Node::new(
                        Point2::new(middle_node.x, middle_node.y),
                        self.max_speed,
                        self.max_force,
                    ),
                    index,
                ));
            }
        }

        for new_node in new_nodes {
            self.insert_node_at(new_node.0, new_node.1);
        }
    }

    fn differentiate(&mut self) {
        let separation_forces: Vec<Vector2<f64>> = self.get_separation_forces();
        let cohesion_forces: Vec<Vector2<f64>> = self.get_edge_cohesion_forces();

        for i in 0..self.nodes.len() {
            let mut separation: Vector2<f64> = separation_forces[i];
            let cohesion: Vector2<f64> = cohesion_forces[i];

            separation.mul_assign(self.separation_cohesion_ration);

            self.nodes[i].apply_force(&separation);
            self.nodes[i].apply_force(&cohesion);
            self.nodes[i].update();
        }
    }

    fn get_separation_forces(&self) -> Vec<Vector2<f64>> {
        // Constructing a kdtree each frame so we can optimise looking for neighbors.
        // This technique is the single most important optimisation we can do.
        let kdtree = KdTree2::build_by_ordered_float(self.nodes.clone());

        let nodes_len: usize = self.nodes.len();
        let mut separate_forces: Vec<Vector2<f64>> = vec![Vector2::default(); nodes_len];

        for i in 0..nodes_len {
            let nodei = &self.nodes[i];

            // We can assume no forces CAN happen outside of desired_separation range and
            // forces MUST happen withing desired_separation range.
            let close_nodes: Vec<&Node> = kdtree.within_radius(nodei, self.desired_separation);

            let _amount_of_close_nodes = close_nodes.len();

            for close_node in close_nodes {
                let force: Vector2<f64> = self.get_separation_force(nodei, close_node);
                separate_forces[i].add_assign(force);
            }

            // This doesn't do much difference visually so I think this can be removed
            // to minimise branching in a hot loop.
            if _amount_of_close_nodes > 0 {
                separate_forces[i].div_assign(_amount_of_close_nodes as f64);
            }

            // Set magnitude can make the separation force become a NaN value.
            // Since this breaks everything, x or y is set to 0 when NaN is detected.
            separate_forces[i].set_magnitude(self.max_speed);
            if separate_forces[i].x.is_nan() {separate_forces[i].x = 0.0;};
            if separate_forces[i].y.is_nan() {separate_forces[i].y = 0.0;};

            separate_forces[i].sub_assign(self.nodes[i].velocity);
            separate_forces[i] = separate_forces[i].cap_magnitude(self.max_force);
        }

        return separate_forces;
    }

    fn get_separation_force(&self, n1: &Node, n2: &Node) -> Vector2<f64> {
        let mut steer: Vector2<f64> = Vector2::default();

        // Optimised version by defering sqrt() to inside if statement.
        let distance_sq: f64 =
            (n2.position.x - n1.position.x).powi(2) + (n2.position.y - n1.position.y).powi(2);

        if distance_sq > 0.0 {
            let mut diff: Vector2<f64> = n1.position.sub(n2.position);
            diff = diff.normalize();
            diff.div_assign(distance_sq.sqrt());
            steer.add_assign(diff);
        }

        return steer;
    }

    fn get_edge_cohesion_forces(&self) -> Vec<Vector2<f64>> {
        let n: usize = self.nodes.len();
        let mut cohesion_forces: Vec<Vector2<f64>> = Vec::with_capacity(n);

        // I'm doing the cohesion force calculation of the first and last
        // node separately to prevent branching in a hot loop.

        // cohesion force of i == 0 (first node)
        {
            let mut sum: Vector2<f64> = Vector2::default();
            sum.add_assign(self.nodes[n - 1].position.coords);
            sum.add_assign(self.nodes[0 + 1].position.coords);
            sum.div_assign(2.0);
            cohesion_forces.push(self.nodes[0].seek(&sum));
        }

        // cohesion force of everything in between
        for i in 1..n - 1 {
            let mut sum: Vector2<f64> = Vector2::default();
            sum.add_assign(self.nodes[i - 1].position.coords);
            sum.add_assign(self.nodes[i + 1].position.coords);
            sum.div_assign(2.0);
            cohesion_forces.push(self.nodes[i].seek(&sum));
        }

        // cohesion force of i == n-1 (last node)
        {
            let mut sum: Vector2<f64> = Vector2::default();
            sum.add_assign(self.nodes[n - 1 - 1].position.coords);
            sum.add_assign(self.nodes[0].position.coords);
            sum.div_assign(2.0);
            cohesion_forces.push(self.nodes[n - 1].seek(&sum));
        }

        return cohesion_forces;
    }
}