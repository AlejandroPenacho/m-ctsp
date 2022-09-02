pub mod multi_dp;

use crate::graph::{Graph, GraphWH, WHNode};

pub trait Solver {
    fn optimize(&mut self);
}
