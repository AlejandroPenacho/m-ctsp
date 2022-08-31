mod multi_dp;

use crate::graph::{Graph, GraphWH, WHNode};

pub trait Solver {

    fn optimize(&mut self, problem: &mut MTSP);

}

struct MTSP<'a> {
    n_agents: usize,
    n_targets: usize,
    graph: &'a GraphWH,
    paths: Vec<Vec<WHNode>>
}


impl<'a> MTSP<'a> {
    pub fn new(graph: &'a GraphWH) -> Self {

        let mut paths = vec![vec![]; (graph.get_n_agents()+1) as usize];
        paths[0] = (0..graph.get_n_targets()).collect();

        MTSP {
            graph,
            n_agents: graph.get_n_agents(),
            n_targets: graph.get_n_targets(),
            paths: vec![]
        }
    }
}
