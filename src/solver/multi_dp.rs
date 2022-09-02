use crate::graph;
use crate::graph::{WHNode,Graph};

use std::collections::HashSet;

pub struct MASolver<'a> {
    graph: &'a graph::GraphWH,
    next_node: Vec<Option<usize>>,
    agent_paths: Vec<Vec<usize>>
}

impl<'a> MASolver<'a> {
    pub fn new(graph: &'a graph::GraphWH) -> Self {
        MASolver {
            graph,
            next_node: vec![None; graph.get_n_agents() + graph.get_n_targets()],
            agent_paths: vec![vec![]; graph.get_n_agents()]
        }
    }

    pub fn get_agent_paths(&self) -> &[Vec<usize>] {
        return &self.agent_paths
    }

    pub fn compute_initial_assignment(&mut self) {
        let mut best_agent_match = vec![];
        let mut cumulative_agents_costs = vec![0; self.graph.get_n_agents()];
        let mut assigned_targets = HashSet::new();
        let mut n_assigned_targets = 0;

        for agent_index in 0..self.graph.get_n_agents() {
            best_agent_match.push(self.get_agent_best_next_target(agent_index, &assigned_targets));
        }

        loop {
            let best_assignment = best_agent_match.iter().enumerate().min_by_key(
                |(_agent_index, (_target_index, cost))| cost
            ).unwrap();

            let winner_agent = best_assignment.0;
            let assigned_target = best_assignment.1.0;
            let path_cost = best_assignment.1.1;
            
            self.agent_paths[winner_agent].push(assigned_target);
            assigned_targets.insert(assigned_target);
            n_assigned_targets += 1;
            cumulative_agents_costs[winner_agent] += path_cost;

            if n_assigned_targets == self.graph.get_n_targets() { break }

            for agent_index in 0..self.graph.get_n_agents() {
                let (next_target, cost) = self.get_agent_best_next_target(agent_index, &assigned_targets);
                best_agent_match[agent_index] = (next_target, cost + cumulative_agents_costs[agent_index])
            }
        }
    }

    fn get_agent_best_next_target(&self, agent_index: usize, assigned_targets: &HashSet<usize>) -> (usize, i32) {
        let (best_match, match_cost) =
            (0..self.graph.get_n_targets())
                .filter(|x| !assigned_targets.contains(x))
                .filter_map(
                |target_index| {
                    let x = (
                        target_index,
                        self.graph.get_arc_cost(WHNode::Agent(agent_index), WHNode::Target(target_index))
                    );

                    x.1.map(|cost| (x.0, cost))
                })
            .min_by_key(|(_index, cost)| *cost).unwrap();

        (best_match, match_cost)
    }

    pub fn iterate(&mut self, agent: usize) {

    }
}
