#![allow(dead_code)]

use crate::graph;
use crate::graph::{WHNode,Graph};

use std::collections::HashSet;



#[derive(Clone)]
struct State<'a> {
    n_agents: usize,
    positions: Vec<WHNode>,
    travelled_distances: Vec<i32>,
    paths: Vec<Vec<WHNode>>,
    remaining_nodes: Vec<WHNode>,
    final_nodes: &'a HashSet<WHNode>,
    total_cost: i32,
    active_robots: Vec<bool>,
    available_final_nodes: u8
}

impl<'a> State<'a> {
    fn new(
        agent_positions: Vec<WHNode>,
        travelled_distances: Vec<i32>,
        remaining_nodes: Vec<WHNode>,
        final_nodes: &'a HashSet<WHNode>) -> Self {

        let paths: Vec<Vec<WHNode>> = agent_positions.iter().map(|x| vec![x.clone()]).collect();

        let n_active_robots = agent_positions.len();

        let available_final_nodes: u8 = (n_active_robots - final_nodes.len() + 1).try_into().unwrap();

        State {
            n_agents: agent_positions.len(),
            positions: agent_positions,
            travelled_distances,
            paths,
            remaining_nodes,
            final_nodes,
            total_cost: 0,
            active_robots: vec![true; n_active_robots],
            available_final_nodes
        }
    }
}

/*
#[derive(Debug)]
struct AutoCycler {
    n_targets: usize,
    n_agents: usize,
    cycle_size: Vec<usize>
}

impl AutoCycler {
    fn new(n_agents: usize, n_targets: usize) -> Self {
        let mut cycle_size: Vec<usize> = Vec::new();
        let n = n_targets;
        let m = n_agents;

        println!("{}, {}", n,m);

        let n_combinations = factorial(n)/factorial(n-m);

        cycle_size.push(n_combinations);

        for i in 0..n_agents {
            cycle_size.push(cycle_size[i as usize] / (n-i));
        }

        Cycler {
            n_targets,
            n_agents,
            cycle_size
        }
    }

    fn get_index_combination(&self, index: usize, elements: &mut Vec<WHNode>) -> Option<Vec<WHNode>> {
        let remaining = elements;

        let mut out = Vec::new();

        for i_agent in 1..=self.n_agents {
            let element_index = (index %  self.cycle_size[i_agent-1] ) / self.cycle_size[i_agent];
            let selected_value = remaining.remove(element_index);
            if WHNode::Final == selected_value && element_index != 0 {
                return None
            }
            out.push(selected_value);
        }
        Some(out)
    }

    fn get_total_n_combinations(&self) -> usize { self.cycle_size[0] }
}
*/

#[derive(Debug)]
struct Cycler<'a> {
    n_agents: usize,
    nodes: &'a[WHNode],
    current_conf: Vec<Option<usize>>,
    available_nodes: Vec<u8>,
    started: bool
}

impl<'a> Cycler<'a> {
    fn new(n_agents: usize, nodes: &'a[WHNode], final_max_avail: u8) -> Self {
        if final_max_avail != 0 {
            assert!(matches!(nodes[0], WHNode::Final))
        }

        let mut available_nodes = nodes.iter().enumerate().map(|(i,_)| {
            if i == 0 && final_max_avail != 0 {
                final_max_avail
            } else { 1 }
        }).collect::<Vec<u8>>();

        let mut current_conf = vec![None; n_agents];

        for i in (0..n_agents).rev() {
            let target_index = available_nodes.iter().enumerate().find(|&(_,&x)| x != 0).unwrap().0;
            available_nodes[target_index] -= 1;
            current_conf[i] = Some(target_index);
        }

        Cycler { n_agents, nodes, current_conf, available_nodes, started: false }
    }
}

impl<'a> std::iter::Iterator for Cycler<'a> {
    type Item = Vec<WHNode>;

    fn next(&mut self) -> Option<Self::Item> {

        if !self.started {
            self.started = true;
            return Some(self.current_conf.iter().map(|&i| self.nodes[i.unwrap()].clone()).collect())
        }

        let mut c_agent = 0;
        loop {
            let c_agent_target_index = self.current_conf[c_agent];
            if let Some(i) = c_agent_target_index {
                self.available_nodes[i] += 1;
            }

            let next_target_index = self.available_nodes.iter()
                .enumerate()
                .skip(c_agent_target_index.map_or(0, |x| x+1))
                .find(|&(i,&x)| x != 0).map(|(i,x)| i);

            match next_target_index {
                None => {
                    self.current_conf[c_agent] = None;

                    c_agent += 1;
                    if c_agent == self.n_agents { return None }
                },
                Some(c_agent_target_index) => {
                    self.available_nodes[c_agent_target_index] -= 1;
                    self.current_conf[c_agent] = Some(c_agent_target_index);
                    if c_agent == 0 { break }
                    c_agent -= 1;
                }
            }
        }
        
        Some(self.current_conf.iter().map(|&i| self.nodes[i.unwrap()].clone()).collect())
    }
}


pub struct MASolver<'a> {
    graph: &'a graph::GraphWH,
    next_node: Vec<Option<usize>>,
    agent_paths: Vec<Vec<WHNode>>
}

impl<'a> MASolver<'a> {
    pub fn new(graph: &'a graph::GraphWH) -> Self {
        let agent_paths = (0..graph.get_n_targets()).map(|x| WHNode::Agent(x));

        MASolver {
            graph,
            next_node: vec![None; graph.get_n_agents() + graph.get_n_targets()],
            agent_paths: vec![vec![]; graph.get_n_agents()]
        }
    }

    pub fn get_agent_paths(&self) -> &[Vec<WHNode>] {
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
            
            self.agent_paths[winner_agent].push(WHNode::Target(assigned_target));
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

    fn get_agent_best_next_target(
        &self, agent_index: usize, assigned_targets: &HashSet<usize>) -> (usize, i32) {
        let (best_match, match_cost) =
            (0..self.graph.get_n_targets())
                .filter(|x| !assigned_targets.contains(x))
                .filter_map(
                |target_index| {
                    let x = (
                        target_index,
                        self.graph.get_arc_cost(&self.agent_paths[agent_index].iter().last().unwrap(), &WHNode::Target(target_index))
                    );
                    x.1.map(|cost| (x.0, cost))
                })
            .min_by_key(|(_index, cost)| *cost).unwrap();

        (best_match, match_cost)
    }

    fn expand_states(state: &State<'a>) -> Vec<State<'a>> {
        let n_active_agents = state.active_robots.iter().filter(|&&x| x).count();

        println!("{}", n_active_agents);

        let cycler = Cycler::new(n_active_agents, &state.remaining_nodes, state.available_final_nodes);
        let mut new_states: Vec<State> = Vec::new();

        for combination in cycler {
            let mut new_state = state.clone();

            for (next_target_index, agent_index) in (0..state.n_agents).filter(|&i| {
                    state.active_robots[i]
                }).enumerate() {

                let prev_agent_position = &new_state.positions[agent_index];
                let new_agent_position: WHNode = combination[next_target_index].clone();

                /*
                let distance = self.graph.get_arc_cost(
                    &state.positions[agent_index].into_real(&final_targets[agent_index]),
                    &next_targets[next_target_index].into_real(&final_targets[agent_index]),
                );
                */

                if let WHNode::Target(_) = new_agent_position {
                    new_state.remaining_nodes.remove(
                        new_state.remaining_nodes.iter().position(|x| x == &new_agent_position).unwrap()
                    );
                }
                if let WHNode::Final = new_agent_position {
                    new_state.available_final_nodes -= 1;
                }
                if state.final_nodes.contains(&new_agent_position) {
                    new_state.active_robots[agent_index] = false;
                }

                let distance = 1;

                new_state.positions[agent_index] = new_agent_position;
                new_state.travelled_distances[agent_index] += distance;
                new_state.total_cost += new_state.travelled_distances[agent_index];

            }

            if new_state.active_robots.iter().filter(|&&x| x).count() == 0 {
                if new_state.remaining_nodes.len() > 1 {
                    continue
                }
                if new_state.remaining_nodes.len() == 1
                    && matches!(new_state.remaining_nodes[0], WHNode::Final) {
                    continue
                }
            }
            println!("{:?}", new_state.positions);
            new_states.push(new_state);
        }
        new_states
    }
}


fn factorial(x: usize) -> usize {
    let mut out = 1;
    for i in 2..=x {
        out *= i;
    }
    out
}


#[cfg(test)]
mod test {
    use super::*;
    mod combinatorics {
        use super::*;
        #[test]
        fn simple() {
            use WHNode::*;

            let nodes = vec![ Final, Target(1), Target(2) ];

            let cycles = Cycler::new(
                2,
                &nodes,
                2
            );

            for conf in cycles {
                println!("{:?}", conf);
            }
        }

        #[test]
        fn expansion() {
            use WHNode::*;
            let mut final_nodes = HashSet::new();
            final_nodes.insert(Final);
            let state = State::new(
                vec![Agent(1), Agent(2)],
                vec![0, 0],
                vec![Final, Target(1), Target(2), Target(3)],
                &final_nodes
            );

            let new_states = MASolver::expand_states(&state);
        }
    }
}
