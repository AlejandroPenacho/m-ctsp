#![allow(dead_code)]

use crate::graph;
use crate::graph::{WHNode,Graph};

use std::collections::{HashSet,HashMap};


#[derive(Clone)]
struct State {
    paths: Vec<Vec<WHNode>>,
    remaining_nodes: Vec<WHNode>,
    available_final_nodes: u8
}

impl State {
    fn new(
        agent_positions: Vec<WHNode>,
        remaining_nodes: Vec<WHNode>,
        available_final_nodes: u8) -> Self {

        let paths: Vec<Vec<WHNode>> = agent_positions.iter().map(|x| vec![x.clone()]).collect();

        State {
            paths,
            remaining_nodes,
            available_final_nodes
        }
    }
}

struct ReducedProblem<'a> {
    graph: &'a graph::GraphWH,
    initial_distances: Vec<i32>,
    final_nodes: HashMap<WHNode, Vec<WHNode>>,
    queue: Vec<(State, i32)>,
    best_solution: Option<(Vec<Vec<WHNode>>, i32)>,
    heuristic: Option<fn(&ReducedProblem, &State) -> i32>
}

impl<'a> ReducedProblem<'a> {
    fn new(
        graph: &'a graph::GraphWH,
        initial_distances: Vec<i32>,
        initial_positions: Vec<WHNode>,
        remaining_nodes: Vec<WHNode>,
        final_nodes: HashMap<WHNode, Vec<WHNode>>,
        initial_solution: Option<Vec<Vec<WHNode>>>) -> Self {


        let n_agents = initial_positions.len();

        let available_final_nodes = if remaining_nodes.len() != 0 && remaining_nodes[0] == WHNode::Final {
            n_agents - final_nodes.len() + 1
        } else {
            assert_eq!(n_agents, final_nodes.len());
            0
        };

        let first_state = State::new(
            initial_positions,
            remaining_nodes,
            available_final_nodes.try_into().unwrap()
        );

        let mut output = ReducedProblem {
            graph,
            initial_distances: initial_distances.clone(),
            final_nodes,
            queue: vec![(first_state, 0)],
            best_solution: None,
            heuristic: Some(compute_simple_heuristic_cost)
            // heuristic: None
        };

        let initial_solution = initial_solution.map(|paths| {
            let total_cost = paths.iter()
                .enumerate()
                .map(|(index, path)| output.get_partial_path_cost(path, index))
                .sum::<i32>();

            (paths, total_cost)
        });

        output.best_solution = initial_solution;
        output
    }

    fn expand_state(&self, state: &State) -> Vec<State> {
        // Get the number of agents that have not reached a final node yet
        let n_active_agents = state.paths.iter().filter(|&x|
            !self.final_nodes.contains_key(x.last().unwrap())
        ).count();

        // Get the real total number of agents
        let n_agents = state.paths.len();

        // The cycler generates all possible next actions for the active agents
        let cycler = Cycler::new(n_active_agents, &state.remaining_nodes, state.available_final_nodes);

        let mut new_states: Vec<State> = Vec::new();
        for combination in cycler {
            let mut new_state = state.clone();

            // next_target_index corresponds to the index of the agent among
            // active agents, and agent_index in the index among all agents
            // The first one correponds to the index of the next node in the
            // combination
            for (next_target_index, agent_index) in (0..n_agents).filter(|&i| {
                    !self.final_nodes.contains_key(state.paths[i].iter().last().unwrap())
                }).enumerate() {

                // let prev_agent_position = new_state.paths[agent_index].iter().last().unwrap();

                // Take the new agent position from the combination
                let new_agent_position: WHNode = combination[next_target_index].clone();

                // If the new position is a target, remove it from the
                // remaining nodes
                if let WHNode::Target(_) = new_agent_position {
                    new_state.remaining_nodes.remove(
                        new_state.remaining_nodes.iter().position(|x| x == &new_agent_position).unwrap()
                    );
                }

                // If it is a final node, reduced the number of available final
                // nodes by one. If it is the last final node, remove the node
                // from the remaining nodes
                if let WHNode::Final = new_agent_position {
                    new_state.available_final_nodes -= 1;
                    if new_state.available_final_nodes == 0 { new_state.remaining_nodes.remove(0); }
                }

                new_state.paths[agent_index].push(new_agent_position.clone());

            }

            new_states.push(new_state);
        }

        new_states
    }

    fn iterate(&mut self) -> bool {
        //Perform one iteration of the algorithm, expanding a single state and
        //adding the valid new states to the queue

        // If there are no states left, return inmediately, singaling the end
        // of the algorithm

        if self.queue.len() == 0 { return true };

        let best_state_index = if self.heuristic.is_some() {
            self.queue.iter()
            .enumerate()
            .min_by_key(|(_,x)| x.1).unwrap().0
        } else {
            0
        };

        // Take one state from the queue
        let state = self.queue.swap_remove(best_state_index).0;

        // Expand the state
        let new_states = self.expand_state(&state);

        // Not all states obtained from the expansion are valid, so the list
        // must be filtered
        for new_state in new_states {
            // Whether there are no remaining nodes to be covered by the agents
            let all_nodes_covered = 
                new_state.remaining_nodes.len() == 0 ||
                (
                    new_state.remaining_nodes.len() == 1
                    && new_state.remaining_nodes[0] == WHNode::Final
                    && new_state.available_final_nodes == 0
                );

            // Whether all agents have reached a final node
            let all_agents_done = new_state.paths
                .iter()
                .filter(|p| !self.final_nodes.contains_key(p.iter().last().unwrap())).count() == 0;

            // If the state is complete (a valid final solution), compute its
            // cost and, if it is better than the current, replace it
            if all_nodes_covered && all_agents_done {
                let cost = new_state.paths.iter().enumerate()
                    .map(|(index, path)| {
                            self.get_partial_path_cost(path, index)
                        })
                    .sum();

                if self.best_solution.as_ref().map_or(true, |(_, prev_best_cost)| &cost < prev_best_cost) {
                    self.best_solution = Some((new_state.paths, cost));
                }
                continue
            }
            // If all agents are finished but not all nodes covered, the state
            // can not be expanded and will not reach a complete solution. So,
            // it is not added to the queue
            if all_agents_done && !all_nodes_covered { continue }

            let heuristic_cost = self.heuristic.map_or(0, |heuristic| heuristic(&self, &new_state));


            if self.best_solution.as_ref().map_or(false, |x| x.1 < heuristic_cost) { continue }

            self.queue.push((new_state, heuristic_cost));
        }

        false
    }

    fn optimize(mut self) -> Vec<Vec<WHNode>> {
        loop {
            let is_finished = self.iterate();
            if is_finished { break }
        }
        let mut paths = self.best_solution.unwrap().0;
        
        for path in paths.iter_mut() {
            path.append(&mut self.final_nodes.get_mut(path.iter().last().unwrap()).unwrap());
        }

        paths
    }

    /*
    fn alt_get_partial_path_cost(&self, path: &[WHNode], index: usize) -> i32 {
        let mut complete_path = Vec::from(path);
        complete_path.append(&mut self.final_nodes.get(path.iter().last().unwrap()).unwrap().clone());
        let mut distance = self.initial_distances[index];
        let mut cost = 0;

        for i in 0..(complete_path.len()-1) {
            distance += self.graph.get_arc_cost(&complete_path[i], &complete_path[i+1]).unwrap();
            cost += distance;
        }
        cost
    }
    */

    fn get_partial_path_cost(&self, path: &[WHNode], index: usize) -> i32 {
        let mut distance = self.initial_distances[index];
        let mut cost = 0;

        if path.len() == 1 {
            return 0
        }

        for i in 0..(path.len()-2) {
            distance += self.graph.get_arc_cost(&path[i], &path[i+1]).unwrap();
            cost += distance;
        }
        let last_node = path.iter().last().unwrap();

        if last_node != &WHNode::Final {
            distance += self.graph.get_arc_cost(&path[path.len()-2], &path[path.len()-1]).unwrap();
            cost += distance;
            cost += distance * (self.final_nodes.get(last_node).map_or(1, |x| x.len()) as i32 - 1);
        }

        cost
    }
}

fn compute_simple_heuristic_cost(reduced_problem: &ReducedProblem, state: &State) -> i32 {
    let mut total_cost = 0;
    let mut travelled_distances = Vec::new();


    for (i, path) in state.paths.iter().enumerate() {
        let mut travelled_distance = reduced_problem.initial_distances[i];
        total_cost += reduced_problem.get_partial_path_cost(&path, i);
        for node_index in 0..(path.len()-1) {
            travelled_distance += reduced_problem.graph
                .get_arc_cost(&path[node_index], &path[node_index+1]).unwrap();
        }
        travelled_distances.push(travelled_distance);
    }
    println!("{:?}", state.paths);
    println!("{travelled_distances:?}");

    for node in state.remaining_nodes.iter().filter(|&&node| node != WHNode::Final) {
        let mut best_collection_time = state.paths.iter()
            .enumerate()
            .map(|(i,x)| (i, x.iter().last().unwrap()))
            .filter(|(_,x)| !reduced_problem.final_nodes.contains_key(x))
            .map(|(i,x)| (i, travelled_distances[i] + reduced_problem.graph.get_arc_cost(x, node).unwrap()))
            .min_by_key(|(_,x)| *x).unwrap().1;

        if let Some(x) = reduced_problem.final_nodes.get(node) {
            best_collection_time *= x.len() as i32;
        }

        total_cost += best_collection_time;
    }

    total_cost
}

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

        let mut available_nodes = nodes.iter().enumerate().map(|(i,node)| {
            if i == 0 && node == &WHNode::Final {
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
    agent_paths: Vec<Vec<WHNode>>
}

impl<'a> MASolver<'a> {
    pub fn new(graph: &'a graph::GraphWH) -> Self {
        let agent_paths = (0..graph.get_n_agents()).map(|x| vec![WHNode::Agent(x)]).collect();

        MASolver {
            graph,
            agent_paths
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

        for path in self.agent_paths.iter_mut() {
            path.push(WHNode::Final);
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

    fn split_problem(&mut self, endpoints: &[(usize, (usize,usize))]) -> ReducedProblem {
        // By keeping the original paths, we have an initial solution for
        // pruning
        let mut original_paths: Vec<Vec<WHNode>> = Vec::new();
        // A list with the nodes that can (and must) be used in the optimization
        let mut remaining_nodes: Vec<WHNode> = Vec::new();
        // The initial positions of the agents in the reduced problem
        let mut initial_nodes: Vec<WHNode> = Vec::new();
        // The final nodes
        let mut final_nodes: HashMap<WHNode, Vec<WHNode>> = HashMap::new();
        let mut initial_travel_distances: Vec<i32> = Vec::new();

        let mut final_node_available = false;

        //For each agent, split the path and add it to the reduced problem
        for (agent_index, (i_0, i_f)) in endpoints.iter() {

            /*
                For i_0=3 and i_f=6:

                0---1---2---3---4---5---6---7---8

                             |||
                             VVV

                0---1---2       - Path that stays in the original problem   self.agent_paths[agent_index]
                3               - Initial node for the reduced problem      agent_initial_node
                4---5---6       - Nodes in  "remaining"                     splitted_nodes
                6               - Node in final nodes                       final_nodes[agent_index]
                7---8           - Final chain connected to 6                final_chain
                    
            */

            // These are the nodes that stand after the splitted part
            let final_chain: Vec<WHNode> = self.agent_paths[*agent_index].split_off(i_f+1);

            // Nodes between i_0 (exclusive) and i_f (inclusive), that are liberated in the new
            // problem.
            let mut splitted_nodes: Vec<WHNode> = self.agent_paths[*agent_index].split_off(i_0+1);

            // The original path is a copy of the path that would be obtained
            // in the reduced problem, that would produce the same path in the
            // complete problem
            let mut agent_original_path = vec![self.agent_paths[*agent_index].iter().last().unwrap().clone()];
            agent_original_path.append(&mut splitted_nodes.clone());
            original_paths.push(agent_original_path);

            // The node in which the agent starts the reduced problem
            let agent_initial_node = self.agent_paths[*agent_index].pop().unwrap();

            // Nodes that, when reached by an agent, terminate its path. They
            // are either final nodes (WHNode::Final), or are associated to a
            // chain that ends in a final node.
            final_nodes.insert(splitted_nodes.iter().last().unwrap().clone(), final_chain);

            initial_nodes.push(agent_initial_node.clone());

            // If the final node is a final node, it must be added to the
            // remaining nodes only once.
            if splitted_nodes.iter().last().unwrap() == &WHNode::Final {
                final_node_available = true;
                splitted_nodes.pop();
            }

            remaining_nodes.append(&mut splitted_nodes);

            // Compute the travelled distance of the agent when it is in the
            // initial node. This affects the cost of all nodes it takes.
            let mut initial_distance = 0;
            if !(self.agent_paths[*agent_index].len() == 0) {
                for i in 0..(self.agent_paths[*agent_index].len()-1) {
                    initial_distance += self.graph.get_arc_cost(
                        &self.agent_paths[*agent_index][i],
                        &self.agent_paths[*agent_index][i+1]
                    ).unwrap();
                }

                initial_distance += self.graph.get_arc_cost(
                        self.agent_paths[*agent_index].iter().last().unwrap(),
                        &agent_initial_node
                    ).unwrap();
            }

            initial_travel_distances.push(initial_distance);
        }

        if final_node_available {
            remaining_nodes.push(WHNode::Final);
            remaining_nodes = remaining_nodes.into_iter().rev().collect();
        }

        let reduced_problem = ReducedProblem::new(
            &self.graph,
            initial_travel_distances,
            initial_nodes,
            remaining_nodes,
            final_nodes,
            None
        );


        reduced_problem
    }

    fn join_problem(&mut self, new_paths: Vec<Vec<WHNode>>, endpoints: &[(usize, (usize,usize))]) {
        for (index, mut new_path) in new_paths.into_iter().enumerate() {
            let agent_index = endpoints[index].0;
            self.agent_paths[agent_index].append(&mut new_path);
        }
    }

    fn solve_reduced(&mut self, endpoints: &[(usize, (usize,usize))]) {
        // Check that the same agent has not been added twice to the
        // optimization and that the last optimized node is at much the final
        // one
        for (index, (agent_index, (_, i_f))) in endpoints.iter().enumerate() {
            for (agent_index_2, _) in endpoints.iter().skip(index+1) {
                if agent_index == agent_index_2 { panic!() }
                if self.agent_paths[*agent_index].len() <= *i_f { panic!() }
            }
        }

        let reduced_problem = self.split_problem(endpoints);

        let new_paths = reduced_problem.optimize();

        self.join_problem(new_paths, endpoints);
    }

    fn compute_total_cost(&self) -> i32 {
        self.agent_paths.iter().map(|path| {
            compute_complete_path_cost(self.graph, path).unwrap()
        }).sum()
    }
}

fn compute_complete_path_cost(graph: &graph::GraphWH, path: &[WHNode]) -> Option<i32> {
    let mut cost = 0;
    let mut distance = 0;

    assert!(path.len() != 0, "A path can not be empty");

    // Ignore the last arc, which goes to the final node
    for i in 0..(path.len()-2) {
        distance += graph.get_arc_cost(&path[i], &path[i+1]).unwrap();
        cost += distance;
    }
    Some(cost)
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::graph::GraphWH;

    fn simple_graph() -> GraphWH {
        GraphWH::create_test_graph(
            3,
            7,
            vec![
                vec![ 1, 6, 5, 5, 2, 3, 8],
                vec![ 8, 5, 1, 3, 8, 2, 4],
                vec![ 1, 2, 3, 7, 5, 6, 5]
            ],
            vec![
                vec![ 2, 4, 6, 2, 6, 4, 3],
                vec![ 3, 7, 1, 4, 6, 2, 5],
                vec![ 1, 1, 4, 5, 3, 1, 9],
                vec![ 9, 6, 4, 5, 2, 4, 3],
                vec![ 7, 4, 3, 2, 2, 8, 2],
                vec![ 5, 3, 6, 4, 3, 5, 7],
                vec![ 3, 5, 7, 3, 7, 3, 5]
            ]
        )
    }

    mod cycler {
        use super::*;
        #[test]
        fn simple() {
            use super::WHNode::*;
            let cycler = Cycler::new(
                2,
                &[Target(0), Target(1), Target(2)],
                0
            );

            let all_combinations = cycler.collect::<Vec<Vec<WHNode>>>();
            assert_eq!(
                vec![
                    vec![Target(1), Target(0)],
                    vec![Target(2), Target(0)],
                    vec![Target(0), Target(1)],
                    vec![Target(2), Target(1)],
                    vec![Target(0), Target(2)],
                    vec![Target(1), Target(2)]
                ],
                all_combinations
            );
        }
        #[test]
        fn with_exhausted_final() {
            use super::WHNode::*;
            let cycler = Cycler::new(
                2,
                &[Final, Target(0), Target(1), Target(2)],
                0
            );

            let all_combinations = cycler.collect::<Vec<Vec<WHNode>>>();
            assert_eq!(
                vec![
                    vec![Target(1), Target(0)],
                    vec![Target(2), Target(0)],
                    vec![Target(0), Target(1)],
                    vec![Target(2), Target(1)],
                    vec![Target(0), Target(2)],
                    vec![Target(1), Target(2)]
                ],
                all_combinations
            );
        }
        #[test]
        fn with_one_final() {
            use super::WHNode::*;
            let cycler = Cycler::new(
                2,
                &[Final, Target(0), Target(1), Target(2)],
                1
            );

            let all_combinations = cycler.collect::<Vec<Vec<WHNode>>>();
            assert_eq!(
                vec![
                    vec![Target(0), Final],
                    vec![Target(1), Final],
                    vec![Target(2), Final],
                    vec![Final,     Target(0)],
                    vec![Target(1), Target(0)],
                    vec![Target(2), Target(0)],
                    vec![Final,     Target(1)],
                    vec![Target(0), Target(1)],
                    vec![Target(2), Target(1)],
                    vec![Final,     Target(2)],
                    vec![Target(0), Target(2)],
                    vec![Target(1), Target(2)],
                ],
                all_combinations
            );
        }
        #[test]
        fn with_two_final() {
            use super::WHNode::*;
            let cycler = Cycler::new(
                2,
                &[Final, Target(0), Target(1), Target(2)],
                2
            );

            let all_combinations = cycler.collect::<Vec<Vec<WHNode>>>();
            assert_eq!(
                vec![
                    vec![Final, Final],
                    vec![Target(0), Final],
                    vec![Target(1), Final],
                    vec![Target(2), Final],
                    vec![Final,     Target(0)],
                    vec![Target(1), Target(0)],
                    vec![Target(2), Target(0)],
                    vec![Final,     Target(1)],
                    vec![Target(0), Target(1)],
                    vec![Target(2), Target(1)],
                    vec![Final,     Target(2)],
                    vec![Target(0), Target(2)],
                    vec![Target(1), Target(2)],
                ],
                all_combinations
            );
        }
        #[test]
        fn three_agents() {
            use super::WHNode::*;
            let cycler = Cycler::new(
                3,
                &[Final, Target(0), Target(1), Target(2)],
                2
            );

            let all_combinations = cycler.collect::<Vec<Vec<WHNode>>>();
            assert_eq!(
                vec![
                    vec![Target(0), Final,      Final],
                    vec![Target(1), Final,      Final],
                    vec![Target(2), Final,      Final],
                    vec![Final,     Target(0),  Final],
                    vec![Target(1), Target(0),  Final],
                    vec![Target(2), Target(0),  Final],
                    vec![Final,     Target(1),  Final],
                    vec![Target(0), Target(1),  Final],
                    vec![Target(2), Target(1),  Final],
                    vec![Final,     Target(2),  Final],
                    vec![Target(0), Target(2),  Final],
                    vec![Target(1), Target(2),  Final],

                    vec![Final,     Final,      Target(0)],
                    vec![Target(1), Final,      Target(0)],
                    vec![Target(2), Final,      Target(0)],
                    vec![Final,     Target(1),  Target(0)],
                    vec![Target(2), Target(1),  Target(0)],
                    vec![Final,     Target(2),  Target(0)],
                    vec![Target(1), Target(2),  Target(0)],

                    vec![Final,     Final,      Target(1)],
                    vec![Target(0), Final,      Target(1)],
                    vec![Target(2), Final,      Target(1)],
                    vec![Final,     Target(0),  Target(1)],
                    vec![Target(2), Target(0),  Target(1)],
                    vec![Final,     Target(2),  Target(1)],
                    vec![Target(0), Target(2),  Target(1)],

                    vec![Final,     Final,      Target(2)],
                    vec![Target(0), Final,      Target(2)],
                    vec![Target(1), Final,      Target(2)],
                    vec![Final,     Target(0),  Target(2)],
                    vec![Target(1), Target(0),  Target(2)],
                    vec![Final,     Target(1),  Target(2)],
                    vec![Target(0), Target(1),  Target(2)],
                ],
                all_combinations
            );
        }
    }
    
    mod split {
        use super::*;
        #[test]
        fn basic_split() {
            use WHNode::*;
            let graph = simple_graph();
            let mut problem = MASolver::new(&graph);

            problem.agent_paths = vec![
                vec![Agent(0), Target(1), Target(2), Target(3), Final],
                vec![Agent(1), Target(4), Target(5), Target(6), Final]
            ];

            let split_problem = problem.split_problem(
                &[(0, (0, 2)), (1,(2,4))]
            );

            // Check initial distances
            assert_eq!(
                vec![
                    0,
                    graph.get_arc_cost(&Agent(1), &Target(4)).unwrap()
                    + graph.get_arc_cost(&Target(4),&Target(5)).unwrap()
                ],
                split_problem.initial_distances
            );

            // Check final node chains
            assert_eq!(
                Some(&vec![Target(3), Final]),
                split_problem.final_nodes.get(&Target(2))
            );
            assert_eq!(
                Some(&vec![]),
                split_problem.final_nodes.get(&Final)
            );


            // Check agent paths in original problem
            assert_eq!(
                vec![vec![], vec![Agent(1), Target(4)]],
                problem.agent_paths
            );
        }
    }

    mod cost {
        use super::*;

        #[test]
        fn basic_cost() {
            use WHNode::*;
            let graph = simple_graph();
            let mut problem = MASolver::new(&graph);

            problem.agent_paths = vec![
                vec![Agent(0), Target(1), Target(2), Target(3), Final],
                vec![Agent(1), Target(4), Target(5), Target(6), Final]
            ];

            let split_problem = problem.split_problem(
                &[(0, (0, 2)), (1,(2,4))]
            );

            let partial_paths = vec![
                vec![Agent(0), Target(4), Target(6), Target(2)],
                vec![Target(5), Target(2), Target(4), Final]
            ];

            let complete_paths = vec![
                //  Partial path-------------------------|------------Final chain
                vec![Agent(0), Target(4), Target(6), Target(2), Target(3), Final],
                vec![Agent(1), Target(4), Target(5), Target(2), Target(4), Final]
                //  Initial path-------------|---Partial path----------------|-Final chain
            ];

            // Check partial costs for arbitrary partial paths
            assert_eq!(
                graph.get_arc_cost(&Agent(0), &Target(4)).unwrap() * 4
                + graph.get_arc_cost(&Target(4), &Target(6)).unwrap() * 3
                + graph.get_arc_cost(&Target(6), &Target(2)).unwrap() * 2,

                split_problem.get_partial_path_cost(
                    &partial_paths[0],
                    0
                )
            );
            assert_eq!(
                split_problem.initial_distances[1] * 2
                + graph.get_arc_cost(&Target(5), &Target(2)).unwrap() * 2
                + graph.get_arc_cost(&Target(2), &Target(4)).unwrap() * 1
                + graph.get_arc_cost(&Target(4), &Final).unwrap() * 0,

                split_problem.get_partial_path_cost(
                    &partial_paths[1],
                    1
                )
            );

                // Check equivalence between partial and complete costs

            // When there are nodes after the final node of the partial path,
            // the complete cost must include the travel cost there, with
            // initial distance 0
            assert_eq!(
                split_problem.get_partial_path_cost(&partial_paths[0], 0) + 
                graph.get_arc_cost(&Target(2), &Target(3)).unwrap(),
                compute_complete_path_cost(&graph, &complete_paths[0]).unwrap()
            );

            // When there are initial nodes, their distance must be added to
            // the partial one.
            assert_eq!(
                graph.get_arc_cost(&Agent(1), &Target(4)).unwrap() * 2 +
                graph.get_arc_cost(&Target(4), &Target(5)).unwrap() +
                split_problem.get_partial_path_cost(&partial_paths[1], 1) ,
                compute_complete_path_cost(&graph, &complete_paths[1]).unwrap()
            )
        }
    }

    mod join {
        use super::*;
        #[test]
        fn simple_join() {
            use WHNode::*;
            let graph = simple_graph();
            let mut problem = MASolver::new(&graph);

            let endpoints = [(0, (0, 2)), (1,(2,4))];

            problem.agent_paths = vec![
                vec![Agent(0), Target(1), Target(2), Target(3), Final],
                vec![Agent(1), Target(4), Target(5), Target(6), Final]
            ];

            let _split_problem = problem.split_problem(&endpoints);

            // The paths obtained in the split problem already include the
            // final nodes
            let partial_paths = vec![
                vec![Agent(0), Target(4), Target(6), Target(2), Target(3), Final],
                vec![Target(5), Target(2), Target(4), Final]
            ];

            problem.join_problem(partial_paths, &endpoints);

            assert_eq!(
                vec![
                    vec![Agent(0), Target(4), Target(6), Target(2), Target(3), Final],
                    vec![Agent(1), Target(4), Target(5), Target(2), Target(4), Final]
                ],
                problem.agent_paths
            );
        }
    }

    mod major_tests {
        use super::*;

        #[ignore]
        #[test]
        fn the_cut() {
            let graph = GraphWH::create_random(20, 4, 20);
            let mut problem = MASolver::new(&graph);
            problem.compute_initial_assignment();
            for path in problem.agent_paths.iter() {
                println!("{:?}", path);
            }

            println!("-----------------------------------");

            problem.solve_reduced(&[(0, (2, 3)), (1,(0,1))]);
        }

        #[test]
        #[ignore]
        fn much_optimal() {
            let n_agents = 4;
            let graph = GraphWH::create_random(20, n_agents, 12);
            let mut problem = MASolver::new(&graph);
            problem.compute_initial_assignment();

            for path in problem.agent_paths.iter() {
                println!("{:?}", path);
            }
            println!("Cost:{}", problem.compute_total_cost());

            for i in 0..n_agents {
                for j in (i+1)..n_agents {
                    problem.solve_reduced(&[
                        (i, (0, problem.agent_paths[i].len()-1)),
                        (j, (0, problem.agent_paths[j].len()-1))
                    ]);
                    println!("\n\nOptimized {} and {}:\n", i, j);
                    for path in problem.agent_paths.iter() {
                        println!("{:?}", path);
                    }
                    println!("Cost:{}", problem.compute_total_cost());
                }
            }
        }

        #[test]
        #[ignore]
        fn mini_optimal() {
            use WHNode::*;
            let n_agents = 4;
            let graph = GraphWH::create_random(20, n_agents, 7);

            println!("Cost R0->0: {}, 0->1: {},  cost 1->2: {}, cost 2->3: {}\n", 
                graph.get_arc_cost(&Agent(0), &Target(0)).unwrap(),
                graph.get_arc_cost(&Target(0), &Target(1)).unwrap(),
                graph.get_arc_cost(&Target(1), &Target(2)).unwrap(),
                graph.get_arc_cost(&Target(2), &Target(3)).unwrap(),
            );
            println!("Cost R1->4: {},  cost 4->5: {}, cost 5->6: {}, cost 6->3: {}\n", 
                graph.get_arc_cost(&Agent(1), &Target(4)).unwrap(),
                graph.get_arc_cost(&Target(4), &Target(5)).unwrap(),
                graph.get_arc_cost(&Target(5), &Target(6)).unwrap(),
                graph.get_arc_cost(&Target(6), &Target(3)).unwrap(),
            );

            let mut problem = MASolver::new(&graph);
            problem.agent_paths = vec![
                vec![Agent(0), Target(0), Target(1), Target(2), Target(3), Final],
                vec![Agent(1), Target(4), Target(5), Target(6), Final]
            ];

            for path in problem.agent_paths.iter() {
                println!("{:?}", path);
            }
            println!("Cost:{}\n\n", problem.compute_total_cost());

            problem.solve_reduced(
                &[
                    (0, (3,4)),
                    (1, (3,4))
                ]
            );

            for path in problem.agent_paths.iter() {
                println!("{:?}", path);
            }
            println!("Cost:{}", problem.compute_total_cost());
        }

        #[test]
        #[ignore]
        fn other_optimal() {
            let n_agents = 4;
            let graph = GraphWH::create_random(20, n_agents, 20);
            let mut problem = MASolver::new(&graph);
            problem.compute_initial_assignment();
            println!("Cost:{}\n\n", problem.compute_total_cost());

            for i in 0..5 {
                let mut endpoints = Vec::new();
                for agent_index in 0..n_agents {
                    if problem.agent_paths[agent_index].len() > (i + 2) {
                        endpoints.push((agent_index, (i, i+2)));
                    }
                }
        
                println!("\n\nEndpoints: {:?}", endpoints);
                problem.solve_reduced(&endpoints);

                for path in problem.agent_paths.iter() {
                    println!("{:?}", path);
                }
                println!("Cost:{}", problem.compute_total_cost());
            }
        }

        #[test]
        #[ignore]
        fn please() {
            let graph = GraphWH::create_random(20,2,3);
            let mut problem = MASolver::new(&graph);
            problem.compute_initial_assignment();

            for agent in 0..2 {
                for target in 0..3 {
                    println!("Agent {agent} to Target {target}: {}", graph.get_arc_cost(&WHNode::Agent(agent), &WHNode::Target(target)).unwrap());
                }
            }
            for target_1 in 0..3 {
                for target_2 in 0..3 {
                    println!("Target {target_1} to Target {target_2}: {}", graph.get_arc_cost(&WHNode::Target(target_1), &WHNode::Target(target_2)).unwrap());
                }
            }

            for path in problem.agent_paths.iter() {
                println!("{:?}", path);
            }
            println!("Cost:{}", problem.compute_total_cost());

            problem.solve_reduced(&[
                (0,(0,problem.agent_paths[0].len()-1)),
                (1,(0,problem.agent_paths[1].len()-1))
            ]);
            for path in problem.agent_paths.iter() {
                println!("{:?}", path);
            }
            println!("Cost:{}", problem.compute_total_cost());
        }
    }

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
            let graph = super::super::graph::GraphWH::create_random(10,2,3);

            let mut final_nodes = HashMap::new();
            final_nodes.insert(Final, vec![]);
            final_nodes.insert(Target(0), vec![Target(4), Final]);
            let reduced_problem = ReducedProblem::new(
                &graph,
                vec![0, 0],
                vec![Agent(0), Agent(1)],
                vec![Final, Target(0), Target(1), Target(2)],
                final_nodes,
                None
            );

            let new_states = reduced_problem.expand_state(&reduced_problem.queue[0].0);

            for state in new_states {
                println!("{:?}", state.paths);
                for path in state.paths.iter() {
                    print!("{:?}, ", compute_complete_path_cost(&graph, path));
                }
                println!();
            }
        }

        #[test]
        fn queue() {
            use WHNode::*;
            let graph = super::super::graph::GraphWH::create_random(10,2,3);

            let mut final_nodes = HashMap::new();
            final_nodes.insert(Final, vec![]);
            final_nodes.insert(Target(0), vec![Target(4), Final]);
            let mut reduced_problem = ReducedProblem::new(
                &graph,
                vec![0, 0],
                vec![Agent(0), Agent(1)],
                vec![Final, Target(0), Target(1), Target(2)],
                final_nodes,
                None
            );

            loop {
                let is_finished = reduced_problem.iterate();
                if is_finished { break }
            }

            /*
            for state in reduced.queue {
                println!("{:?}", state.paths);
                for path in state.paths.iter() {
                    print!("{}, ", state.get_path_cost(path));
                }
                println!();
            }
            */
            println!("Best: {:?}", reduced_problem.best_solution.map(|(s,c)| (s, c)));
        }
    }
}
