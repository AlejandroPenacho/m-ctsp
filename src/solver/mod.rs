#![allow(dead_code)]

use crate::graph;
use crate::graph::{WHNode,Graph};

use std::collections::{HashSet,HashMap};



#[derive(Clone)]
struct State<'a,'b> {
    graph: &'a graph::GraphWH,
    paths: Vec<Vec<WHNode>>,
    init_travelled_distances: Vec<i32>,
    remaining_nodes: Vec<WHNode>,
    final_nodes: &'b HashMap<WHNode, Vec<WHNode>>,
    available_final_nodes: u8
}

impl<'a,'b> State<'a,'b> {
    fn new(
        graph: &'a graph::GraphWH,
        agent_positions: Vec<WHNode>,
        init_travelled_distances: Vec<i32>,
        remaining_nodes: Vec<WHNode>,
        final_nodes: &'b HashMap<WHNode, Vec<WHNode>>) -> Self {

        let paths: Vec<Vec<WHNode>> = agent_positions.iter().map(|x| vec![x.clone()]).collect();

        let n_active_robots = agent_positions.len();

        let available_final_nodes: u8 = (n_active_robots - final_nodes.len() + 1).try_into().unwrap();

        State {
            graph,
            paths,
            init_travelled_distances,
            remaining_nodes,
            final_nodes,
            available_final_nodes
        }
    }

    fn expand(&self) -> Vec<Self> {
        let n_active_agents = self.paths.iter().filter(|&x|
            !self.final_nodes.contains_key(x.last().unwrap())
        ).count();

        let n_agents = self.paths.len();

        let cycler = Cycler::new(n_active_agents, &self.remaining_nodes, self.available_final_nodes);
        let mut new_states: Vec<State> = Vec::new();

        for combination in cycler {
            let mut new_state = self.clone();

            for (next_target_index, agent_index) in (0..n_agents).filter(|&i| {
                    !self.final_nodes.contains_key(self.paths[i].iter().last().unwrap())
                }).enumerate() {


                // let prev_agent_position = new_state.paths[agent_index].iter().last().unwrap();
                let new_agent_position: WHNode = combination[next_target_index].clone();

                if let WHNode::Target(_) = new_agent_position {
                    new_state.remaining_nodes.remove(
                        new_state.remaining_nodes.iter().position(|x| x == &new_agent_position).unwrap()
                    );
                }
                if let WHNode::Final = new_agent_position {
                    new_state.available_final_nodes -= 1;
                    if new_state.available_final_nodes == 0 { new_state.remaining_nodes.remove(0); }
                }

                new_state.paths[agent_index].push(new_agent_position.clone());

            }

            /*
            if new_state.paths.iter().filter(|&x| !new_state.final_nodes.contains_key(x.iter().last().unwrap())).count() == 0 {
                if new_state.remaining_nodes.len() > 1 {
                    continue
                }
                if new_state.remaining_nodes.len() == 1
                    && matches!(new_state.remaining_nodes[0], WHNode::Final) {
                    continue
                }
            }
            */
            new_states.push(new_state);
        }
        new_states
    }

    fn get_path_cost(&self, path: &[WHNode]) -> i32 {
        let mut last_node: &WHNode = &path[0];

        let mut cost = 0;
        for node in &path[1..] {
            /*
            println!("Adding {:?} -> {:?} ({})",
                     last_node,
                     node,
                     self.graph.get_arc_cost(last_node, node).unwrap()
                );
            */

            if node == &WHNode::Final { continue }
            cost += cost + self.graph.get_arc_cost(last_node, node).unwrap();
            last_node = node;
        }

        match self.final_nodes.get(path.iter().last().unwrap()) {
            None => {},
            Some(next_nodes) => cost *= 1 + next_nodes.len() as i32
        };

        // println!("Cost: {cost}");

        return cost
    }
}

struct ReducedProblem<'a,'b> {
    queue: Vec<State<'a,'b>>,
    best_solution: Option<(State<'a,'b>, i32)>
}

impl<'a,'b> ReducedProblem<'a,'b> {
    fn new(first_state: State<'a,'b>) -> Self {
        ReducedProblem {
            queue: vec![first_state],
            best_solution: None
        }
    }

    fn expand_first(&mut self) -> bool {
        if self.queue.len() == 0 { return true };
        let state = self.queue.remove(0);
        let new_states = state.expand();

        for new_state in new_states {
            let all_nodes_covered =  new_state.remaining_nodes.len() == 0 ||
                new_state.remaining_nodes.len() == 1 && new_state.remaining_nodes[0] == WHNode::Final && new_state.available_final_nodes == 0;

            let all_agents_done = new_state.paths
                .iter()
                .filter(|p| !new_state.final_nodes.contains_key(p.iter().last().unwrap())).count() == 0;

            if all_nodes_covered && all_agents_done {
                let cost = new_state.paths.iter().map(|p| new_state.get_path_cost(p)).sum();

                // println!("New solution:\n{:?}\n{:?}\n", new_state.paths, cost);

                if self.best_solution.as_ref().map_or(true, |(_, prev_best_cost)| &cost < prev_best_cost) {
                    self.best_solution = Some((new_state, cost));
                }
                continue
            }
            if all_agents_done && !all_nodes_covered { continue }

            self.queue.push(new_state);
        }

        false
    }

    fn optimize(mut self) -> Vec<Vec<WHNode>> {
        loop {
            let is_finished = self.expand_first();
            if is_finished { break }
        }
        self.best_solution.unwrap().0.paths
    }
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

    fn solve_reduced(&mut self, endpoints: &[(usize, (usize,usize))]) {
        for (index, (agent_index, (_, i_f))) in endpoints.iter().enumerate() {
            for (agent_index_2, _) in endpoints.iter().skip(index+1) {
                if agent_index == agent_index_2 { panic!() }
                if self.agent_paths[*agent_index].len() <= *i_f { panic!() }
            }
        }


        let mut remaining_nodes: Vec<WHNode> = Vec::new();
        let mut initial_nodes: Vec<WHNode> = Vec::new();
        let mut final_nodes: HashMap<WHNode, Vec<WHNode>> = HashMap::new();
        let mut initial_travel_distances: Vec<i32> = Vec::new();

        let mut final_node_available = false;

        for (agent_index, (i_0, i_f)) in endpoints.iter() {
            let final_chain: Vec<WHNode> = self.agent_paths[*agent_index].split_off(i_f+1);
            let mut splitted_nodes: Vec<WHNode> = self.agent_paths[*agent_index].split_off(i_0+1);

            final_nodes.insert(splitted_nodes.iter().last().unwrap().clone(), final_chain);

            initial_nodes.push(self.agent_paths[*agent_index].pop().unwrap());

            if splitted_nodes.iter().last().unwrap() == &WHNode::Final {
                final_node_available = true;
                splitted_nodes.pop();
            }

            remaining_nodes.append(&mut splitted_nodes);

            let mut initial_distance = 0;
            if !self.agent_paths[*agent_index].len() == 0 {
                for i in 0..(self.agent_paths[*agent_index].len()-1) {
                    initial_distance += self.graph.get_arc_cost(
                        &self.agent_paths[*agent_index][i],
                        &self.agent_paths[*agent_index][i+1]
                    ).unwrap();
                }
                initial_distance += self.graph.get_arc_cost(
                        self.agent_paths[*agent_index].iter().last().unwrap(),
                        &initial_nodes[*agent_index]
                    ).unwrap();
            }
            initial_travel_distances.push(initial_distance);
        }

        if final_node_available {
            remaining_nodes.push(WHNode::Final);
            remaining_nodes = remaining_nodes.into_iter().rev().collect();
        }


        println!("Remaining nodes: ");
        println!("{:?}\n", remaining_nodes);
        println!("Initial nodes: ");
        println!("{:?}\n", initial_nodes);
        println!("Final nodes: ");
        for (key, nodes) in final_nodes.iter() {
            println!("{:?} -> {:?}", key, nodes);
        }

        let first_state = State::new(
            self.graph,
            initial_nodes,
            initial_travel_distances,
            remaining_nodes,
            &final_nodes
        );

        let reduced_problem = ReducedProblem::new(first_state);
        let new_paths = reduced_problem.optimize();

        for (index, mut new_path) in new_paths.into_iter().enumerate() {
            let agent_index = endpoints[index].0;
            let last_node = new_path.iter().last().unwrap();
            new_path.append(final_nodes.get_mut(last_node).unwrap());
            self.agent_paths[agent_index].append(&mut new_path);
        }
    }

    fn compute_total_cost(&self) -> i32 {
        self.agent_paths.iter().map(|path| {
            let mut cost = 0;
            for i in 0..(path.len()-2) {
                cost += cost + self.graph.get_arc_cost(&path[i], &path[i+1]).unwrap();
            }
            cost
        }).sum()
    }
}




#[cfg(test)]
mod test {
    use super::*;
    use crate::graph::GraphWH;


    #[test]
    fn the_cut() {
        let graph = GraphWH::create_random(20, 4, 20);
        let mut problem = MASolver::new(&graph);
        problem.compute_initial_assignment();
        for path in problem.agent_paths.iter() {
            println!("{:?}", path);
        }

        println!("-----------------------------------");

        problem.solve_reduced(&[(0, (2, 4)), (1,(0,1))]);
    }

    #[test]
    fn much_optimal() {
        let graph = GraphWH::create_random(20, 4, 12);
        let mut problem = MASolver::new(&graph);
        problem.compute_initial_assignment();

        for path in problem.agent_paths.iter() {
            println!("{:?}", path);
        }
        println!("Cost:{}", problem.compute_total_cost());

        for (i,j) in [(0,1),(1,2),(2,3),(3,1)] {
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

    #[test]
    fn please() {
        let graph = GraphWH::create_random(20,2,3);
        let mut problem = MASolver::new(&graph);
        problem.compute_initial_assignment();

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
            let state = State::new(
                &graph,
                vec![Agent(0), Agent(1)],
                vec![0, 0],
                vec![Final, Target(0), Target(1), Target(2)],
                &final_nodes
            );

            let new_states = state.expand();

            for state in new_states {
                println!("{:?}", state.paths);
                for path in state.paths.iter() {
                    print!("{}, ", state.get_path_cost(path));
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
            let state = State::new(
                &graph,
                vec![Agent(0), Agent(1)],
                vec![0, 0],
                vec![Final, Target(0), Target(1), Target(2)],
                &final_nodes
            );

            let mut reduced = ReducedProblem::new(state);

            loop {
                let is_finished = reduced.expand_first();
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
            println!("Best: {:?}", reduced.best_solution.map(|(s,c)| (s.paths, c)));
        }
    }
}
