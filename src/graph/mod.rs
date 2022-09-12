use rand::Rng;


pub trait Graph {
    type Node;
    fn get_arc_cost(&self, x_0: &Self::Node, x_1: &Self::Node) -> Option<i32>;
}

#[derive(Clone, PartialEq, Eq, Debug, Hash)]
pub enum WHNode {
    Agent(usize),
    Target(usize),
    Final
}

#[derive(Debug)]
pub struct GraphWH {
    n_agents: usize,
    n_targets: usize,
    target_to_target_costs: Vec<i32>,
    agent_to_target_costs: Vec<i32>
}

impl Graph for GraphWH {
    type Node = WHNode;

    fn get_arc_cost(&self, x_0: &Self::Node, x_1: &Self::Node) -> Option<i32> {
        use WHNode::*;

        let target_dest_index = match x_1 {
            Target(x) => x,
            Agent(_) => return None,
            Final => return Some(0)
        };

        // println!("{:?} -> {:?}", x_0, x_1);

        Some(
            match x_0 {
                Agent(agent_origin_index) => {
                    self.agent_to_target_costs[
                        agent_origin_index + target_dest_index * self.n_agents
                    ]
                },
                Target(target_origin_index) => {
                    self.target_to_target_costs[
                        target_origin_index + target_dest_index * self.n_targets
                    ]
                },
                Final => return None
            }
        )
    }
}

impl GraphWH {
    pub fn get_n_agents(&self) -> usize {
        self.n_agents
    }

    pub fn get_n_targets(&self) -> usize {
        self.n_targets
    }

    pub fn create_random(size: i32, n_agents: usize, n_targets: usize) -> GraphWH {
        let mut rng = rand::thread_rng();

        let mut available_cells: Vec<(i32,i32)> = (0..size).map(
            move |x| (0..size).map(move |y| (x,y))
        ).flatten().collect();

        let mut agent_positions: Vec<(i32,i32)> = Vec::new();
        let mut target_positions: Vec<((i32,i32),(i32,i32))>  = Vec::new();

        for _ in 0..n_agents {
            let agent_cell = available_cells.swap_remove(rng.gen_range(0..available_cells.len()));
            agent_positions.push(agent_cell);
        }
        for _ in 0..n_targets {
            let target_1 = available_cells.swap_remove(rng.gen_range(0..available_cells.len()));
            let target_2 = available_cells.swap_remove(rng.gen_range(0..available_cells.len()));
            target_positions.push(
                (target_1, target_2)
            );
        }

        let mut target_to_target_costs = vec![0; n_targets*n_targets];
        let mut agent_to_target_costs = vec![0; n_targets*n_agents];

        for i_dest in 0..n_targets {
            let (dest_1, dest_2) = target_positions[i_dest];
            let constant_cost = manhattan_distance(dest_1, dest_2);

            for i_origin in 0..n_targets {
                let origin = target_positions[i_origin].1;

                target_to_target_costs[i_dest + i_origin * n_targets] =
                    manhattan_distance(origin, dest_1) + constant_cost;
            }

            for i_origin in 0..n_agents {
                let origin = agent_positions[i_origin];

                agent_to_target_costs[i_dest + i_origin * n_targets] =
                    manhattan_distance(origin, dest_1) + constant_cost;
            }

        }
        

        GraphWH {
            n_agents,
            n_targets,
            target_to_target_costs,
            agent_to_target_costs
        }
    }

    pub fn create_test_graph(
        n_agents: usize,
        n_targets: usize,
        agent_to_target_costs: Vec<Vec<i32>>,
        target_to_target_costs: Vec<Vec<i32>>) -> GraphWH {

        assert_eq!(agent_to_target_costs.len(), n_agents);
        for cost_vector in agent_to_target_costs.iter() {
            assert_eq!(cost_vector.len(), n_targets);
        }

        assert_eq!(target_to_target_costs.len(), n_targets);
        for cost_vector in target_to_target_costs.iter() {
            assert_eq!(cost_vector.len(), n_targets);
        }

        let mut agent_to_target_flat = vec![0; n_agents*n_targets];

        for agent in 0..n_agents {
            for target in 0..n_targets{
                agent_to_target_flat[agent + n_agents*target] = agent_to_target_costs[agent][target];
            }
        }

        let mut target_to_target_flat = vec![0; n_targets * n_targets];
        for origin_target in 0..n_targets {
            for dest_target in 0..n_targets {
                target_to_target_flat[origin_target + n_targets * dest_target] = 
                    target_to_target_costs[origin_target][dest_target]
            }
        }

        return GraphWH {
            n_agents,
            n_targets,
            agent_to_target_costs: agent_to_target_flat,
            target_to_target_costs: target_to_target_flat

        }
    }
}

fn manhattan_distance(x_0: (i32,i32), x_1: (i32,i32)) -> i32 {
    (x_0.0 - x_1.0).abs() + (x_1.0 - x_1.1).abs()
}


#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn check_test_graph() {

        let agent_to_target = vec![
            vec![1,3,5],
            vec![2,8,12]
        ];
        let target_to_target = vec![
            vec![4,5,6],
            vec![7,2,1],
            vec![2,4,9]
        ];
                
        let graph = GraphWH::create_test_graph(
            2,
            3,
            agent_to_target.clone(),
            target_to_target.clone()
        );

        println!("{:?}", graph.agent_to_target_costs);
        println!("{:?}", graph.target_to_target_costs);

        use WHNode::*;

        for agent in 0..2 {
            for target in 0..3 {
                assert_eq!(
                    Some(agent_to_target[agent][target]),
                    graph.get_arc_cost(&Agent(agent), &Target(target))
                );
            }
        }
        for origin_target in 0..3 {
            for dest_target in 0..3 {
                assert_eq!(
                    Some(target_to_target[origin_target][dest_target]),
                    graph.get_arc_cost(&Target(origin_target), &Target(dest_target)),
                    "Fail in origin: {}, dest {}", origin_target, dest_target
                );

            }
        }

    }
}

