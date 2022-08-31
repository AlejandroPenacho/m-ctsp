use rand::Rng;


pub trait Graph {
    type Node;
    fn get_arc_cost(&self, x_0: Self::Node, x_1: Self::Node) -> Option<i32>;
}


pub enum WHNode {
    Agent(usize),
    Target(usize)
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

    fn get_arc_cost(&self, x_0: Self::Node, x_1: Self::Node) -> Option<i32> {
        use WHNode::*;

        let target_dest_index = match x_1 {
            Target(x) => x,
            Agent(_) => return None
        };

        Some(
            match x_0 {
                Agent(agent_origin_index) => {
                    self.agent_to_target_costs[
                        agent_origin_index + target_dest_index * self.n_agents
                    ]
                },
                Target(target_origin_index) => {
                    self.target_to_target_costs[
                        target_origin_index + target_dest_index * self.n_agents
                    ]
                }
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
}


fn manhattan_distance(x_0: (i32,i32), x_1: (i32,i32)) -> i32 {
    (x_0.0 - x_1.0).abs() + (x_1.0 - x_1.1).abs()
}
