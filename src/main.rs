mod graph;
mod solver;

fn main() {
    let graph = graph::GraphWH::create_random(80,80,240);
    // println!("{:?}", graph);

    let mut solver = solver::MASolver::new(&graph);

    solver.compute_initial_assignment();

    for path in solver.get_agent_paths() {
        println!("{:?}", path);
    }
}
