mod graph;
mod solver;

fn main() {
    let graph = graph::GraphWH::create_random(15,8,24);
    println!("{:?}", graph);

    let mut solver = solver::multi_dp::MASolver::new(&graph);

    solver.compute_initial_assignment();

    for path in solver.get_agent_paths() {
        println!("{:?}", path);
    }
}
