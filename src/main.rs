mod graph;
mod solver;

fn main() {
    let graph = graph::GraphWH::create_random(6,2,4);
    println!("{:?}", graph);
}
