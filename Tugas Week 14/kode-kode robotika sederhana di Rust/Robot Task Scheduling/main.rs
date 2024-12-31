use std::collections::BinaryHeap;
use std::cmp::Reverse;

#[derive(Debug, Eq, PartialEq)]
struct Task {
    priority: u32,
    description: String,
}

impl Ord for Task {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.priority.cmp(&self.priority)
    }
}

impl PartialOrd for Task {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

fn main() {
    let mut task_queue = BinaryHeap::new();

    // Add tasks to the queue
    task_queue.push(Task {
        priority: 1,
        description: String::from("Charge battery"),
    });
    task_queue.push(Task {
        priority: 3,
        description: String::from("Pick up package"),
    });
    task_queue.push(Task {
        priority: 2,
        description: String::from("Deliver package"),
    });

    println!("Starting task execution based on priority:");

    while let Some(task) = task_queue.pop() {
        println!("Executing task: {} with priority: {}", task.description, task.priority);
    }
}
