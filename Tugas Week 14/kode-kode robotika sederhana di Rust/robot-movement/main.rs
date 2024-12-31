use std::io;

struct Robot {
    x: i32,
    y: i32,
}
impl Robot {
    fn new() -> Self {
        Robot { x: 0, y: 0 }
    }

    fn move_robot(&mut self, direction: &str) {
        match direction {
            "up" => self.y += 1,
            "down" => self.y -= 1,
            "left" => self.x -= 1,
            "right" => self.x += 1,
            _ => println!("Invalid direction! Use: up, down, left, or right."),
        }
    }

    fn current_position(&self) {
        println!("Current position: ({}, {})", self.x, self.y);
    }
}

fn main() {
    let mut robot = Robot::new();
    let mut input = String::new();

    println!("Robot initialized at position (0, 0). Use 'up', 'down', 'left', or 'right' to move. Type 'exit' to quit.");

    loop {
        input.clear();
        println!("Enter direction:");
        io::stdin().read_line(&mut input).expect("Failed to read input");
        let direction = input.trim();

        if direction == "exit" {
            println!("Exiting program. Goodbye!");
            break;
        }

        robot.move_robot(direction);
        robot.current_position();
    }
}
