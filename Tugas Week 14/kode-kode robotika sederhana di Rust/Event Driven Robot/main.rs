use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Debug, Clone, PartialEq)]
enum Event {
    ObstacleDetected,
    TargetChanged((i32, i32)),
    NoEvent,
}

#[derive(Debug)]
struct Robot {
    position: (i32, i32),
    target: (i32, i32),
}

impl Robot {
    fn new() -> Self {
        Robot {
            position: (0, 0),
            target: (0, 0),
        }
    }

    fn handle_event(&mut self, event: Event) {
        match event {
            Event::ObstacleDetected => {
                println!("Obstacle detected! Stopping movement.");
            }
            Event::TargetChanged(new_target) => {
                self.target = new_target;
                println!("Target changed to {:?}. Moving towards target...", self.target);
                self.move_towards_target();
            }
            Event::NoEvent => {
                println!("No event detected. Standing by.");
            }
        }
    }

    fn move_towards_target(&mut self) {
        while self.position != self.target {
            if self.position.0 < self.target.0 {
                self.position.0 += 1;
            } else if self.position.0 > self.target.0 {
                self.position.0 -= 1;
            }

            if self.position.1 < self.target.1 {
                self.position.1 += 1;
            } else if self.position.1 > self.target.1 {
                self.position.1 -= 1;
            }

            println!("Moving to position {:?}", self.position);
            thread::sleep(Duration::from_millis(500));
        }

        println!("Reached target at {:?}!", self.position);
    }
}

fn main() {
    let robot = Arc::new(Mutex::new(Robot::new()));
    let robot_clone = Arc::clone(&robot);

    thread::spawn(move || {
        let events = vec![
            Event::NoEvent,
            Event::TargetChanged((5, 5)),
            Event::ObstacleDetected,
            Event::TargetChanged((2, 3)),
            Event::NoEvent,
        ];

        for event in events {
            thread::sleep(Duration::from_secs(2));
            let mut robot = robot_clone.lock().unwrap();
            robot.handle_event(event.clone());
        }
    })
    .join()
    .unwrap();
}
