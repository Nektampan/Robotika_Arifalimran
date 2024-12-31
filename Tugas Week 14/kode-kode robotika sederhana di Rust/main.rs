use std::collections::VecDeque;

#[derive(Debug, Clone)]
struct Point {
    x: usize,
    y: usize,
}

fn is_valid_move(matrix: &Vec<Vec<i32>>, visited: &Vec<Vec<bool>>, x: i32, y: i32) -> bool {
    let rows = matrix.len() as i32;
    let cols = matrix[0].len() as i32;
    x >= 0 && y >= 0 && x < rows && y < cols && matrix[x as usize][y as usize] == 0 && !visited[x as usize][y as usize]
}

fn find_path(matrix: Vec<Vec<i32>>, start: Point, goal: Point) -> Option<Vec<Point>> {
    let directions = vec![
        (0, 1),  // right
        (1, 0),  // down
        (0, -1), // left
        (-1, 0), // up
    ];

    let mut visited = vec![vec![false; matrix[0].len()]; matrix.len()];
    let mut queue: VecDeque<(Point, Vec<Point>)> = VecDeque::new();

    queue.push_back((start.clone(), vec![start.clone()]));
    visited[start.x][start.y] = true;

    while let Some((current, path)) = queue.pop_front() {
        if current.x == goal.x && current.y == goal.y {
            return Some(path);
        }

        for (dx, dy) in &directions {
            let new_x = current.x as i32 + dx;
            let new_y = current.y as i32 + dy;

            if is_valid_move(&matrix, &visited, new_x, new_y) {
                let next_point = Point {
                    x: new_x as usize,
                    y: new_y as usize,
                };
                let mut new_path = path.clone();
                new_path.push(next_point.clone());
                queue.push_back((next_point, new_path));
                visited[new_x as usize][new_y as usize] = true;
            }
        }
    }

    None
}

fn main() {
    let matrix = vec![
        vec![0, 0, 0, 0, 0],
        vec![0, 1, 1, 1, 0],
        vec![0, 0, 0, 1, 0],
        vec![0, 1, 0, 0, 0],
        vec![0, 0, 0, 0, 0],
    ];

    let start = Point { x: 0, y: 0 };
    let goal = Point { x: 4, y: 4 };

    match find_path(matrix, start, goal) {
        Some(path) => {
            println!("Path found:");
            for point in path {
                println!("({}, {})", point.x, point.y);
            }
        }
        None => println!("No path found."),
    }
}
