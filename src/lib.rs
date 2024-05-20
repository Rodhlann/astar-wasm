use std::slice::Iter;

use wasm_bindgen::prelude::*;

use std::fmt::Display;

/// Point represents an x,y coordinate on a grid'
#[wasm_bindgen]
#[derive(PartialEq, Clone, Debug, Copy)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[ x: {}, y: {} ]", &self.x, &self.y)
    }
}

impl Point {
    /// Add another point to the current point and return a new point
    pub fn add(&self, other_point: &Self) -> Point {
        Point {
            x: self.x + other_point.x,
            y: self.y + other_point.y,
        }
    }
}

///
/// A Node is the total data related to a single Point in the grid along the search path
///
/// parent_pos: `Option<point>` -> The position of this Node's parent Node, if one exists. This is useful for back tracking once the optimal path is found
///
/// pos: `Point` -> The position of the Node in the grid
///
/// g: `i32` -> The cost of movement from the start Point to this Node's pos
///
/// h: `i32` -> The estimated cost of movement from this Node's pos to the end Point
///
/// f: `i32` -> The combined cost of g and h, representing the total estimated cost to move from start to finish along this Node's path
///
/// NOTE: This implementation uses the Manhattan Distance Heuristic to calculate `h`
///
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub struct Node {
    pub parent: Option<Point>,
    pub pos: Point,
    pub g: i32,
    pub h: i32,
    pub f: i32,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.pos == other.pos
    }
}

const MOVE_COST: i32 = 1;
impl Node {
    fn new(pos: Point, parent_node: Option<Node>, end_pos: &Point) -> Self {
        let base_g = 0;
        let base_h = (pos.x - end_pos.x).abs() + (pos.y - end_pos.y).abs();

        let (parent, g, h, f) = if let Some(parent) = parent_node {
            let g = base_g + parent.g + MOVE_COST;
            let h = base_h;
            let f = g + h;
            (Some(parent.pos), g, h, f)
        } else {
            let g = base_g;
            let h = base_h;
            let f = g + h;
            (None, g, h, f)
        };

        Self {
            parent,
            pos,
            g,
            h,
            f,
        }
    }
}

enum Dirs {
    N,
    // NE,
    E,
    // SE,
    S,
    // SW,
    W,
    // NW,
}

impl Dirs {
    fn get(dir: &Dirs) -> Point {
        match dir {
            Dirs::N => Point { x: 0, y: -1 },
            // Dirs::NE => Point { x: 1, y: -1 },
            Dirs::E => Point { x: 1, y: 0 },
            // Dirs::SE => Point { x: 1, y: 1 },
            Dirs::S => Point { x: 0, y: 1 },
            // Dirs::SW => Point { x: -1, y: 1 },
            Dirs::W => Point { x: -1, y: 0 },
            // Dirs::NW => Point { x: -1, y: -1 },
        }
    }

    fn iter() -> Iter<'static, Dirs> {
        static D: [Dirs; 4] = [
            Dirs::N,
            // Dirs::NE,
            Dirs::E,
            // Dirs::SE,
            Dirs::S,
            // Dirs::SW,
            Dirs::W,
            // Dirs::NW,
        ];
        D.iter()
    }
}

#[wasm_bindgen]
#[allow(clippy::same_item_push)] // Required because width and height are not constant when building the vector matrix
pub fn a_star(
    width: usize,
    height: usize,
    start: Point,
    end: Point,
    walls: Vec<Point>,
) -> Node {
    let mut open: Vec<Node> = Vec::new();
    let mut closed: Vec<Node> = Vec::new();

    open.push(Node::new(start, None, &end));

    loop {
        if open.is_empty() {
            panic!("No nodes in open list, end not found!")
        }

        // Sort so Node with lowest f value is last in list
        open.sort_by(|a, b| b.f.cmp(&a.f));
        let best_node = open.pop().expect("No node found on open list");

        // Found end node, end search
        if best_node.pos.eq(&end) {
            break best_node;
        }

        // TODO: is there any way not to clone this instance?
        // I am concerned that it will impact performance, especially in the for loop below
        closed.push(best_node.clone());

        for dir in Dirs::iter() {
            let direction = Dirs::get(dir);
            let successor = Node::new(
                Point::add(&best_node.pos, &direction),
                Some(best_node.clone()),
                &end,
            );

            let next_pos = &successor.pos;
            if next_pos.x >= width as i32
                || next_pos.x < 0
                || next_pos.y >= height as i32
                || next_pos.y < 0
                || walls.contains(next_pos)
            {
                continue;
            }

            if !open.contains(&successor) && !closed.contains(&successor) {
                open.push(successor)
            }
        }
    }
}
