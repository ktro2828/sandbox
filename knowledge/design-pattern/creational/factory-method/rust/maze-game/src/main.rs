pub mod maze;

use crate::maze::magic_maze::MagicMaze;
use crate::maze::ordinary_maze::OrdinaryMaze;

/**
 * Client code
 */

fn main() {
    // Option 1: The game starts with an ordinary maze.
    let ordinary_maze = OrdinaryMaze::new();
    maze::run(ordinary_maze);

    // Option 2: The game starts with a magic maze.
    let magic_maze = MagicMaze::new();
    maze::run(magic_maze);
}
