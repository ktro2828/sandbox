pub mod magic_maze;
pub mod ordinary_maze;

/**
 * Interface of Product & Creator
 */

/**
 * Product interface.
 *
 * Maze room that is going to be instantiated with a factory method.
 */
pub trait Room {
    fn render(&self);
}

/**
 * Creator interface.
 *
 * Maze game has a factory method producing different rooms.
 */
pub trait MazeGame {
    type RoomImpl: Room;

    // A factory method.
    fn rooms(&self) -> Vec<Self::RoomImpl>;

    fn play(&self) {
        for room in self.rooms() {
            room.render();
        }
    }
}

/**
 * The client code initializes resources and does other preparations
 * then it uses a factory to construct and run the game.
 */
pub fn run(game: impl MazeGame) {
    println!("Loading resources...");
    println!("Starting the game...");

    game.play();
}
