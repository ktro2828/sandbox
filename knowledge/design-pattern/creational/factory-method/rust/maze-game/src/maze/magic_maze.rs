use crate::maze::{MazeGame, Room};

// Concrete Product
#[derive(Clone)]
pub struct MagicRoom {
    title: String,
}

impl MagicRoom {
    pub fn new(title: String) -> Self {
        Self { title }
    }
}

impl Room for MagicRoom {
    fn render(&self) {
        println!("Magic Room: {}", self.title);
    }
}

// Concrete Creator
pub struct MagicMaze {
    rooms: Vec<MagicRoom>,
}

impl MagicMaze {
    pub fn new() -> Self {
        Self {
            rooms: vec![
                MagicRoom::new("Infinite Room".into()),
                MagicRoom::new("Red Room".into()),
            ],
        }
    }
}

impl MazeGame for MagicMaze {
    type RoomImpl = MagicRoom;

    fn rooms(&self) -> Vec<Self::RoomImpl> {
        self.rooms.clone()
    }
}
