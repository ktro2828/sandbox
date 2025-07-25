use std::{sync::mpsc::Sender, time::Duration};

fn spawn_thread(tx: Sender<String>) {
    std::thread::spawn(move || {
        // send multiple values
        let values = vec![
            String::from("Hi"),
            String::from("from"),
            String::from("the"),
            format!("thread: {:?}", std::thread::current().id()),
        ];

        for value in values {
            tx.send(value).unwrap();
            std::thread::sleep(Duration::from_millis(1));
        }
    });
}

fn main() {
    let (tx, rx) = std::sync::mpsc::channel();

    spawn_thread(tx.clone());
    // in here, `tx` is moved and will be dropped when the thread finishes
    spawn_thread(tx);
    // if specify a sender by cloning `tx`, we need drop `tx` explicitly
    // spawn_thread(tx.clone());
    // drop(tx);

    for receive in rx {
        println!("Received: {}", receive);
    }
}
