use std::{thread, time::Duration};

fn main() {
    let handle = thread::spawn(|| {
        for i in 1..10 {
            println!("Hi, number {} from the spawned thread!", i);
            thread::sleep(Duration::from_millis(1));
        }
    });

    // // Join the spawned thread to wait for it to finish.
    // // Then messages from the main thread will be printed after the spawned thread finishes.
    // handle.join().unwrap();

    for i in 1..5 {
        println!("Hi, number {} from the main thread!", i);
        thread::sleep(Duration::from_millis(1));
    }

    // Join the spawned thread to wait for it to finish.
    // Then both messages from the main and spawned thread will be printed at the same time.
    handle.join().unwrap();
}
