use std::{
    sync::{Arc, Mutex},
    thread,
};

fn spawn_thread(counter: Arc<Mutex<i32>>) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        for _ in 0..10 {
            let mut num = counter.lock().unwrap();
            *num += 1;
            println!("Sorry, I'm busy in thread {:?}", thread::current().id());
        }
    })
}

fn main() {
    let counter = Arc::new(Mutex::new(0));
    let mut handles = vec![];

    handles.push(spawn_thread(Arc::clone(&counter)));
    handles.push(spawn_thread(Arc::clone(&counter)));

    for handle in handles {
        handle.join().unwrap();
    }

    println!("counter = {:?}", *counter.lock().unwrap());
}
