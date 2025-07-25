use std::thread;

fn main() {
    let v = vec![1, 2, 3, 4];

    // COMPILE ERROR: cannot move out of `v` because it is unavailable to ensure that `v` still survives until the thread is spawned
    // let handle = thread::spawn(|| {
    //     println!("Here's a vector: {:?}", v);
    // });

    // use move keyword to move ownership of v into the closure
    let handle = thread::spawn(move || {
        println!("Here's a vector: {:?}", v);
    });

    // `v` has been moved into the closure, so it is no longer available to access it in the main thread
    // println!("Here's a vector from main thread: {:?}", v);

    handle.join().unwrap();
}
