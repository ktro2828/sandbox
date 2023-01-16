// 使用されていないコードによる警告を隠す
#![allow(dead_code)]

enum Status {
    Rich,
    Poor,
}

enum Work {
    Civilian,
    Soldier,
}

fn main() {
    // `use`することで絶対名でなくても使用可能になる．
    use crate::Status::{Poor, Rich};
    // `Work`の中の名前を全て`use`
    use crate::Work::*;

    let status = Poor; // = `Status::Poor`
    let work = Civilian; // = `Work::Civilian`

    match status {
        Rich => println!("The rich have lots of money!"),
        Poor => println!("The poor have no money..."),
    }

    match work {
        Civilian => println!("Civilians work!"),
        Soldier => println!("Soldiers fight!"),
    }
}
