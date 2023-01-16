use std::convert::From;

#[derive(Debug)]
struct Number {
    value: i32,
}

// `From`トレイトは，ある方に対し，別の型からその型を作る方法を定義するようにする．
impl From<i32> for Number {
    fn from(item: i32) -> Self {
        return Number { value: item };
    }
}

// `Into`トレイトは，`From`トレイトの逆の働きをする．

fn main() {
    let int = 5;
    let num: Number = Number::from(30);
    let num_into: Number = int.into();
    println!("My number is {:?}", num);
    println!("My number is {:?}", num_into);
}
