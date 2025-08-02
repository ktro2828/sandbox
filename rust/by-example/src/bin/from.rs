#[derive(Debug)]
struct Number {
    value: i32,
}

impl From<i32> for Number {
    fn from(value: i32) -> Self {
        Number { value }
    }
}

fn main() {
    let num = Number::from(42);
    println!("{:?}", num.value);

    let num2: Number = num.into();
    println!("{:?}", num2);
}
