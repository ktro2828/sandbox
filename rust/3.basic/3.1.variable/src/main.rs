fn main() {
    let x = 5; // x is immutable.
               // x = 4 => Error
    println!("The value of x is: {x}");

    let mut y = 5;
    println!("The value of y is: {y}");
    y = 4; // ok
    println!("The value of y is overlapped by: {y}");

    let spaces = "   ";
    let num_space = spaces.len();
    println!("The number of spaces is: {num_space}");
}
