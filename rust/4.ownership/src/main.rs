fn main() {
    let mut s = String::from("hello"); // NOTE: s is mutable = heap data
    s.push_str(", world");
    println!("{s}");
    let s2 = s.clone(); // [Invalid] let s2 = s; (ownership was moved for heap)
    println!("{s}");
    println!("{s2}");

    let s3 = String::from("hello");
    let s5 = s3;
    println!("{s5}");

    let x = 5;
    let y = x;
    println!("{}", x);
    println!("{}", y);
}
