fn main() {
    // if you want use float, deleare float explicitly
    let x = 2.0; // f64
    let y: f32 = 3.0; // f32
    println!("x={x}, y={y}");

    // numeric operation
    let sum = 5 + 10;
    let floored = 2 / 3; // Results in 0
    println!("sum={sum}, floored={floored}");

    // boolean
    let t = true;
    let f: bool = false;
    println!("t={t}, f={f}");

    // character
    let c = 'z';
    let z: char = 'Z'; // with explicit type annotation: "" = String, '' = char
    println!("c={c}, z={z}");

    // tuple
    let tup = (3, 2, 1.0);
    let (t1, t2, t3) = tup;
    let three = tup.0;
    println!("(t1, t2, t3)=({t1}, {t2}, {t3})");
    println!("tup[0]={three}");

    // array
    let a = [1, 2, 3, 4];
    println!("{}", a[0]);
}
