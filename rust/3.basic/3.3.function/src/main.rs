fn main() {
    println!("Hello, world!");

    let y = another_function(1, 'h');
    println!("{y}");
}

fn another_function(x: i32, uint_label: char) -> i32 {
    println!("The value of x is: {x}{uint_label}");

    let y = {
        let x = 1;
        // ;無しでreturnと同義
        x + 1
    };

    println!("{y}");
    y
}
