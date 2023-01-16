#[derive(Debug)]
struct Rectangle {
    width: u32,
    height: u32,
}

fn main() {
    let width1 = 30;
    let height1 = 50;

    println!(
        "The area of rectangle is {} square pixels.",
        area(width1, height1)
    );

    let rect1 = (30, 50);
    println!(
        "The area of rectangle is {} square pixels.",
        area_from_tuple(rect1)
    );

    let rect2 = Rectangle {
        width: 30,
        height: 50,
    };
    println!(
        "The area of rectangle is {} square pixels.",
        area_from_rect(&rect2)
    );

    // #[derive(Debug)]をつければprintln!()できる
    println!("rect2 is {:?}", rect2);
    // dbg!(&struct)でも可
    dbg!(&rect2);
}

fn area(width: u32, height: u32) -> u32 {
    return width * height;
}

// tuple使用ver
fn area_from_tuple(dimensions: (u32, u32)) -> u32 {
    return dimensions.0 * dimensions.1;
}

// struct使用ver
fn area_from_rect(rect: &Rectangle) -> u32 {
    return rect.width * rect.height;
}
