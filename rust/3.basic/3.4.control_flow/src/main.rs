fn main() {
    let number = 3;

    if number < 5 {
        println!("number is lower than 5");
    } else if number == 5 {
        println!("number is 5");
    } else {
        println!("number is upper than 5");
    }

    let condition = true;
    // 1行で書くこともできる．ただし，typeは同一である必要あり．
    let mut num = if condition { 5 } else { 40 };
    println!("{num}");

    // loop
    let mut cnt = 0;
    let result = loop {
        cnt += 1;

        if cnt == 10 {
            break cnt * 2;
        }
    };
    println!("The result is {result}");

    // loopがネストする場合にはloopにラベルを付与することでbreakできる
    let mut count = 0;
    'counting_up: loop {
        println!("count = {count}");
        let mut remaining = 10;

        loop {
            println!();
            if remaining == 9 {
                break;
            }
            if count == 2 {
                break 'counting_up;
            }
            remaining -= 1;
        }
        count += 1;
    }

    while num != 0 {
        println!("{num}!");

        num -= 1;
    }
    println!("LIFTOFF!");

    // 各要素を取り出す
    let a = [10, 20, 30, 40, 50];
    for element in a {
        println!("The value is: {element}");
    }

    // (start..end)で整数値のfor文
    // .rev()で降順
    for n in (1..10).rev() {
        println!("{n}");
    }
    println!("LIFTOFF!!");
}
