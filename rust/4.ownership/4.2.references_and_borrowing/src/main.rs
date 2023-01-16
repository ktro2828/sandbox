fn main() {
    let s1 = String::from("hello");
    // 参照渡しの関数に対しては，&で渡す(=borrowing)
    let s = calculate_length(&s1);

    println!("{s1:?} is {s:?} length");

    let mut ms = String::from("hello");
    println!("before {ms:?}");
    // 可変参照渡しのときは，&mutで渡す
    change(&mut ms);
    println!("after {ms:?}");

    // 可変参照渡しはスコープ内で同時に1回のみ
    // 不可変参照渡しならOK
    let _r1 = &ms; // no problem
    let _r2 = &ms; // no problem
                   // _r1と_r2はこれ以上使われないので可変参照渡しでもOK
    let _r3 = &mut ms;

    println!("{_r3:?}");

    let ss = dangle(_r3);
    println!("{ss:?}");
}

// 不可変参照渡し=(変数: &型)
fn calculate_length(s: &String) -> usize {
    return s.len();
}

// 可変参照渡し=(変数: &mut 型)
fn change(s: &mut String) {
    s.push_str(", world");
}

// [Invalid]
// 参照返しは変数の寿命に注意
// fn dangle() -> &String {
//     // sは関数内で定義される変数なので関数終了後に消える = 参照先がなくなる
//     let s = String::from("hello");

//     return &s;
// }

fn dangle(s: &String) -> &String {
    return &s;
}
