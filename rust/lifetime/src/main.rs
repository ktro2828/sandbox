// ライフタイム注釈
// fn 関数名<'注釈文字>(変数: &'注釈文字 型, ...) -> &'注釈文字 型{...}
fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
    if x.len() > y.len() {
        x
    } else {
        y
    }
}

fn main() {
    let x = "foo".to_string();
    let y = "barbar".to_string();
    {
        let z = longest(&x, &y);
        println!("{}", z);
    }
}
