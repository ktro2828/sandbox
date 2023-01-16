// 構造体
// struct 構造体名 {
//     変数1: 型,
//     変数2: 型,
//     ...
// }

struct User {
    active: bool,
    username: String,
    email: String,
    sign_in_count: u64,
}

// tuple宣言も可能
struct Color(i32, i32, i32);

fn main() {
    let mut user = User {
        email: String::from("someone@example.com"),
        username: String::from("someone"),
        active: true,
        sign_in_count: 1,
    };
    // mutableなら可能
    user.email = String::from("another@example.com");

    // ""だけだと&strなので，String::from() or "".to_string()を使いStringにする．
    let _user1 = build_user("email@example.com".to_string(), "foo".to_string());

    let _black = Color(0, 0, 0);
}

fn build_user(email: String, username: String) -> User {
    // フィールド名と変数名が同じなら補完してくれる
    return User {
        email,
        username,
        active: true,
        sign_in_count: 1,
    };
}
