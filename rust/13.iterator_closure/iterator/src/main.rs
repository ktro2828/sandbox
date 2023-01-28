fn main() {
    let v1 = vec![1, 2, 3];
    // // iteratorでfor文で各要素にアクセスできる
    // let v1_iter = v1.iter();
    // for val in v1_iter {
    //     println!("Got: {}", val);
    // }

    // // Iteratorトレイトを実装している
    // // nextで得られる値は，要素への不変参照
    // let mut v1_iter = v1.iter();
    // assert_eq!(v1_iter.next(), Some(&1));

    // let v1_iter = v1.iter();
    // let total: i32 = v1_iter.sum();
    // assert_eq!(total, 6);
}
