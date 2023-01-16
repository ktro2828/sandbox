fn main() {
    let mut s = String::from("hello"); // NOTE: s is mutable = heap data
                                       // Stringは3つのパーツで構成される．ptr, len, capacity．<- これらはstack領域に保存される．
                                       // ptrが示すデータ(=hello)はheap領域に保存される．
    s.push_str(", world");
    println!("{s}");
    let s2 = s.clone(); // [Invalid] let s2 = s; (ownership was moved for heap)
    println!("{s}");
    println!("{s2}");

    let s3 = String::from("hello");
    let s5 = s3; // ownershipがs3からs5に移動．(ptrはどちらも同じheap領域のアドレスを示す)
    println!("{s5}");

    let x = 5;
    let y = x;
    println!("{}", x);
    println!("{}", y);

    let s6 = takes_and_gives_back(s5);

    // warning回避のため，使わない変数は_をつける(_hoge)
    let (_s7, _len) = calculate_length(s6);
}

fn takes_and_gives_back(a_string: String) -> String {
    return a_string;
}

fn calculate_length(s: String) -> (String, usize) {
    let length = s.len();

    return (s, length);
}
