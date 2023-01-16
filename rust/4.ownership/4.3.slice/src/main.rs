fn main() {
    let s = String::from("hello world");
    let hello_size = first_word(&s);

    println!("{hello_size:?}");

    let hello = &s[0..5]; // [start..end]
    let world = &s[6..11];
    let hello2 = &s[0..=5]; // [start..=length]

    println!("{hello:?} {world:?}, {hello2:?}");
}

fn first_word(s: &String) -> usize {
    let bytes = s.as_bytes();

    // byteに一旦直して.iter().enumerate() -> (usize, &item)
    for (i, &item) in bytes.iter().enumerate() {
        if item == b' ' {
            return i;
        }
    }

    return s.len();
}
