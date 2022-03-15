// struct
struct Point {
    x: i32,
    y: i32,
}

// enum
enum Color {
    Red,
    Green,
    Blue,
}

// function
fn add(x: i32, y: i32) -> i32 {
   return x + y;
}

// macro
macro_rules! log {
	     ($x:expr) => { println!("{}", $x); }
}

// impl
// Rustではクラスはサポートされていないが、implによって構造体にメソッドを追加できる．(selfは自オブジェクトを示す．)
impl Point {
     fn distance(&self) -> i32 {
     	return self.x * self.x + self.y * self.y;
     }
}

// trait
// 構造体が実装すべきメソッドを定義。
trait Printable { fn print(&self); }

impl Printable for Point {
  fn print(&self) {
    println!("x: {}, y: {}", self.x, self.y)
  }
}

struct PointT<T> { x: T, y: T }
trait PrintableT { fn print_t(&self); }
impl<T> PrintableT for PointT<T> where T: std::fmt::Display {
  fn print_t(self: &PointT<T>) {
    println!("{}, {}", self.x, self.y);
  }
}

// <T>など、通常型を指定する箇所に、型(type)ではなくトレイと(trait)を指定する場合は、トレイトであることを明示するためにdynを指定する．
use std::boxed::Box;
struct Dog {}
struct Cat {}
trait Animal { fn cry(&self); }
impl Animal for Dog { fn cry (&self) { println!("Bow-wow"); } }
impl Animal for Cat { fn cry (&self) { println!("Miaow"); } }

fn get_animal(animal_type: &str) -> Box<dyn Animal> {
  if animal_type == "dog" {
    return Box::new(Dog{});
  } else {
    return Box::new(Cat{});
  }
}

// iterator
// Iteratorトレイとを実装するオブジェクトをイテレータと呼ぶ．イテレータはforで利用できる．Iteratorトレイとでは，next()により次のオブジェクトを返却し，最後に到達するとNoneを返す．
// selfはimplにおける自分自身の型を示す．
struct Counter {
  max: u32,
  count: u32,
}

impl Counter {
  fn new(max: u32) -> Counter {
    Counter { max: max, count: 0}
  }
}

impl Iterfor for Counter {
  type Item = u32;
  fn next(&mut self) -> Option<Self::Item> {
    self.count += 1;
    if self.count < self.max {
      Some(self.count);
    }
  }
}



fn main () {
    // struct
    let p = Point{x:300, y:300};
    println!("({}, {})", p.x, p.y);
    
    // enum
    let _red = Color::Red;
    let _green = Color::Green;
    let _blue = Color::Blue;
    
    // tuple
    // 型の異なる要素を含むことは可
    let tup = (10, "20", 30.0);
    println!("({}, {}, {})", tup.0, tup.1, tup.2);
    
    // array
    // 型の異なる要素を含むことは不可
    let arr = [10, 20, 30];
    println!("[{}, {}, {}]", arr[0], arr[1], arr[2]);
    
    for v in &arr {
        println!("{}", v);
    }
    
    // vector
    // 型の異なる要素を含むことは不可
    let mut vect = vec![10, 20, 30];
    vect.push(40);
    println!("[{}, {}, {}, {}]", vect[0], vect[1], vect[2], vect[3]);
    
    for v in &vect {
        println!("{}", v);
    }
    
    // hashmap
    use std::collections::HashMap;
    let mut map = HashMap::new();
    map.insert("x", 10);
    map.insert("y", 20);
    map.insert("z", 30);
    println!("{} {} {}", map["x"], map["y"], map["z"]);
    
    for (k, v) in &map {
        println!("{} {}", k, v);
    }
    
    // string
    let mut name: &str = "Yamada";
    println!("{}", name);
    name = "Tanaka";
    println!("{}", name);
    
    let mut name = String::from("Yamada");
    println!("{}", name);
    name = "Tanaka".to_string();
    println!("{}", name);
    name.push_str(" Taro");
    println!("{}", name);
    
    // heap(Box): オブジェクトのメモリを動的に確保
    let p2: Box<Point> = Box::new(Point{x:100, y:200});
    println!("{} {}", p2.x, p2.y);
    
    // slice
    let s = String::from("ABCDEFGH");
    let s1 = &s[0..3]; // 0~2番目までのスライス
    let s2 = &s[3..6]; // 3~5番目までのスライス
    println!("{} {}", s1, s2);

    // function
    let ans:i32 = add(30, 40);
    println!("{}", ans);

    // closure
    let square = | x: i32 | {
    	x * x
    };
    println!("{}", square(9));
    // moveはクロージャ内で参照するクロージャ外変数が存在する場合，その所有権をクロージャに移動させることを宣言する．
    let msg = String::from("Hello");	// クロージャ外変数
    let func = move || {		// msgの所有権がfunc()に移動
    	println!("{}", msg);
    };					// クロージャ終了時に所有者が不在となり解放
    func();

    // macro
    log!("ABC...");

    // if
    let n = 3;
    if n == 1 {
       println!("One");
    } else if n == 2 {
      println!("Two");
    } else {
      println!("Other");
    }

    let s = if n == 1 {"OK!"} else {"NG!"};
    println!("{}", s);

    // while
    let mut n = 0;
    while n < 10 {
    	  n += 1;
    }

    // for
    for i in 0..10 {
    	println!("{}", i);
    }

    // loop, break, continue
    n = 0;
    loop {
    	 n += 1;
	 if n == 10 {
	    println!("break");
	    break;
	 } else {
	   println!("continue");
	   continue;
	 }
    }

    // match
    let x = 2;
    match x {
    	  1 => println!("One"),
	  2 => println!("Two"),
	  3 => println!("Three"),
	  _ => println!("More"),
    }

    // impl
    println!("{}", p.distance());

    // trait
    p.print();

    let pt1: PointT<i32> = PointT{ x: 100, y: 100};
    let pt2: PointT<i64> = PointT{ x: 100, y: 200};
    pt1.print_t();
    pt2.print_t();

    get_animal("dog").cry();
    get_animal("cat").cry();
}