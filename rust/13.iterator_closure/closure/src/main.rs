fn main() {
    let sample_user_intensity: u32 = 10;
    let sample_random_number: u32 = 4;

    generate_workout(sample_user_intensity, sample_random_number);
}

// 処理が重い(=2dかかる)関数
#[allow(dead_code)]
fn simulated_expensive_calculation(intensity: u32) -> u32 {
    print!("calculating slowly...");
    std::thread::sleep(std::time::Duration::from_secs(2));
    intensity
}

// intensityとrandom_numberからトレーニングプランを出力する関数
fn generate_workout(intensity: u32, random_number: u32) {
    // // 方法1. 結果を変数に抽出 -> 使わないifブロックもあるので非効率的
    // let expensive_result = simulated_expensive_calculation(intensity);

    // // 方法2. 関数をクロージャで定義 -> 1つめのifブロックで２回クロージャ呼び出し
    // let expensive_closure = |num| {
    //     print!("calculating slowly...");
    //     std::thread::sleep(std::time::Duration::from_secs(2));
    //     num
    // };

    // 方法3.ジェネリクス引数とFnトレイトを使用してクロージャを保存する
    // Cacher.value()はクロージャが初めて呼ばれればそれを実行し，二回目以降はキャッシュされた値を返す
    // ※しかし，この実装ではクロージャの`num`は1回目の値しか機能しないので注意!!
    let mut expensive_result = Cacher::new(|num| {
        print!("calculating slowly...");
        std::thread::sleep(std::time::Duration::from_secs(2));
        num
    });

    if intensity < 25 {
        println!("Today, do {} pushups!", expensive_result.value(intensity));
        println!("Next, do {} situps!", expensive_result.value(intensity));
    } else {
        if random_number == 3 {
            println!("Take a break today! Remember to stay hydrated!");
        } else {
            println!(
                "Today, run for {} minutes!",
                expensive_result.value(intensity)
            );
        }
    }
}

// 方法3
struct Cacher<T>
where
    T: std::ops::Fn(u32) -> u32,
{
    calculation: T,
    value: Option<u32>,
}

impl<T> Cacher<T>
where
    T: std::ops::Fn(u32) -> u32,
{
    fn new(calculation: T) -> Cacher<T> {
        Cacher {
            calculation,
            value: None,
        }
    }

    fn value(&mut self, arg: u32) -> u32 {
        match self.value {
            Some(v) => v,
            None => {
                let v = (self.calculation)(arg);
                self.value = Some(v);
                v
            }
        }
    }
}

#[allow(dead_code)]
fn move_closure() {
    let x = vec![1, 2, 3];
    let equal_to_x = move |z| z == x;
    // // `x` はクロージャequal_to_xにmoveされたので以降では使えない
    // // なお，x: VecがCopyトレイトを実装していれば回避される
    // println!("can't use x here: {:?}", x);

    let y = vec![1, 2, 3];
    assert!(equal_to_x(y));
}
