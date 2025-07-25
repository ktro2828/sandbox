# FAQ

## `clone()` vs `to_owned()`

- `clone()`

  - 基本的に対象の型の複製を作り、参照型`&T`に対しては型の複製`T`を生成
  - スライスのようなサイズが不明な型に対しても`[T]` -> `[T]`を生成
  - ただし、参照型の中身が参照の場合は、元と同じ参照型を生成
    - 参照型スライスの場合 = 参照型:`&[&T]` -> `&[&T]`
    - 参照型でサイズがわかる場合 = 複製: `&[&T; N]` -> `[&T; N]`

- `to_owned()`
  - スライスのようなサイズが不明な型もしくは、中身も参照型の場合はベクタを生成する`&[T]` -> `Vec<T>` or `&[&T]` -> `Vec[&T]`
    - `Vec<T>`と`[T]`の違いは`capacity`メソッドがあるかないか
    - `to_owned()`によって複製されたデータは、データ自身が必ず所有権を持つことになる
  - 他は`clone()`と同じ

## `&str` vs `String`

- `&str` -> str配列のスライス = `&[str]`
- `String` -> strのベクタ = `Vec<str>`

|          | `&str`                     | `String`                   |
| -------- | -------------------------- | -------------------------- |
| 正式名   | string slice               | owned string               |
| 所有権   | ❌ (借用参照)              | ✅                         |
| Size     | 静的またはスライス長分     | ヒープ確保で動的           |
| 書き換え | ❌                         | ✅                         |
| 主な用途 | 　定数/借用/軽量な受け渡し | 動的生成/編集/所有する場合 |
| リテラル | `"foo"` -> `&'static str`  | `"foo".to_string()`等      |

### 特徴・使い分け

#### `&str`

- **「誰かが持っている文字列データの一部orすべて」を参照する借用型**
- サイズは「データの長さ」だけ (ヒープ確保しないため、コンパイル時に決定)
- 文字列リテラルは全て`&'static str`
- コピーやcloneは安い(ただの参照)
- **編集・所有権移動できない**(そもそも所有権なし)
- 主な用途:
  - 関数の引数などで「借用」だけしたいとき
  - 文字列リテラル

#### `String`

- **自分自身がヒープ上に文字データを確保している「所有型」**
- サイズ・データを動的に変更可能(push,clear,etc.)
- 関数感で所有権の受け渡しができる
- 必要なら`&str`への変換(`&my_string`, `my_string.as_str()`)は安価にできる

### Rust vs C++対応表

| Rust           | C++                  | 主な用途・特徴                             |
| -------------- | -------------------- | ------------------------------------------ |
| `&'static str` | `const char *`       | 文字列リテラル、グローバルな不変文字列     |
| `&str`         | `std::string_view`   | 借用・軽量参照型、編集不可、所有しない     |
| `String`       | `std::string`        | 所有型の可変文字列、編集可、ヒープ上の管理 |
|                | `const std::string&` | C++で関数に文字列参照を渡すとき            |

### 似たような関係性 (`Path` vs `PathBuf`)

|          | `Path`                 | `PathBuf`                        |
| -------- | ---------------------- | -------------------------------- |
| 種類     | 借用型(スライス)       | 所有型(可変バッファ)             |
| 所有権   | なし                   | あり                             |
| 編集可否 | 不可(変更操作できない) | 可(push,pop,set_file_name等)     |
| 主な用途 | 一時的な参照・関数引数 | パスを生成・編集・保持したい場合 |
| 作り方   | `Path::new("foo/bar")` | `PathBuf::from("foo/bar")`       |
| 変換     | `pathbuf.as_path()`    | `Path::to_path_buf()`            |
| 役割     | `&str`的(借用)         | `String`的(所有・編集可能)       |

## アクセシビリティ

- `pub(in path)` : 指定した`path`内からアクセス可能
- `pub(crate)` : 同一の`crate`内からアクセス可能
- `pub(super)` : 親モジュールからアクセス可能
- `pub(self)` : 同一モジュール内からアクセス可能

## `feature`

### `feature`の定義

`Cargo.toml`に`features`を定義する。`default`を定義することで、featureを指定せずに`cargo add`した場合にデフォルトで追加されるfeatureを指定できる。
定義したfeatureに特定のクレートが依存する場合は、`[]`に追記する。
このとき、feature名は依存クレートと異なる名前で定義する必要がある。
`dependencies`に追加する際に、`optional = true`にするとdefaultではビルドされない。

```toml
[features]
default = ["foo"]
foo = []
bar = ["rand"] # randに依存

[dependencies]
rand = { version = "0.7", optional = true }
```

ソースコード内では、アトリビュートもしくは`cfg!`マクロで属するfeatureを指定する

```rust
#[cfg(feature="foo")]
pub fn foo_print() {
    print!("A member of foo feature!!")
}

#[cfg(not(feature="foo"))]
pub fn not_foo_print() {
    println!("Not a member of foo feature!!")
}
```

## パターンマッチング

### `if let 式`

`if let 式`を使うことで式が正のとき処理が実行される。

```rust
let x = Some(1)

if let Some(1) = x {
    // 実行される
}

if let Some(2) = x {
    // 実行されない
}
```

タプルでも使える

```rust
if let (Some(1), 2) = (x, 2) {
    // 実行される
}
```

`@`を使うことである範囲において、式が正のときのみ処理が実行される

```rust
let n = Some(3);
if let Some(i @ 0..=10) = n {
    println!("{}", i);
}

let m = 3;
if let i @ 0..=10 = m {
    println!("{}", i);
}
```

## `isize` vs `usize`

PCのCPUアーキテクチャにあったビット分のサイズを取る整数型。

- 32bit

  - `isize` : `i32`
  - `usize`: `u32`

- 64bit
- `isize` : `i64`
- `usize` : `u64`

## 関連型(Associated Type)

関連型はトレイとの拡張機能で、ジェネリックな構造体に対するトレイトを使うときに書き方がシンプルになり可読性が上がる。

以下のようなジェネリックなコンテナを定義する場合、ジェネリクス引数を何度も書く必要があり、見づらい + 面倒。

```rust
struct Container(i32, i32, i32);

trait Contains<A, B, C> {
    fn contains(&self, _: &A, _: &B, _: &C_) -> bool;
    // 先頭の値を取得
    fn first(&self) -> i32;
    // 最後の値を取得
    fn last(&self) -> i32;
}

// 関連型を使わないと<...>が増えていく <= 見づらい
impl Contains<i32, i32, i32> for Point {
    fn contains(&self, x: &i32, y: &i32, z: &i32) -> bool {
        (&self.0 == x) && (&self.1 == y) && (&self.2 == z)
    }

    fn first(&self) -> i32　{ self.0 }

    fn last(&self) -> i32 { self.2 }

}

// 見づらい + 面倒
fn difference<A, B, C, D>(contaier: &D) -> i32
    where
        D: Contains<A, B, C>
    {
        contaier.last() - container.first()
    }
```

関連型を定義することで以下のようにかける。

```rust
trait Contains {
    // 出力型を書く
    type A;
    type B;
    type C;

    fn contains(&self, _: &Self::A, _: &Self::B, _: _&Self::C) -> bool;

    // ...以下はさっきと同じ
}

impl Contains for Container {
    type A = i32;
    type B = i32;
    type C = i32;

    // `&i32`の代わりに、`&Self::A`や`&self::B`も使える
    fn contains(&self, x: &i32, y: &i32, z: &i32) -> bool {
        (&self.0 == x) && (&self.1 == y) && (&self.2 == z)
    }
}

// スッキリした
fn difference<T: Contains>(container: &T) -> i32 {
    ...
}

// where句でもOK
// fn difference<T>(container: &T) -> i32
// where
//     T: Contains
// {
//
//     ...
// }
```

## 幽霊型(Phantom Type)

幽霊型は、コンパイル時には静的に型チェックがされるが、実行時には中身が無いがあるように振る舞う。

```rust
use std::marker::PhantomData;

struct PhntomStruct<X, A> {
    value: A,
    _phantom: PhantomData<X>,
}
```

## Unsafeな操作

Rustには以下のような安全でない操作がある。

- 生ポインタ
  - 参照は借用チェッカーによって安全であることが保証されているが、生ポインタのでリファレンスは`unsafe`ブロック内でのみしか実行できない。

```rust
fn main() {
    let raw_p: *const u32 = &10;

    unsafe {
        assert!(*raw_p == 10);
    }
}
```

- 安全でない関数やメソッドの呼び出し(FFI(Foreign Function Interface)経由の関数呼び出しを含む)
- 静的なミュータブル変数へのアクセスや操作
- 安全でないトレイトの実装
