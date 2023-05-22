# FAQ

## `&str` vs `String`
- `&str` -> str配列のスライス = `&[str]`
- `String` -> strのベクタ = `Vec<str>`

### 似たような関係性
- `Path` vs `PathBuf`

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

## インライン展開

`#[inline]`アトリビュートをつけることで関数をインライン展開できる。
- `#[inline(always)]` : crate境界内で、その関数を（できるだけ）インライン展開
- `#[inline(never)]` : 関数を（できるだけ）インライン展開しない

```rust
#[inline(always)]
pub fn add(a: i32, b: i32) -> i32 {
    a + b
}
```

## Enumの縮小化

Enum要素のうち他の要素と比べて大きなサイズの型をBox化することで、ヒープ割り当てが必要になる代わりにEnum自体のサイズが削減できる。

```rust
type LargeType = [u8; 100];

enum A {
    X,
    Y(i32),
    Z(i32, LargeType),
}

enum B {
    X,
    Y(i32),
    Z(Box<(i32, LargeType)>),
}
```

## `SmallVec<[T;N]>`
`Vec<T>`の代わりに`SmallVec<[T;N]>`を使うと、N個の要素はスタックに保存され、N+1個以降の要素はヒープに保持されることによって、アロケーションコストを削減できる。
ただし、`SmallVec`はアクセス時に特定の要素がアロケーションされているかチェックする必要があるので、操作コストがわずかに増える。
