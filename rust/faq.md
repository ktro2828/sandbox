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
