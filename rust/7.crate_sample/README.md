# Rustにおけるパッケージ，クレート，モジュール

- **パッケージ**: クレートをビルドし，テストし，共有できるCargoの機能
- **クレート**: ライブラリか実行可能ファイルを生成する，木構造をしたモジュール郡
- **モジュール**と**use**: これを使うことで，パスの構成，スコープ，公開するか否かを決定できる
- **パス**: 要素(例えば構造体や関数やモジュール)に名前をつける方法

## パッケージとクレート

パッケージは，1個以上のバイナリクレートと，1個以下のライブラリクレートを持っていなければならない．
Cargoは，`src/main.rs`が含まれていれば，これを**バイナリクレートのクレートルート**と判断する．
また，`src/lib.rs`が含まれていれば，これを**ライブラリクレートのクレートルート**と判断する．

例えば，`cargo new PACKAGE`で新しいパッケージを作る．
```shell
$ cargo new my-project
    Created binary (application) `my-project` package
```
初期の構成は以下のようになる．
```shell
$ tree my-project/
my-project/
├── Cargo.toml
└── src
    └── main.rs
```
今，このパッケージには`src/main.rs`のみなので，このパッケージは`my-project`とという名前のバイナリクレートのみを持っていることとなる．
ここに，`src/lib.rs`もあれば，それぞれ`my-project`という名前のバイナリクレートとライブラリクレートである．
生成したファイルを`src/bin`以下に置くことで，パッケージは複数のバイナリクレートを持つことになり，それぞれのファイルが別のバイナリクレートとなる．

例えば，`src/bin/foo.rs`を指定すると，このときパッケージは`my-project`と`foo`というバイナリクレートを持つことになる．
```shell
$ tree my-project/
my-project/
├── Cargo.toml
└── src/
    ├── bin
    │   └── foo.rs
    └── main.rs
```
これを実行するには，`cargo run --bin <CRATE_NAME>`とする必要がある．

```shell
$ cargo run --bin foo
   Compiling my-project v0.1.0 (/home/ktro2828/my-project)
    Finished dev [unoptimized + debuginfo] target(s) in 0.18s
     Running `target/debug/foo`
This is foo!
```

## モジュールとスコープ
ライブラリを作成する際には，`cargo new --lib LIBRARY_NAME`を実行する．
すると，デフォルトで`src/lib.rs`が生成される．
```shell
$ cargo new --lib my-library
     Created library `my-library` package

$ tree my-library/
my-library/
├── Cargo.toml
└── src
    └── lib.rs
```
`src`以下にディレクトリを作ることで，複数のモジュールを定義することができる．
```shell
$ tree my-library/
my-library/
├── Cargo.toml
└── src
    ├── lib.rs
    └── modules
        ├── mod1
        │   ├── sub_mod11
        │   └── sub_mod12
        └── mod2
            ├── sub_mod21
            ├── sub_mod22
            └── sub_mod21
```
### モジュールパス
モジュールツリーのパスには2種類ある．
- **絶対パス**: クレート名もしくは，`crate`を使うことで，クレートルートからスタートする．
- **相対パス**: `self`，`super`またはモジュール内の識別子を使うことで，現在のモジュールからスタートする．

`src/lib.rs`に以下のようになっている場合，
```rust
mod foo {
    mod bar {
        fn add() {}
    }
}

pub fn foo_bar_add() {
    // 絶対パス
    crate::foo::bar::add();

    // 相対パス
    foo::bar::add();
}
```
**しかし，これはコンパイルできない．**`foo`モジュールにはアクセスできるが，`foo`内の`bar`モジュール，`bar`内の`add()`関数はそれぞれプライベートなので，`pub`キーワードで公開する．

```rust
mod foo {
    pub mod bar {
        pub fn add() {}
    }
}

pub fn foo_bar_add() {
    // 絶対パス
    crate::foo::bar::add();

    // 相対パス
    foo::bar::add();
}
```
また，呼び出したい関数や構造体等が親モジュールから始まる場合，`super`を使う．これはファイルシステムパスでの`..`のイメージ．
```rust
fn foo() {}

mod parent {
    fn bar() {
        child();
        super::foo();
    }

    fn child() {}
}
```

## `use`キーワードでパスをスコープに持ち込む
`use`キーワードを使うことで，絶対パスや相対パスを指定しなくても，パス内の要素にアクセスできる．

- 例1
    ```rust
    mod parent {
        pub mod child {
            pub fn foo() {}
        }
    }

    use parent::child;

    pub fn bar() {
        child::foo();
    }
    ```
同様に，`self`や`crate`を使っても同じ操作ができる．
- 例2
    ```rust
    mod parent {
        pub mod child {
            pub fn foo() {}
        }
    }

    use self::parent::child;  // use crate::parent::child;も同じ

    pub fn bar() {
        child::foo();
    }
    ```
また，関数をスコープに持ち込むこともできる．
- 例3
    ```rust
    mod parent {
        pub mod child {
            pub fn foo() {}
        }
    }

    use parent::child::foo;

    pub fn bar() {
        foo();
    }
    ```
例1, 3は同じ操作だが，**関数を持ち込む場合，例1を使うの慣例である．**
一方で，**構造体やenum等を持ち込む場合は，例3のようにフルパスを書くのが慣例．**
```rust
use std::collections::HashMap;

fn main() {
    let mut map = HashMap::new();
    map.insert(1, 2);
}
```
### エイリアス
`use`で持ち込んだ要素を`as`を使うことで，他の名前で使うことも可能．
```rust
use std::fmt::Result;
use std::io::Result as IoResult;

fn function1() -> Result {
    // --snip--
}

fn function2() -> IoResult<()> {
    // --snip--
}
```
### glob演算子
パス内で定義されているすべての公開要素をスコープに持ち込みたいときは，`*`を使う．
```rust
use std::collections::*;
```
glob演算子は，テストの際，テストされるあらゆるものを一気に持ち込むために使われる．
