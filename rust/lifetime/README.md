# ライフタイム注釈

## [前提]借用規則
1. 不変・可変を問わず，参照元のライフタイムは参照先のライフタイムより短くなくてはいけない．
```rust
{
	let x = "foo".to_string(); // -------+-- 'a
	{                          // -+--'b |
		let y = &x;            //  |     |
	}                          // -+     |
	                           // -------+
}
```
2. 値が共有されている間(不変の参照が有効な間)は値の変更は許さない．すなわちオブジェクト`T`に対して以下のいずれかの状態のみしか存在できない．
   a. 任意の数の不変の参照`&T`を持つ．
   ```rust
   let x = 10;
   let y = &x;  // let mut y だとエラー
   let z = &x;  // let mut y だとエラー
   ```
   b. 単一の可変の参照`&mut T`を持つ．
   ```rust
   let x = 10;
   ```

以下のようなコードを書いてみる．
```rust
fn longest(x: &str, y: &str) -> &str {
	if x.len() > y.len() {
		x
	} else {
		y
	}
}

int main() {
	let x = "foo".to_string();
	let y = "barbar".to_string();
	let z = longest(&x, &y);
}
```
これはコンパイルエラーになる．
```shell
   Compiling lifetime v0.1.0 (/home/ktro2828/sandbox/rust/lifetime)
error[E0106]: missing lifetime specifier
 --> src/main.rs:1:33
  |
1 | fn longest(x: &str, y: &str) -> &str {
  |               ----     ----     ^ expected named lifetime parameter
  |
  = help: this function's return type contains a borrowed value, but the signature does not say whether it is borrowed from `x` or `y`
help: consider introducing a named lifetime parameter
  |
1 | fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
  |           ++++     ++          ++          ++
```
この関数の戻り値は借用された値(参照)だが，関数のシグネチャからは，戻り値が`x`，`y`のどちらからの借用なのか読み取れない．
そこで注釈で，`x`と`y`はどちらも同じライフタイムを持つことを教える．
書き方は，
`fn 関数名<'注釈名>(変数1: &'注釈名 型, 変数2: &'注釈名 型,...) -> &'注釈名 型 {...}`
のようにジェネリクスのように書く．

```rust
fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
	if x.len() > y.len() {
		x
	} else {
		y
	}
}
```

`x`よりも`y`のライフタイムが長いときはどうすればいいか？
ジェネリクスと同様，ライフタイム注釈をもう一つ追加する．
このとき，`where 'b: 'a`は`y`の方がライフタイムが長いことを示しており，
これが無い，もしくはあっても戻り値が`&'b str`だとコンパイルエラーになる．

```rust
fn longest<'a, 'b>(x: &'a str, y: &'b str) -> &'a str where 'b: 'a {
	if x.len() > y.len() {
		x
	} else {
		y
	}
}
```
