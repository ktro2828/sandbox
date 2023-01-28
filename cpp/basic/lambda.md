# ラムダ式

ラムダ式の基本的な記法は以下

```cpp
auto function = []( auto value ) { return value; };
```

細かく分解すると以下のようになる．

```
[] // ラムダ導入子  ...キャプチャ方法を指定する．
() // 引数リスト    ...通常の関数同様，型名と引数名を指定する．型名にautoを指定することで任意の型をを適用できる．(=ジェネリックラムダ)
{} // 複合文        ...関数の処理内容．
```

ラムダ式の型指定には，`auto`もしくは`std::function`を使う．

`std::function<戻り値の型(引数の型,...)>`

```cpp
#include <functional>

// typedefでエイリアス
typedef std::function<float(float, float)> fFunc;

int main() {
  std::function<int(int)> f = [](int x) { return ++x; };

  fFunc ff = [](float y, float z) { return y + z; };
}
```

## キャプチャ

ラムダ式は，スコープ内のローカル変数を使うことができる．
キャプチャには`コピーキャプチャ`と`リファレンスキャプチャ`がある．

### コピーキャプチャ: `[=]`

コピーキャプチャは変数をコピーによってキャプチャする．

```cpp
int main() {
  int x = 0;
  // コピーキャプチャ
  auto f = [=] { return x; };
}
```

コピーキャプチャした変数はラムダ式の中で変更できない．

```cpp
int main() {
  int x = 0;
  // Error!!
  auto f = [=] { x = 1; };
}
```

## リファレンスキャプチャ: `[&]`

リファレンスキャプチャは変数をリファレンス（参照）によってキャプチャする．
リファレンスキャプチャでは，キャプチャした変数をラムダ式の中で変更可能．

```cpp
int main() {
  int x = 0;
  auto f = [&]{ ++x; };

  f(); // x == 1
  f(); // x == 2
  f(); // x == 3
}
```

## キャプチャリスト

キャプチャの内容を変数ごとに明示的に指定することもできる．

| 記法      | 概要                                                       |
| :-------- | :--------------------------------------------------------- |
| `[&]`     | デフォルト参照キャプチャ（全ての変数を参照キャプチャ)      |
| `[=]`     | デフォルトコピーキャプチャ（全ての変数をコピーキャプチャ） |
| `[&x]`    | 変数 x のみを参照キャプチャ                                |
| `[x]`     | 変数 x のみをコピーキャプチャ                              |
| `[&x, y]` | x を参照，y をコピーキャプチャ                             |
| `[&, x]`  | x はコピー，その他は全て参照キャプチャ                     |
| `[=, &x]` | x は参照，その他全てはコピーキャプチャ                     |

## 初期化キャプチャ(>=C++14)

C++14 以降では，キャプチャに任意の式を書くことができるようになった．
`unique_ptr`のようなコピーできない変数は`std::move()`によって move キャプチャすることができる．

```cpp
auto x = std::make_unique<int>(1);
auto func = [y=std::move(x)]() mutable {
  std::cout << *y << std::endl;
  *y++;
}
```

## パラメータパック

引数リストに`auto ... args`を指定することで複数型の引数を任意の数指定できる．

```cpp
#include <iostream>

int main() {
  auto fn = [](auto ... args) {
    (std::cout << ... << args) << std::endl;  // Fold表現で展開．
  }
}
```

なお，Fold 表現は`-std=c++17`または`-std=gnu++17`のみで使用可能なので警告がでる．

```shell
xxx.cpp:5:29: warning: fold-expressions only available with ‘-std=c++17’ or ‘-std=gnu++17’
    5 |        (std::cout << ... << args) << std::endl;
      |                             ^~~~
```

## Example

- `std::for_each()`にラムダ式を指定して，ベクタの各要素を標準出力する例．

```cpp
#include <iostream>
#include <vector>
#include <algorithm>

void print(std::vector<int> &v) {
std::cout << "[ ";
std::for_each(v.begin(), v.end(), [](const int &n) {std::cout << n << " ";});
std::cout << "]" << std::endl;
}

int main() {

std::vector<int> x{1, 2, 3};

print(x);
}
```
