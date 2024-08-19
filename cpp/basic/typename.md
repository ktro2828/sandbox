# `typename\*\*

C++ではテンプレート使用時に`typename`をよく使うがそれ以外にも必要になる場合があり、主に 2 パターンの使い方がある。

## テンプレートパラメータでの`typename`

こちらは`typename`のよく使う使用方法。テンプレートのパラメータ`T`が**型であることを宣言**するために`typename`を使う。
ここでは`typename`の代わりに`class`を使うこともできる。

```c++
template <typename T> // or template <class T>
class Point
{
  T x_;
  T y_;
};
```

どちらを使っても構わないが、使い分けとしては以下を参考:

| Keyword  | Use case               |
| -------- | ---------------------- |
| typename | 基本型を使うことを想定 |
| class    | それ以外               |

## 階層を持つテンプレートのための`typename`

例として、さっきの`Point`クラスの配列(`vector`)を持つ多角形(`Polygon`)クラスで考える。

```c++
template <typename T>
class Polygon
{
  std::vector<Point<T>> points_;

public:
  using value_type = T;
  using size_type = typename std::vector<Point<T>>::size_type;
  using iterator = typename std::vector<Point<T>>::iterator;
  using const_iterator = typename std::vector<Point<T>>::const_iterator;

  size_type size() const { return points_.size(); }

  iterator begin() { return points_.begin(); }
  iterator end() { return points_.end(); }
  const_iterator begin() const { return points_.begin(); }
  const_iterator end() const { return points_.end(); }
};
```

`using size_type = typename ...`としないと以下のようなコンパイルエラーが出る。

```shell
typename.cpp:38:23: error: need 'typename' before 'std::vector<Point<T> >::size_type' because 'std::vector<Point<T> >' is a dependent scope
     using size_type = std::vector< Point<T> >::size_type;
```

`T`はテンプレートパラメータのため不確定なため、`Point<T>`も不確定で、`std::vector<???>::size_type`という状況。
`ClassName::xxx`は型だけでなく、定数に使える。そのため、型かどうかわからないものに対して別名定義である`using`を使うと、コンパイルエラーになる。
コンパイルエラーを解消するには、`typename`を記述して、**次に来るのが型だとコンパイラに明示的に伝える**必要がある。

ただし、以下の場合は例外

1. 派生クラスの定義で基底クラスを指定する場合
2. コンストラクタの初期化リストで基底クラスを指定する場合

```c++
class Base {
public:
  class Nested { // ネストされたクラス型
  public:
    explicit Nested(int) {};
  };
};

template<class B>
class Derived : public B::Nested { // 1. typenameは不要
public:
  explicit Derived(int x) : B::Nested(x) {} // 2. typenameは不要
};

int main(int argc, const char *argv[]) {
  Derived<Base> d(10); // OK
  return 0;
}
```
