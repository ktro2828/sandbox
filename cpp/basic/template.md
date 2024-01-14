# テンプレート

https://learn.microsoft.com/ja-jp/cpp/cpp/templates-cpp?view=msvc-170

## テンプレートの使用と作成

以下は基本的な関数テンプレートの例

```c++
template <typename T>
T minimum(const T & lhs, const T & rhs)
{
    return lhs < rhs ? lhs : rhs;
}

// int型でテンプレートのインスタンスを宣言
int a = 0, b = 1;

int i = minimum<int>(a, b);
```

ただし、これは関数テンプレートで`a`と`b`の型から`T`の型を推測できるので、通常の関数のように呼び出し可能。

```c++
int i = minimum(a, b);
```

コンパイラは最後のステートメントを検出すると、テンプレート内の`T`のすべての出現箇所を`int`に置き換えた新しい関数を生成する。

```c++
int minimum(const int & lhs, const int & rhs)
{
  return lhs < rhs ? lhs : rhs;
}
```

## 型パラメータ

上記の`minimum`テンプレートでは、型パラメータ`T`は、`const`および参照修飾子が追加される関数呼び出しパラメータで使用されるまで、どのような方法でも修飾されない。

型パラメータの数には事実上制限はない。複数のパラメータをコンマで区切る。

```c++
template <typename T, typename U, typename V> class Foo{};
```

キーワード`typename`は`class`でも代用可能。

```c++
template <class T, class U, class V> class Foo{};
```

省略記号演算子`...`を使うと、任意の0個以上の型パラメータを受け取るテンプレートを定義可能。

```c++
template <typename... Arguments> class vtclass;

vtclass< > vtinstance1;
vtclass<int> vtinstance2;
vtclass<float, bool> vtinstance3;
```

任意の組み込みまたはユーザー定義型を型引数として使用可能。例えば、標準ライブラリで`std::vector`を使用して、型`int`、`double`、`std::string`、`MyClass`、`const MyClass*`、`MyClass&`などの変数を格納できる。

テンプレートを使用する場合の主な制限は、型引数が型パラメータに適用されるすべての操作をサポートする必要があること。

```c++
class MyClass
{
public:
  int num;
  std::wstring description;
};

int main()
{
  MyClass mc1 {1, L"hello"};
  MyClass mc2 {2, L"goodbye"};
  auto result = minimum(mc1, mc2); // Error! [C2678](https://learn.microsoft.com/ja-jp/cpp/error-messages/compiler-errors-2/compiler-error-c2678?view=msvc-170)
}
```

上の例では、`MyClass`は`<`演算子のオーバロードが未定義のため、コンパイルエラーが生成される。

特定のテンプレートの型引数がすべてのオブジェクト階層に属している固有の要件は無いが、このような制限を適用するテンプレートを定義することはできる。
オブジェクト指向の手法をテンプレートと組み合わせることができる。
例えば、`std::vector<Base*>`で`Derived*`を格納できる。引数はポインタである必要があることに注意。

```c++
#include <memory>
#include <vector>

std::vector<MyClass*> vec;

MyDerived d(3, L"back again", time(0));
vec.push_back(&d);

// or more relalistivally:
std::vector<shared_ptr<MyClass>> vec2;
vec2.push_back(std::make_shared<MyDerived>());
``**

このときの`T`が満たすべき要件は、`T`がcopy-assignableかつcopy-constructibleであること。

## 非型パラメータ

C++テンプレートでは、値パラメータと呼ばれる*非型パラメータ*がサポートされている。

```c++
template <typename T, std::size_t L>
class MyArray
{
private:
  T arr_[L];
public:
  MyArray() { /* Do somothing */ };
};
```

このとき、`std::size_t`の値はコンパイル時にテンプレート引数として渡され、`const`式または`constexpr`式である必要がある。

```c++
MyArray<MyClass*, 10> arr;
```

## 非型テンプレートパラメータの型推論

C++17以降では、コンパイラは`auto`で宣言されている非型テンプレート引数の型を推測する。

```c++
template <auto x> constexpr auto constant = x;

auto v1 = constant<5>;    // v1 == 5, decltype(v1) is int
auto v2 = constant<true>; // v2 == true, decltype(v2) is bool
auto v3 = constant<'a'>;  // v3 == 'a', decltype(v3) is char
```

## テンプレートパラメータとしてのテンプレート

テンプレートは、テンプレートパラメータにすることができる。
下の例では、`MyClass2`には、typenameパラメータ`T`とテンプレートパラメータ`Arr`の2つのテンプレートパラメータがある。

```c++
template <typename T, template<typename U, int I>> class Arr>
class MyClass2
{
  T t; // OK
  Arr<T, 10> a;
  U u; // Error. U not in scope
};
```

`Arr`パラメータ自体には本体がないため、パラメータ名は必要ない。実際には、`MyClass2`の本体内から`Arr`のtypenameまたはクラスパラメータ名を参照するとエラーになる。
このため、次の例のように、`Arr`の型パラメータ名を省略できる。

```c++
template <typename T, template <typename, int> class Arr>
{
  T t; // OK
  Arr<T, 10> a;
};
```

## 規定のテンプレート引数

クラステンプレートと関数テンプレートには、規定の引数を指定できる。テンプレートに規定の引数がある場合は、使用時にそのままにしておくことができる。
例えば、`std::vector`テンプレートには、アロケータの規定の引数がある。

```c++
template <class T, class Allocator = allocator<T>> class vector;
```

ほとんどの場合、規定の`std::allocator`クラスが許容されるが、必要に応じてカスタムアロケータを指定できる。

```c++
std::vector<int> myInts;
std::vector<int, MyAllocator> ints;
```

パラメータがすべて規定値のテンプレートの場合は、空の`<>`を使用する。

```c++
template <typename A = int, typename B = double>
class Bar
{
  // ...
};

int main()
{
  Bar<> bar; // use all default type arguments
}
```


## テンプレートの特殊化

任意の型を許容するテンプレートに対して、特定の型を指定して定義することを特殊化という。
すべてのパラメータを特殊化することを完全特殊化、一部のみを部分特殊化という。

```c++
template <typename K, typename V>
class MyMap { /* ... */ };

// partial specialization for string keys
template <typename V>
class MyMap<std::string, V> { /* ... */ };

MyMap<int, MyClass> classes; // uses original template
MyMap<std::string, MyClass> classes2; // uses the partial specialization
```
