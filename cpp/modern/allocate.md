# Modern C++時代の動的な変数アロケーション

Modern C++では基本的にアプリケーションコードに`new`キーワードを書くことはしない．
以下の 4 パターンのいずれかを使ってアロケーション/構築を行う．

- ローカル変数として構築
- `make_unique`を使用して構築
- `make_shared`を使用して構築
- ターゲットのオブジェクトの中に構築

## 変数アロケーションの選択肢

### ローカル変数として構築

ムーブのコストが小さいのであればよい．代表的な動的構造である`std::string`や`std::vector`，`std::map`のようなオブジェクトはムーブのコストが小さいのでそのまま引き回す．
代入やオブジェクトの引き渡しが発生した際に，オブジェクトがムーブされたのかコピーされたのか確実に把握して有効なデータのありかを管理しましょう．

```cpp
MovableObj function()
{
    MovableObj mc;
    return mc;
}
```

### `make_unique`で構築

ムーブできなかったり，ムーブ効率の悪いオブジェクトを引き回したいときは`unique_ptr`を使う．
かつて`new`で行っていたオブジェクト生成は`unique_ptr`を生成する`make_unique`関数で代替するのが第一候補．
実行効率的には生ポインタをハンドリングすることとほぼ同等とされる．

```cpp
std::unique_ptr<ImmobilizedObj> foo() {
    // 内部では動的な領域が確保され，ImmobilizedObj(100)相当の初期化が実行される
    return std::make_unique<ImmobilizedObj>(100);
}
```

`unique_ptr`で確保したオブジェクトを明示的に`delete`する必要はなく，`unique_ptr`ポインタの寿命が切れた際に領域を自動解放する．
`unique_ptr`はコピーに対応せず，ムーブのみに対応するため，代入元の`unique_ptr`は無効になる．

下の例では，`foo`関数で生成されたオブジェクトを`boo`関数が受け取った後`boo`関数が`unique_ptr`を保持し，`bar`関数に渡すことなく握り続けている．
つまり，`f`の生存期間を`boo`関数が管理していることと同義．このような状況を「`boo`が`f`の所有権を握っている」などと表現する．

```cpp
std::unique_ptr<ImmobilizedObj> foo() {
    return std::make_unique<ImmobilizedObj>(100);
}

void bar(ImmobilizedObj& m) {
    // DO SOMETHING
}

void boo() {
    auto f = foo(); // fはfooからムーブされる
    std::cout << f->getNum() << std::endl; // fのメンバ変数を出力
    bar(*f); // オブジェクトの生存期間が確保できていれば参照渡しでもよい
    // ここでImmobilizedObjのデストラクタが呼ばれる
}
```
