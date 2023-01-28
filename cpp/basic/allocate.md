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
代入やオブジェクトの引き渡しが発生した際に，オブジェクトがムーブされたのかコピーされたのか確実に把握して有効なデータのありかを管理する．

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

- 例１

  ```cpp
  std::unique_ptr<ImmobilizedObj> foo() {
      // 内部では動的な領域が確保され，ImmobilizedObj(100)相当の初期化が実行される
      return std::make_unique<ImmobilizedObj>(100);
  }
  ```

`unique_ptr`で確保したオブジェクトを明示的に`delete`する必要はなく，`unique_ptr`ポインタの寿命が切れた際に領域を自動解放する．
`unique_ptr`はコピーに対応せず，ムーブのみに対応するため，代入元の`unique_ptr`は無効になる．

- 例 2
  下の例では，`foo`関数で生成されたオブジェクトを`boo`関数が受け取った後`boo`関数が`unique_ptr`を保持し，`bar`関数に渡すことなく握り続けている．
  つまり，`f`の生存期間を`boo`関数が管理していることと同義．このような状況を「`boo`が`f`の所有権を握っている」などと表現する．
  `unique_ptr`のメンバにアクセスする場合は，旧来のポインタ同様アロー演算子`->`でアクセスする．

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

- 例 3(オブジェクトに対してムーブする場合)

  ```cpp
  // unique_ptrを返す関数
  std::unique_ptr<ImmobilizedObj> foo() {
    return std::make_unique<ImmobilizedObj>(100);
  }

  class ParentObj
  {
    private:
        std::unique_ptr<ImmobilizedObj> ptr = nullptr;
    public:
        // 右辺値参照でムーブするsetter
        void setImmobilized(unique_ptr<ImmobilizedObj> && ptr) {
            // unique_ptrを受け取るときはムーブ(=コピー不可)なので，右辺値参照で受けるのが行儀が良い
            this->ptr = std::move(ptr);  // ムーブ代入
        }

        // デストラクタ
        ~ParentObj() {
            this->ptr = nullptr;  // unique_ptrのデストラクタが呼び出される．
            // 明示的にnullptrを代入しなくても~ParentObjの最後(=ライフライム終了後)にデストラクタが呼ばれる
        }
  }

  void boo() {
    auto f = foo();  // fはfooからムーブされる
    {
        ParentObj p;
        p.setImmobilized(std::move(f));  // pはfからムーブされたunique_ptrを受け取る
    }
    // fはpにムーブかつ，pのライフタイムが終了したので，foo()が生成したunique_ptrはここで存在しない．
    // ...
  }
  ```

  ※左辺値(lvalue)，右辺値(rvalue)の話は[move.md](./move.md)を参照．

### `make_shared`で構築する

`unique_ptr`は実行効率はいいが，コピーができないため複数のオブジェクトから参照することができない．
そのような場合には`shared_ptr`を使う．
`shared_ptr`は対象となるすべての`shared_ptr`が破棄されたときに，管理対象のオブジェクトのデストラクタを呼び，領域を自動解放する．
便利だが，`unique_ptr`よりも実行効率が劣るため，適切な方を使う．

- 例

  ```cpp
  // shared_ptrを返す関数
  std::shared_ptr<ImmobilizedObj> foo() {
    return std::make_shared<ImmobilizedObj>(100);
  }

  class ParentObj
  {
    private:
        std::shared_ptr<ImmobilizedObj> ptr = nullptr;
    public:
        // shared_ptrを左辺値参照で受け取るsetter
        void setImmobilized(const shared_ptr<ImmobilizedObj> &ptr) {
            // share_ptrをコピーするので左辺値参照でOK
            // ムーブしたい場合は，右辺値参照にする
            this->ptr = ptr;  // shared_ptrがコピーされ，参照数+1
        }

        // デストラクタ
        ~ParentObj() {
            this->ptr = nullptr;  // share_ptrのデストラクタが呼ばれる
            // 明示的にnullptrを代入しなくても~ParentObjの最後でデストラクタが呼ばれる
        }
  };

  void boo() {
    auto f = foo(); // fはfooからムーブされる
    {
        ParentObj p1, p2;
        p1.setImmobilized(f);  // fをp1に対してコピー
        p2.setImmobilized(f);  // fをp2に対してコピー
        f = nullptr;  // オリジナルを破棄
    }
    p1.setImmobilized(nullptr);
    // まだfooが生成したshared_ptr<ImmobilizedObj>は生きている
    p2.setImmobilized(nullptr);
    // 全てのshared_ptrインスタンスが消えたので，shared_ptr<ImmobilizedObj>のデストラクタが呼ばれ，破棄される
  }
  ```

## 参考

- [Modern C++時代の動的な変数アロケーション](https://zenn.dev/dec9ue/books/8c59757478a547/viewer/096fed)
