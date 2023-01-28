# コピーの禁止

オブジェクトのコピーを禁止したい場合には，`delete定義`によってコピーコンストラクタとコピー代入演算子を消去できる．

```cpp

class CopyDisableObj
{
public:
    SomeObj x;

    // コピーコンストラクタ
    CopyDisableObj(const CopyDisableObj &) = delete;

    // コピー代入演算子
    CopyDisableObj& operator=(const CopyDisableObj &) = delete;

    // デフォルトコンストラクタ
    CopyDisableObj() {}

    // ムーブコンストラクタ
    CopyDisableObj(CopyDisableObj && other) : x(other.x) { other.x = nullptr; }

    // ムーブ代入演算子
    CopyDisableObj& operator=(CopyDisableObj && other) : x(other.x) { other.x = nullptr; }
};
```

## 5 原則

C++には「5 原則」という作法があり，「この内 1 つを独自に定義したのならば，残りの 4 つも定義するべきである．」というもの．

1. コピーコンストラクタ
2. コピー代入演算子
3. ムーブコンストラクタ
4. ムーブ代入演算子
5. デストラクタ
