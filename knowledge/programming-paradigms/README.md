# Programming Paradigms: 手続き型(Procedural) vs 関数型(Functional) vs オブジェクト指向(Object-Oriented)

## サマリ

| 観点         | 手続き型(Procedural)                        | 関数型(Functional)                           | オブジェクト指向(Object-Oriented)                            |
| ------------ | ------------------------------------------- | -------------------------------------------- | ------------------------------------------------------------ |
| 中心概念     | 手続き・状態更新(procedures, mutable state) | 純粋関数・合成(pure functions, compositions) | オブジェクト・カプセル化(objects, encapsulation)             |
| 状態管理     | 直線更新(in-place updates)                  | 不変(immutability)、必要時に明示(effects)    | オブジェクト内部に閉じ込める(encapsulation)                  |
| 再利用性     | ルーチン分割・共通化                        | 公開関数・型抽象(higher-order, ADTs)         | 継承/移譲/インタフェース(inheritance/composition/interfaces) |
| 並行性       | ロック中心(locks, threads)                  | 不変&純粋で安全に並列化しやすい              | 役者/メッセージや不変オブジェクトで設計次第                  |
| テスト容易性 | 副作用が多いと難化                          | 純粋関数はテスト容易                         | モック/DI前提でテスト                                        |
| 学習コスト   | 低〜中                                      | 中〜高(抽象の理解)                           | 中(設計原則の習得)                                           |
| 代表言語     | C, early Python                             | Haskell, OCaml, Rust/Scalaの関数的側面       | Java, C++, Python/TSのOOPの側面                              |
| 典型落とし穴 | グローバル状態の氾濫                        | 過度な抽象(monad soup)                       | 継承地獄/God Object                                          |

> 実務では**ハイブリッド**が標準: **Functional Core, Imperative Shell, OOP + 不変値(Value Objects), データ至幸設計(Data-Oriented Design, ECS)**など

## 1. 手続き型(Procedural)

- **要点**: 手続き(procedure)、逐次実行(sequence)、可変状態(mutable state)
- **強み**
  - I/Oやデバイス制御など**状態主体の処理**に素直
  - 低レベル制御・性能チューニング(キャッシュ局所性、in-place更新)
- **弱み**
  - 状態が増えるほど**把握・検証が難化(implicit contracts)**
  - 並行化はロック競合やデータ競合に注意
- **典型ユース**: 組み込み/HPC、ストリーミング処理の低レベル部分、数値カーネル

## 2. 関数型(Functional)

- **要点**: 純粋関数・参照透過性(pure functions, referential transparency)、不変(immutability)、高階関数(higher-order functions)、代数的データ型(ADTs)、パターンマッチ(pattern matching)
  - **効果**: 例外/IO/状態は**型で追跡**(effect typing, monads, algebraic effects)
- **強み**
  - **合成(composition)**と**テスト容易性**
  - 不変中心で**安全に並列化しやすい**
  - 網羅性チェック(exhaustiveness)でバグ削減
- **弱み**
  - 思想と抽象(モナド/型クラス)に学習コスト
  - in-placeが苦手 -> 性能要件で工夫(持続データ構造、fusion、適切な逃げ道)
- **典型ユース**: データ変換パイプライン、評価・解析、分散処理のコアロジック

## 3. オブジェクト指向(OOP)

- **要点**: オブジェクト(state+behavior)、カプセル化(encapsulation)、ポリモーフィズム(polymorphism)、継承/移譲(inheritance/composition)、SOLID原則
- **強み**
  - **境界(API)設計**と**長寿ドメインのモデリング**に強い
  - UI/ドメイン挙動の拡張(サブタイプ/インタフェース)に適する
- **弱み**
  - 深い継承やミュータブル共有で複雑化
  - データ指向な高性能処理とは相性が出る場合
- **典型ユース**: GUI/業務システム/長期運用サービス、プラグイン拡張

## 4. 同じ課題を各パラダイムで(e.g. 「偶数の二乗の総和」)

### 手続き型(C)

```c
int sum_sq_even(const int * x, int n) {
    int s = 0;
    for (int i = 0; i < n; ++i) {
        if (x[i] % 2 == 0) {
            int a = x[i];
            s += x * x;
        }
    }
    return s;
}
```

### 関数型(Rustの関数的イテレータ)

```rust
fn sum_sq_even(x: &[i32]) -> i32 {
    x.iter()
        .copied()
        .filter(|x| x % 2 == 0)
        .map(|x| x * x)
        .sum()
}
```

### オブジェクト指向(Pythonの「コレクションに振る舞いをもたせる」)

```python
class IntList:
    def __init__(self, xs: list[int]):
        self.xs = xs

    def sum_sq_even(self) -> int:
        return sum(x*x for x in self.xs if x % 2 == 0)
```

## 5. 設計指針(When to choose what?)

- **I/O中心・リアルタイム・厳しいキャッシュ最適化** -> 手続き型/データ指向で**ホットパスを明確化**
- **データ変換・検証・統計/MLの前処理** -> 関数型の**合成(composition)**と**不変(immutability)**でバグ・競合を減らす
- **長期運用のドメイン/GUI/拡張可能なAPI** -> OOPで**境界を明示**、内部は不変値+小さなメソッド
- **ハイブリッド**
  - **Functional Core, Imperative Shell**: 純粋計算を関数型、外界I/Oを手続き型で囲う
  - **OOP + 不変値(Value Objects)**: ドメイン値はイミュータブル、サービスは副作用を持つ
  - **ECS(Entity-COmponent-System)**: データ指向、並列実行しやすい

## 6. 並行・並列(Concurrency/Parallelism)

- **手続き型**: スレッド+ロックでデータ所有を明確に、共有は限定
- **関数型**: 不変x純粋関数で**安全な並列**でMpa/Reduceパターン、Actorも相性良
- **OOP**: Actor/メッセージ駆動(mailbox, supervision)で**オブジェクト境界=スレッド境界**に

## 7. エラー処理と検証(Error Handling & Verification)

- **手続き型**: 戻り値/errno -> 規約を徹底(早期return、guard)
- **関数型**: `Option/Result`、総称代数データ型(ADTs)で**型が契約**
- **OOP**: 例外or`Result`型等で`例外の境界(外に出さない/まとめる)`を設計

## 8. 性能(Performance) -- 誤解の解消

パラダイム自体は遅速を決めるわけではない！！

- **手続き型**: メモリアクセスを最適化しやすい
- **関数型**: 一見アロケーション増でも、**インライン/融合(Fusion)**で高速化可能
- **OOP**: 仮想呼出しやポリモーフィズムは**ホットパスでは避けて分離([Pimpl](https://learn.microsoft.com/ja-jp/cpp/cpp/pimpl-for-compile-time-encapsulation-modern-cpp?view=msvc-170)/[CRTP](https://en.cppreference.com/w/cpp/language/crtp.html)/[コンセプト](https://cpprefjp.github.io/lang/cpp20/concepts.html))**

## 9. アンチパターン(Pitfalls)

- **手続き型**: グローバル状態/可変の蔓延、蜜結合
- **関数型**: 過抽象、分かりにくい型
- **OOP**: 継承深堀り、God Object, setterだらけで不変性ゼロ
