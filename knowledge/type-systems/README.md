# 型システム (Type Systems)

## 1. WHat is a Type System?

**定義**: プログラミングの各式に**型(type)**を割り当て、誤りを早期に検出し、抽象化を支援する静的/動的な規則の集合。

### 目的 (Purposes)

- **安全性(Safety)**: 未定義動作や型不一致を抑止(例: `int + string`)
- **抽象化(Abstraction)**: 内部表現を隠し契約(contract)を明示(例: interface/trait)
- **ドキュメント(Documentation)**: 型が仕様の一部
- **最適化(Optimization)**: 実行時チェック削減やレイアウト最適化

## 2. 基本分類 (Key Axes)

- **静的型付け(Static typing)** vs **動的型付け(Dynamic typing)**
  - 例: Rust/C++(静的)、Python/JS(動的)
- **強い型付け(Strong typing)** vs **弱い型付け(Weak typing)**
  - 暗黙変換の強さや不正アクセス許容度の違い
- **名義型(Nominal typing)** vs **構造的型(Structural typing)**
  - 名義: 型名一致が要件(Java/C++)
  - 構造: メンバ構造一致(TypeScript/Goの一部・Rustの`impl Trait`は性質が近い)
- **合成(Sum/Product types)**
  - **直和型(sum types, tagged union)**: `A | B` (Rust `enum/Result/Option`, C++ `std::variant`)
  - **直積型(product type)**: レコード/構造体

## 3. 型付けの形式化 (Typing Judgement)

- 記法: `Γ ⊢ e : τ`(環境`Γ`で式`e`は型`τ`)
- **進行(Progress)**: 「よく型付けされた式は行き詰まらない(not stuck)」
- **保存(Preservation)**: 「評価で型が保存される」
- この２つが**健全性(Soundness)**の核

## 4. サブタイピング(Subtyping)と置換原則

- **LSP(Liskov Substitution Principal)**: `S <: T`なら`T`が期待される場所で`S`を使える
- **変性(Variance)**
  - **共変(Covariant)**: `T <: U -> F[T] <: F[U]`(戻り値など)
  - **反変(Contravariant)**: 引数位置
  - **不変(Invariant)**: ミュータブルなコンテナは通常不変(C++`std::vector<T>`, Rust`Vec<T>`)
- 例(C++): `std::vector<Derived>`は`std::vector<Base>`の**サブタイプではない**(不変)

## 5. 多相(Polymorphism)

- **パラメトリック多相(Parametric polymorphism/Generics)**
  - 単一実装で任意型に適用(Rust generics, C++ template)
- **包含多相(Subtype polymorphism)**
  - `Base*`に`Derived*`を渡す
- **アドホック多相(Ad-hoc polymorphism/Overloading/Type classes)**
  - Haskell: **type classes**、Rust: **traits**、C++ **concepts**

## 6. 型推論(Type Inference)

- **Hindley-Milner(HM)**: HM/OCaml/Haskell(拡張前)系の完全推論、**主型(principal type)**を持つ
- サブタイピングや型クラス、効果を入れると**完全推論**は難化 -> 多くの実用言語は**局所推論(local inference)**
- Rust: 単相化点やヒントが必要な場面あり(`collect::<T>`など)

## 7. 実務で効く型の設計(Practical Design Patterns)

### 7.1 NULL安全(Null-safety)

- **Option/Maybe**を使う(Rust: `Option<T>`、C++: `std::optional<T>`、TypeScript: `T | undefined`)

```rust
fn find(id: u64) -> Option<User> { /* ... */ }
```

### 7.2 エラーは例外より値で返す(Errors as Values)

- Rust: `Result<T, E>`、C++: `tl::expected<T, E>`(C++23 `std::expected`)

```rust
fn parse(s: &str) -> Result<Config, ParseError> { /* ... */ }
```

### 7.3 和型(Sum types)で状態を尽くす

- Rust: `enum`、C++: `std::variant`、TypeScript: `discriminated unions`

```rust
enum Payment {
    Cash(u32),
    Card { brand: Brand, last4: u16 }
}
```

### 7.4 インタフェース記述(Interfaces/Traits/Concepts)

- Rust: **trait**、C++: **concepts**で**契約(contract)**を型で表現

```cpp
template<class T>
concept Hashable = requires(T a) {
    { std::hash<T>{}(a) } -> std::convertible_to<size_t>;
};
```

### 7.5 変性/所有権/借用 (Variance, Ownership & Borrowing)

- Rustの**所有権(ownership)と借用(borrowing)はアフィン型(affine types)** + **リージョン(lifetimes)**の実用化
  - 可変エイリアス禁止・データ競合の型レベルでの防止

### 7.6 APIの戻り値の型選定

- 「失敗しない」-> `T`
- 「存在しないかも」-> `Option<T>` / `std::optional<T>`
- 「失敗する可能性」-> `Result<T, E>` / `std::expected<T, E>`
- 「部分列挙」-> `enum` / `variant`にディスクリミネータ
