# Rustの最適化

## インライン展開

`#[inline]`アトリビュートをつけることで関数をインライン展開できる。
- `#[inline(always)]` : crate境界内で、その関数を（できるだけ）インライン展開
- `#[inline(never)]` : 関数を（できるだけ）インライン展開しない

```rust
#[inline(always)]
pub fn add(a: i32, b: i32) -> i32 {
    a + b
}
```

## Enumの縮小化

Enum要素のうち他の要素と比べて大きなサイズの型をBox化することで、ヒープ割り当てが必要になる代わりにEnum自体のサイズが削減できる。

```rust
type LargeType = [u8; 100];

enum A {
    X,
    Y(i32),
    Z(i32, LargeType),
}

enum B {
    X,
    Y(i32),
    Z(Box<(i32, LargeType)>),
}
```

## `SmallVec<[T;N]>`
`Vec<T>`の代わりに`SmallVec<[T;N]>`を使うと、N個の要素はスタックに保存され、N+1個以降の要素はヒープに保持されることによって、アロケーションコストを削減できる。
ただし、`SmallVec`はアクセス時に特定の要素がアロケーションされているかチェックする必要があるので、操作コストがわずかに増える。
