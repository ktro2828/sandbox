# ムーブセマンティクスと関数の引数と戻り値のベストプラクティス

## 左辺値(lvalue)と右辺値(rvalue)

- 左辺値: 名前のついたオブジェクト．名前のスコープが切れたところでデストラクタが呼ばれる．

```cpp
#include <vector>

void func() {
	std::vector<int> v;
	// DO SOMETHING 1
	v.push_back(1);
	// DO SOMETHING 2
	v.push_back(2);
	// スコープが切れるところでデストラクタが呼ばれる
}
```

- 右辺値: 名前の着いてない，いわゆる**一時オブジェクト**．式全体の評価が終わったときにデストラクタが呼ばれる．

```cpp
#include <vector>

void func() {
	// pop()の処理が終わったときにvector(一時オブジェクト)のデストラクタが呼ばれる．
	int i = (std::vector<int>{1, 2, 3}).pop();
	// DO SOMETHING 1
	// vectorにアクセスする方法はない．
	i++;
	// DO SOMETHING 2
	i = i + 2;

	return;
}
```

## コピーとムーブ

次のような大きなデータを含む構造があるとする．

```cpp
std::vector<int> someBigVector{0,1,2,...,200000};
```

このような大きなデータでもコピーコンストラクタや代入演算子を使うことで複製できる．

```cpp
// コピーコンストラクタ
std::vector<int> otherBigVector1(someBigVector);
// 代入演算子
std::vector<int> otherBigVector2 = some;
```

`vector`の内部データを複製せず，**ポインタの挿げ替えだけした方が効率が良い場合が多々ある．**

過去の C++では，以下のような方法があった．

- 呼び出し元で領域を確保し，参照に対してデータを書き込む．
- `new`や`auto_ptr`を使用してオブジェクト全体をダイナミックに確保してポインタ渡し．
- コピーコンストラクタ，代入演算子の複製版と移動版を作成して使い分ける．

↑ のような方法では，記述や処理そのものが冗長になる，オブジェクトの種類によってコピーコンストラクタや代入演算子の意味合いが変わってくるなど混乱を招くことが問題になっていた．

C++11 以降では **「移動」に関わる処理はムーブコンストラクタ，ムーブ演算子というカテゴリを別途設けることで「コピー」とは処理を明確に区別する．**

## ムーブコンストラクタ，ムーブ代入演算子の例

```cpp
class BigArray {
	private:
		char* big_data_;

	public:
		// デフォルトコンストラクタ
		BigArray() : big_data_(new char[BIG_SIZE]) {}

		// ムーブコンストラクタ
		BigArray(BigArray&& old) : big_data_(old.big_data_)
		{
			old.big_data_ = nullptr;
		}

		// ムーブ演算子
		BigArray& operator=(BigArray&& old)
		{
			this->big_data_ = old.bid_data_;
			old.big_data_ = nullptr;
			return *this;
		}
}

// コピー
BigArray arr1;
BigArray arr2 = arr1; // コピーコンストラクタを使用
BigArray arr3;
arr3 = arr2; // コピー代入演算子を使用

// ムーブ
BigArray arr4;
BigArray arr5 = std::move(arr4); // ムーブコンストラクタを使用
BigArray arr6;
arr6 = std::move(arr5); // ムーブ代入演算子を使用
```

`&&`は右辺値参照と呼ばれる．右辺値参照を関数の引数にした場合，右辺値にのみマッチする = 「破壊されていい」オブジェクトにのみ適用される．
上の例で`arr4`や`arr5`はオブジェクトとしての実態は残るもののデータは破壊されているので注意．

### 関数引数の型マッチングの整理

| 型のパターン |            典型的な使用ケース            |
| :----------- | :--------------------------------------: |
| `T`          |    データをコピーして使用する際に使用    |
| `const T&`   |       元のデータを読み出す際に使用       |
| `T&`         |      元のデータを書き換える際に使用      |
| `T&&`        | 元のデータをムーブして破壊すると際に使用 |

※`const T&&`となることは性質上ありえない

## 戻り値の指定の仕方

### 戻り値の値返しの基本

関数の戻り値にはコピーよりもムーブが優先される．具体的には，戻り値では左辺値を書いても右辺値のように扱われるため，`std::move`を書く必要がない．

```cpp
MovableObj function() {
	MovableObj mc;
	return mc; // ムーブが優先される
}
```

※ムーブができないオブジェクトの場合，コピーを試みる．(ムーブもコピーもできないオブジェクトはスコープから出られないのでコンパイルエラー)

### RVO と NRVO

上の例では，「ムーブが優先される」と曖昧に書いてあるが，このまま実装するとコピーもムーブもしないコードを生成するコンパイラが多い．

- RVO(Return Value Optimization): **「右辺値を値返しするコードを書いたときに，呼び出し元が確保した領域にオブジェクトを構築する仕組み」**．

  - C++17 以降は RVO はコンパイラのサポートが必須とされたため，大体のコンパイラは RVO ベースでコードを吐いてくれる．なお，RVO が効いた場合，コピー/ムーブ系の処理の実装がなくてもコンパイルが通る．

  ```cpp
  MovableObj function()
  {
  	return MovableObj(); // <- この右辺値が確保されるのは呼び出し元のメモリ領域
  }

  int main()
  {
  	MovableObj mc = function(); // <- 最初からここにオブジェクトの領域が割り当てられる
  }
  ```

- NRVO(Named Return Value Optimization): **「左辺値を値返しするコードを書いたときに，呼び出し元が確保した領域にオブジェクトを構築する仕組み」**．

  - NRVO はコンパイラのサポートが必須でないため，必ず適用されるとは限らない．

  ```cpp
  MovableObj function()
  {
  MovableObj m; // <- NRVO有効時，この左辺値が確保されるのは呼び出し元のメモリ領域
  return m;
  }

  int main()
  {
  MovableObj mc = function(); // <- NRVO有効時は最初からここにオブジェクトが割り当てられる
  }
  ```

  - NRVO がうまくいかない例

  ```cpp
  MovableObj function()
  {
  MovableObj m1, m2; // <- m1, m2どちらにNRVOを適用していいかオブジェクトの生成時点では不明!

  if (some_condition())
  {
  	return m1;
  } else {
  	return m2;
  }
  }
  ```
