# Flyweight

## 📝 Intent

Flyweight(フライウェイト、フライ級)は、構造に関するデザインパターンの一つで、複数のオブジェクト間で共通する部分を各自で持つ代わりに共有することによって、利用可能な RAM により多くのオブジェクトを収められるようにします。

## 💡 Key Concepts

- 内因的状態(Intrinsic state): イミュータブルで複数のオブジェクト間で共有可能なデータ
- 外因的状態(Extrinsic state): ミュータブルで各オブジェクトごとにユニークかつ共有不可能なデータ

### Flyweightと不変性

同じFlyweightオブジェクトを違ったコンテキストから使うので、その状態は変更できないようにする必要があります。
Flyweightは、コンストラクタのパラメータを介して一度だけ状態の初期化を行います。外部から`setter`や`public`フィールドにアクセスできないようにする必要がある。

### Flyweight Factory

様々なFlyweightオブジェクトへのアクセスをより便利にするために、既存のFlyweightオブジェクトのプールを管理するFactoryメソッドを作成することもできる。
このメソッドはクライアントから所望のFlyweightの内因的状態を受け取り、この状態に一致する既存のFlyweightオブジェクトを探し、見つかったらそれを返します。
見つからなかった場合は、新しいFlyweightオブジェクトを作成し、プールに追加します。


<div align="center">
<img src="./img/structure.png">
</div>


## 💻 Pseudo Code

```cpp
// Flyweightクラスはツリーの状態を一部含む。これらのフィールドには、
// それぞれのツリーに固有の値を格納する。例えば、ここにはツリーの
// 座標は見つからない。しかし多くのツリーの間で共有されている質感
// と色はここに含む。このデータは通常巨大であるため、各ツリーオブジェクト
// に保存するとメモリを大量に無駄にすることになる。その代わりに、
// 質感、色などの繰り返すデータを別個のオブジェクトに抽出し、多数の
// 各ツリーオブジェクトから参照することができる。
class TreeType is
    field name
    field color
    field texture
    field texture
    constructor TreeType(name, color, texture) { ... }
    method draw(canvas, x, y) is
        // 1. 指定の種類、色、質感のビットマップを作成
        // 2. キャンバス上、X/Yの座標にビットマップを作成

// Flyweightオブジェクトのファクトリは、既存のフライウェイトを再利用するか、
// 新しいオブジェクトを作成するかを決定。
class TreeFactory is
    static field treeTypes: collection of tree types
    static method getTreeType(name, color, texture) is
        type = treeTypes.find(name, color, texture)
        if (type == null)
            type = new TreeType(name, color, texture)
            treeTypes.add(type)
        return type;

// Contextオブジェクトにはツリー状態の外因的な部分が含まれる。
// これは2つの整数座標と1つの参照フィールドからできていて、とても小さいため、
// アプリケーションはこれを数重億個作成することも可能。
class Tree is
    field x, y
    field type: TreeType
    constructor Tree(x, y, type) { ... }
    method draw(canvas) is
        type.draw(canvas, this.x, this.y);

// `Tree`クラスと`Forest`クラスはFlyweightのクライアント。
// これ以上ツリークラスを開発する予定がなければ、一緒にしてもよい。
class Forest is
    field trees: collection of Trees

    method plantTree(x, y, name, color, texture) is
        type = TreeFactory.getTreeType(name, color, texture)
        tree = new Tree(x, y, type)
        trees.add(tree)

    method draw(canvas) is
        foreach(tree in trees) do
            tree.draw(canvas);
```