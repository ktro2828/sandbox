# Abstract Factory

- [Abstract Factory](https://refactoring.guru/design-patterns/abstract-factory)

## :memo: Intent
Abstract Factoryは、生成に関するデザインパターンの一つで、関連したオブジェクトの集まりを具象クラスを指定すること無く生成すること可能にします。

## :confused: Problem
家具工場シミュレータを作成するとします。コードは以下のようなものを実現するクラスで構成されています:

1. 関連する製品ファミリー。例えば: `Chair` + `Sofa` + `CoffeeTable`。
2. このファミリーの様式。例えば、`Char` + `Sofa` + `CoffeeTable`の様式には以下のようなものがあります: `Modern`、`Victorian`、`ArtDeco`。

<div align="center">
<img src="./img/problem.png">
<br>
<em>製品群と様式</em>
</div>

様式の一致する家具を製造するには、同じ様式の家具オブジェクトを生成する何らかの法則が必要となります。様式の異なる家具が配達されたら、客は怒りますよね。

<div align="center">
<img src="./img/abstract-factory-comic-1.png">
<br>
<em>現代風ソファーは、ビクトリア様式の椅子と合わない！</em>
</div>

プログラムに新しい製品や製品群を追加するとき、既存のコードの変更はしたくありません。
家具メーカーはカタログを常に更新しますが、そのたびに中核のコードを変更するのは避けたいところです。

## :smile: 解決策
Abstract Factoryパターンでは、まず、家具ファミリーの個別製品(椅子、ソファー、コーヒーテーブル)ごとに、明示的にインターフェイスを宣言します。様式別の製品は、これらのインターフェイスに従って作成します。例えば、椅子の異なる様式に対応する変種(バリエーション)を作るには、`Chair`インターフェイスの実装を行い、コーヒーテーブルの変種を作るには、`CoffeeTable`のインターフェイスを実装するといった具合です。

<div align="center">
<img src="./img/solution1.png">
<br>
<em>同じオブジェクトの異種は全部、単一クラス階層に移動</em>
</div>

次に、**抽象ファクトリー**を宣言します。これは、製品ファミリー全製品の生成メソッド(例: `createChair`、`createSofa`、`createCoffeeTable`)を並べたインターフェイスとなります。
これらのメソッドは、このように抽出した`Chair`、`Sofa`、`CoffeeTable`といったインターフェイスで表現された、製品の**抽象**型を返す必要があります。

<div align="center">
<img src="./img/solution2.png">
<br>
<em>個々の具象ファクトリークラスは、製品の様式に対応する</em>
</div>

では製品の様式ごとの変種はどうするか？というと、製品の様式ごとに、`AbstractFactory`を基に別々のファクトリークラスを作成します。ファクトリーは、特定の種類の製品を返すクラスです。例えば、`ModernFurnitureFactory`は、`ModernChair`、`ModernSofa`、`ModernCoffeeTable`のオブジェクトのみを作ることができます。

```c++
// -- Product --
// Abstract Product for Chair
class Chair {
public:
  virtual bool hasLegs() const = 0;
  virtual std::string sitOn() const = 0;
};

// Concrete Products for Chair
class ModernChar : public Chair {
public:
  bool hasLegs() const override { return false; }
  std::string sitOn() const override { return "{Result of sitting on ModernChair}"; }
};
class VictoriaChair : public Chair {
public:
  bool hasLegs() const override { return true; }
  std::string sitOn() const override { return "{Result of sitting on VictoriaChair}"; }
};
class ArtDecoChair : public Chair {
public:
  bool hasLegs() const override { return true; }
  std::string sitOn() const override { return "{Result of sitting on ArtDecoChair}"; }
};

// Concrete Products for Sofa and CoffeeTable of a particular kind.
// ...

// -- Factory --
// Abstract Factory
class FurnitureFactory {
public:
  virtual Chair * createChair() const = 0;
  virtual Sofa * createSofa() const = 0;
  virtual CoffeeTable * createCoffeeTable() const = 0;
};

// Concrete Factories
class ModernFurnitureFactory : public FurnitureFactory
{
public:
  Chair * createChair() const override { return new ModernChair(); }
  Sofa * createSofa() const override { return new ModernSofa(); }
  CoffeeTable * createCoffeeTable() const override { return new ModernCoffeeTable(); }
};

// Concrete Factories for Victoria and ArtDeco of a particular kind.
// ...
```

クライアント側のコードは、ファクトリー、製品ともそれぞれの抽象インターフェイスを通して機能します。これにより、クライアントのコードに渡すファクトリーの型を変更したり、クライアントの受け取る製品の様式を変更しても、クライアント側コードは問題なく動作します。

<div align="center">
<img src="./img/abstract-factory-comic-2.png">
<br>
<em>クライアントは、ファクトリーの実際のクラスが何かを気にする必要はない</em>
</div>

クライアントはファクトリーに椅子を生成して欲しいとします。クライアントは、ファクトリーのクラスを気にする必要もなく、どういう種類の椅子が返ってくるかを気にする必要もありません。

もう一つ明らかにしておくべきことがあります。クライアントが抽象インターフェイスだけに依存しているとすると、実際のファクトリーオブジェクトは何によって作成されるのでしょうか？通常、アプリケーションは、初期化段階でファクトリーオブジェクトを一つ生成します。その少し前に、アプリは構成や環境設定を使用してファクトリーの型を選択します。

## :computer: Structure

<div align="center">
<img src="./img/structure.png">
</div>

## :bulb: Applicability

:lady_beetle: 関連する製品の集まりである様々な変種に対応したいが、製品の具象クラス(それは設計段階では未知かもしれません)に依存させたくない場合に、Abstract factoryを使用します。あるいは、単に将来の拡張に備えるために使用することもできます。

:zap: Abstract Factoryでは、製品ファミリーの各クラスのオブジェクトを作成するインターフェイスを使用します。このインターフェイスによってオブジェクトを生成する限り、すでにアプリで作成した製品と合わない間違った変種の製品が作成されることを心配する必要はありません。

---

:lady_beetle: 一連のファクトリー・メソッドからなるクラスがあり、それが一次責任を曖昧にしてしまう場合に、Abstract Factoryの使用を検討してください。

:zap: よく設計されたプログラムでは、**それぞれのクラスは単一の事項についてのみ責任を持ちます。**ある一つのクラスが複数の製品型を扱う場合、ファクトリー・メソッドを抽出して独立したファクトリークラスを作るか、完全なAbstract Factoryの実装を行うことをお勧めします。

## :anchor: Pros and Cons

:white_check_mark: ファクトリーから得られる製品同士は、互換であることが保証される。
:white_check_mark: 具象製品とクライアント側コードの密結合を防止できる。
:white_check_mark: 単一責任の原則(*Single Responsibility Principle*)。製品作成のコードが一箇所にまとめられ、保守が容易になる。
:white_check_mark: 開放閉鎖の原則(*Open/Closed Principle*)。製品の新しい変種を導入しても、既存クライアント側コードは動作する。

:x: パターンの使用に伴い、多数の新規インターフェースやクラスが導入され、コードが必要以上に複雑になる可能性あり。

## :arrows_counterclockwise: Relations with Other Patters

- 多くの設計は、まず比較的単純でサブクラスによりカスタマイズ可能な、**[Factory Method](../factory-method/README.md)**から始まり、次第に、もっと柔軟だが複雑な**Abstract Factory**や**[Prototype](../prototype/README.md)**や**[Builder](../builder/README.md)**へと発展していきます。
- **Builder**は、複雑なオブジェクトを段階的に構築することに重点をおいています。**Abstract Factory**は、関連するオブジェクトの集団を作成することに特化しています。*Abstract Factory*がすぐにプロダクトを返すのに対して、*Builder*ではプロダクトの取得前に、いくつかの追加の構築ステップを踏まなければなりません。
- **Abstract Factory**クラスは、多くの場合**Factory Method**の集まりですが、**Prototype**を使ってメソッドを書くこともできます。
- サブシステムがオブジェクトを作成する方法をクライアントから隠蔽することだけが目的なら、**Abstract Factory**を**[Facade](../../structual/facade/README.md)**の代わりに使えます。
- **Abstract Factory**は、**[Bridge](../../structual/bridge/README.md)**と一緒に使用できます。この組み合わせは、*Bridge*によって定義された抽象化層のいくつかが特定の実装としか動作できない場合に便利です。この場合、*Abstract Factory*はこれらの関係をカプセル化し、クライアントコードから複雑さを隠すことができます。
- **Abstract Factory**、**Builder**、**Prototype**はどれも**[Singleton](../singleton/README.md)**で実装可能です。

