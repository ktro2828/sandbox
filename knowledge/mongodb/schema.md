# MongoDB Schema Design

- [MongoDB Schema Design Best Practices](https://www.mongodb.com/developer/products/mongodb/mongodb-schema-design-best-practices/)
- [6 Rules of Thumb for MongoDB Schema Design](https://www.mongodb.com/blog/post/6-rules-of-thumb-for-mongodb-schema-design)

## Schema Design Approaches - Relational vs. MongoDB

リレーショナルデータベース(RDBMS; Relational Database Management System)とMongoDBの比較は以下のようになる。
1. データのモデリング
   - RDBMS: データはテーブル、行、列の形式で保存される。データは関連性のあるテーブルに正規化され、外部キーによって関連付けられる。
   - MongoDB: ドキュメント指向データベースであり、BSON形式(=Binary JSON)でデータを保存する(表示の際にはJSON形式)。それぞれのドキュメントはキーと値のペアで構成され、データは階層的な構造を持つことができる。
2. スキーマの柔軟性
   - RDBMS: 事前に定義されたスキーマに従ってデータを保存する必要があり、スキーマに変更がある場合にはALTER文によって変更する必要がある。
   - MongoDB: スキーマの柔軟性が高く、ドキュメントは異なる構造を持つことができ、同じコレクション無いで様々なフィールドを持つことができる。
3. トランザクション処理
   - RDBMS: トランザクションの[ACID(Atomicity, Consistency, Isolation, Durability)特性](https://www.databricks.com/jp/glossary/acid-transactions)をサポートし、データの整合性と永続性を確保している。
   - MongoDB: トランザクションをサポートするが、RDBMSほど厳密なトランザクション処理が必要でない場合がある。MongoDBのトランザクションは、複数のドキュメントの操作を1つのトランザクション内でグループ化することができる。
4. スケーラビリティ
   - RDBMS: 水平方向のスケーラビリティが難しい場合があり、通常、垂直方向のスケーリングが主なアプローチ。
   - MongoDB: 水平方向のスケーラビリティに強く、シャーディングを使用してデータを分散させることができる。これにより、大規模なデータセットを効果的に処理できる。

## MongoDB Schema Design

### Embedding vs Referencing

MongoDBのスキーマデザインは大きく分けてEmbeddingとReferencingの2種類に分けられる。

#### Embedding

- Advantages
  - 1つのクエリから全ての関連する情報を得ることができる。
  - [`lookup`](https://docs.mongodb.com/manual/reference/operator/aggregation/lookup/?_ga=2.144400439.360715707.1712910410-1318381661.1711001114)を使ったりやアプリケーション内で`join`を実装することを避ける。
  - 関連する情報を一つのアトミック処理で更新できる。
    - [デフォルトでは1つのドキュメントに対する全CRUD処理はACIDに準拠する。](https://docs.mongodb.com/manual/core/read-isolation-consistency-recency/?_ga=2.144400439.360715707.1712910410-1318381661.1711001114)
  - 複数の処理をまたいだトランザクションが必要な場合は、[トランザクション演算子](https://docs.mongodb.com/master/core/transactions/?_ga=2.110767239.360715707.1712910410-1318381661.1711001114)を使うことができる。
- Limitations
  - ほとんどのフィールドが関連しない場合、大きなドキュメントではオーバーヘッドが増加する。
  - MongoDBでは1つのドキュメントサイズの上限は16MB。

#### Referencing

任意のドキュメントから別のドキュメントを参照する際にユニークなIDを使い、`lookup`でそれらを紐付ける。ReferencingはSQLクエリでのJOIN演算子と似ている。

- Advantages
  - 