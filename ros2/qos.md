# QoS

ROS 2では、QoSポリシーのセットを統合し、QoSプロファイルを形成することで、UDPのようなベストエフォートな通信からTCPのような通信までを表現できる。
ただし、様々なユースケースに対するただしQoSポリシーを選択することは難しいため、ROS 2では一般的なユースケース(e.g. センサーデータ)のための所定のQoSプロファイルがデフォルトで提供されている。

## QoSポリシー

### History

- **keep last** : 最大`N`サンプルまで保持する。`N`はキューの`Depth`オプションで指定可能。
- **keep all** : すべてのサンプルをDDSミドルウェアのリソース最大制限まで保持する。

### Depth

- **size of queue** : Historyオプションが**keep last**の場合、そのキューサイズを決定する。

### Reliability

- **Best effort** : サンプルを配信するが、通信が不安定な場合欠損する可能性がある。
- **Reliable** : サンプルが配信されることを保証する。このため、複数回リトライが発生する可能性がある。

### Durability

- **Transient local** : 遅いタイミングでsubscribeした相手に配信するために、publish時に最後のサンプルを保持しておく。
- **Volatile** : サンプルは保持されない。


## QoSプロファイル

上記のQoSポリシーを毎回設定するのは面倒なので、以下に示す特定のユースケースに対するQoSプロファイルが提供されている。
各ユースケースのQoSプロファイルの詳細は[ここを参照](https://github.com/ros2/rmw/blob/release-latest/rmw/include/rmw/qos_profiles.h)。
[(※各型の詳細)](https://github.com/ros2/rmw/blob/release-latest/rmw/include/rmw/types.h)

### Default

ROSからROS 2へ移行すには、同様のネットワーク動作で実行することが望ましく、ROSと同じような設定になっている。

- History : keep last
- Reliability : reliable
- Durability : volatile

### Service

再起動したサービスサーバーが再要求する期限切れの要求を受け取ることを避けるためDurabilityをvolatileにする必要がある。

- Reliability : reliable
- Durability : volatile

### Sensor data

センサーデータはほとんどの場合、すべてを確実に受信するのではなく、適切なタイミングで受信することが重要であり、開発者はサンプルが欠損するリスクを考慮しつつ、キャプチャされたらすぐに最新のサンプルを入手する必要がある。

- Reliability : best effort
- Depth : size of queueをできるだけ小さく

### Parameter

ROS 2のパラメータはサービスに基づいているため、サービスと同様のプロファイルを持つが、パラメータクライアントがパラメータサービスサーバにアクセスできなかった場合に要求が失われないよう、キューのdepthを大きく取る。

## QoSの互換性

Qosプロファイルはpublisherとsubscriber(=server/client)に対して個別に設定できるが、Request側(subscriber or client)の方Offer側よりも厳しいQoSポリシーを設定している場合には接続できない。

### QoS Durabilityの互換性

| Offer側         | Request側       | 接続               | 結果            | 備考              |
|:---------------:|:---------------:|:------------------:|:---------------:|:-----------------:|
| Volatile        | Volatile        | :heavy_check_mark: | Volatile        | 両者同一          |
| Volatile        | Transient local | :x:                | 無し            | Request側が厳しい |
| Transient local | Volatile        | :heavy_check_mark: | Volatile        | Offer側が厳しい   |
| Transient local | Transient local | :heavy_check_mark: | Transient local | 両者同一          |

## QoS Reliabilityの互換性

| Offer側     | Request側   | 接続               | 結果        | 備考              |
|:-----------:|:-----------:|:------------------:|:-----------:|:-----------------:|
| Best effort | Best effort | :heavy_check_mark: | Best effort | 両者同一          |
| Best effort | Reliable    | :x:                | 無し        | Request側が厳しい |
| Reliable    | Best effort | :heavy_check_mark: | Best effort | Offer側が厳しい   |
| Reliable    | Reliable    | :heavy_check_mark: | Reliable    | 両者同一          |
