# autoware_simple_object_merger

このパッケージは、[autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.msg) の複数のトピックを、低計算コストでマージできます。

## 設計

### 背景

[Object_merger](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/object_merger) は主に DetectedObjects マージ処理に使用されます。`Object_merger` には 2 つの特性があります。第 1 に、`object_merger` はハンガリー法などのデータアソシエーションアルゴリズムを使用してマッチング問題を解決しますが、計算コストが必要です。第 2 に、`object_merger` は 2 つの DetectedObjects トピックのみを処理でき、1 つのノードで 2 つ以上のトピックを処理することはできません。現在、6 つの DetectedObjects トピックをマージするには、6 つの `object_merger` ノードが必要です。

そのため、`autoware_simple_object_merger` は、複数の DetectedObjects を低計算コストでマージすることを目的としています。
このパッケージはデータアソシエーションアルゴリズムを使用しないため、計算コストを削減し、多数のノードが起動するのを防ぐために、1 つのノードで 2 つ以上のトピックを処理できます。

### ユースケース

- 複数のレーダー検出

`autoware_simple_object_merger` は、複数のレーダー検出に使用できます。複数のレーダーのトピックから 1 つのトピックにまとめることで、レーダーによる遠距離検出のパイプラインを簡略化できます。

### 制限事項

- センサーデータの欠損と遅延

マージされたオブジェクトは、初期化時にすべてのトピックデータが受信されるまで発行されません。さらに、センサーデータの欠損と遅延に対処するために、このパッケージにはタイムアウトを判定するパラメータがあります。トピックのデータの最新の時間が、タイムアウトパラメータより古くなると、出力オブジェクトとマージされません。現時点では、このパッケージの仕様では、すべてのトピックデータが最初に受信され、その後データが欠損し、タイムアウトとして判断されたオブジェクトのないマージされたオブジェクトが発行されます。このタイムアウトパラメータは、センサーサイクル時間によって決定する必要があります。

- 後処理

このパッケージにはマッチング処理がないため、入力オブジェクトによっては重複するオブジェクトが発生します。そのため、後処理を使用する場合にのみ出力オブジェクトを使用できます。現時点では、[クラスタリング処理](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_radar_object_clustering) を後処理として使用できます。

## インターフェイス

### 入力

入力トピックは、`input_topics`（List[文字列]）のパラメータによって定義されます。入力トピックのタイプは `std::vector<autoware_perception_msgs/msg/DetectedObjects.msg>` です。

### 出力

- `~/output/objects`（`autoware_perception_msgs/msg/DetectedObjects.msg`）
  - 入力トピックから結合されたマージされたオブジェクト。

### パラメータ

- `update_rate_hz`（double）[Hz]
  - デフォルトパラメータ: 20.0

このパラメータは `onTimer` 関数の更新レートです。
このパラメータは入力トピックのフレームレートと同じにする必要があります。

- `new_frame_id`（文字列）
  - デフォルトパラメータ: "base_link"

このパラメータは、出力トピックのヘッダーフレーム ID です。
出力トピックを Perception モジュールに使用する場合、"base_link" に設定する必要があります。

- `timeout_threshold`（double）[s]
  - デフォルトパラメータ: 0.1

このパラメータは、タイムアウトの判断のためのしきい値です。
`input_topics` の最初のトピックと入力トピックとの時間差がこのパラメータを超過すると、トピックのオブジェクトは出力オブジェクトにマージされません。


```cpp
  for (size_t i = 0; i < input_topic_size; i++) {
    double time_diff = rclcpp::Time(objects_data_.at(i)->header.stamp).seconds() -
                       rclcpp::Time(objects_data_.at(0)->header.stamp).seconds();
    if (std::abs(time_diff) < node_param_.timeout_threshold) {
      // merge objects
    }
  }
```

- `input_topics` (List[string])
  - 初期設定パラメータ: "[]"

このパラメータは入力トピックの名前です。
たとえば、このパッケージがレーダーオブジェクトに使用されるとき、


```yaml
input_topics:
  [
    "/sensing/radar/front_center/detected_objects",
    "/sensing/radar/front_left/detected_objects",
    "/sensing/radar/rear_left/detected_objects",
    "/sensing/radar/rear_center/detected_objects",
    "/sensing/radar/rear_right/detected_objects",
    "/sensing/radar/front_right/detected_objects",
  ]
```

config yaml ファイルで設定できます。
現時点では、時間差は `input_topics` の最初のトピックと入力トピック間のヘッダ時刻によって計算されるため、検出する最も重要なオブジェクトは `input_topics` リストの最初に設定する必要があります。

