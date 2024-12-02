# Planning Evaluator

## 目的

このパッケージは、PlanningとControlの品質を評価するための指標を生成するノードを提供します。

## 内部の動き / アルゴリズム

評価ノードは、トラジェクトリ `T(0)` を受信するたびに指標を計算します。
指標は、次の情報を使用して計算されます。

- トラジェクトリ `T(0)` 自体
- 前のトラジェクトリ `T(-1)`
- `T(0)` を計画するために使用される基準と想定される_基準_トラジェクトリ
- 自車位置
- 環境内のオブジェクトのセット

これらの情報は、`MetricsCalculator` クラスのインスタンスによって維持され、指標の計算もこのインスタンスによって担当されます。

### 統計量

各指標は、指標に対して計算された最小値、最大値、平均値、測定された値の数を格納する `autoware::universe_utils::Accumulator` インスタンスを使用して計算されます。

### 指標の計算と新しい指標の追加

可能な指標はすべて、`include/planning_evaluator/metrics/metric.hpp` で定義された `Metric` 列挙体内に定義されています。
このファイルは、文字列との相互変換や、出力ファイルのヘッダーとして使用される人間が読める説明も定義します。

`MetricsCalculator` は、関数の呼び出しを通じて指標の統計量を計算します。


```C++
Accumulator<double> MetricsCalculator::calculate(const Metric metric, const Trajectory & traj) const;
```

新しいメトリクス `M` を追加するには、次の手順が必要です。

- `metrics/metric.hpp`: `enum`、文字列変換マップ、および説明マップに `M` を追加します。
- `metrics_calculator.cpp`: `calculate` 関数の `switch/case` ステートメントに `M` を追加します。
- `selected_metrics` パラメーターに `M` を追加します。

## 入出力

### 入力

| 名前                   | タイプ                                         | 説明                                       |
| --------------------- | --------------------------------------------- | ------------------------------------------------- |
| `~/input/trajectory`    | `autoware_planning_msgs::msg::Trajectory`    | 評価するメインの走行軌跡                       |
| `~/input/reference_trajectory` | `autoware_planning_msgs::msg::Trajectory`    | 偏差の測定値に使用される基準走行軌跡 |
| `~/input/objects`       | `autoware_perception_msgs::msg::PredictedObjects` | 障害物                                         |

### 出力

各指標は、指標名にちなんだトピックで公開されます。

| 名前        | 型                                  | 説明                                                       |
| ----------- | ------------------------------------- | ----------------------------------------------------------------- |
| `~/metrics` | `tier4_metric_msgs::msg::MetricArray` | `tier4_metric_msgs::msg::Metric`のメトリックを多数持つMetricArray |

`output_metrics = true` の場合、評価ノードはシャットダウン時に、寿命中に測定されたメトリクスの統計データを `<ros2_logging_directory>/autoware_metrics/<node_name>-<time_stamp>.json` に書き込みます。

## パラメータ

{{ json_to_markdown("evaluator/autoware_planning_evaluator/schema/autoware_planning_evaluator.schema.json") }}

## 仮定 / 既知の制限

次のことを前提としています : Planningモジュールがトラектория `T(0)` を受信するとき、そのPlanningモジュールは最後に受信したリファレンストラекトリーとオブジェクトを使用して生成されている。`T(0)` の計算中に新しいリファレンストラекトリーまたはオブジェクトが公開された場合、これは誤りになる可能性があります。

精度は現時点ではトラекトリーの分解能によって制限されています。トラекトリーとリファレンストラекトリーの補間により、精度を向上させることは可能ですが、計算が大幅に負荷のかかるものになります。

## 今後の拡張 / 未実装部分

- リファレンストラекトリーとして `Route` または `Path` メッセージを使用します。
- RSS メトリクス（別のノード <https://tier4.atlassian.net/browse/AJD-263> で完了）。
- `min` と `max` の各メトリクス値を公開するためのオプションを追加します。現時点では `mean` の値のみが公開されています。
- `motion_evaluator_node`
  - 自己位置の実際の動きから時系列的にトラекトリーを構築するノード
  - 現時点では概念実証のみが実装されています。

