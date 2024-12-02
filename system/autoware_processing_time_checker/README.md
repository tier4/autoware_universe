# 処理時間チェッカー

## 目的

このノードは、各モジュールの処理時間が有効かどうかを確認し、診断を送信します。
注意：現在、検証機能はなく、診断では常に「OK」が割り当てられます。

### スタンドアロン起動


```bash
ros2 launch autoware_processing_time_checker processing_time_checker.launch.xml
```

## 内部の仕組み / アルゴリズム

## 入出力

### 入力

| 名前                    | タイプ                           | 説明                               |
| ------------------------- | -------------------------------- | -------------------------------- |
| `/.../processing_time_ms` | `tier4_debug_msgs/Float64Stamped` | 各モジュールの処理時間             |

### 出力

**Autoware自動運転ソフトウェア**

**目次**

- [アーキテクチャ](#アーキテクチャ)
- [Planning](#Planning)
- [Perception](#Perception)
- [Visualization](#Visualization)
- [Reference Documents](#Reference-Documents)

**アーキテクチャ**

Autowareは、以下のモジュールから構成されるモジュール式のソフトウェアスタックです。

- Controller
- Localization
- Planning
- Perception
- Vehicle Control
- Visualization

**Planning**

Planningモジュールは、自車位置とセンサデータに基づいて、車両の経路を計画します。以下のタスクを実行します。

- 障害物回避
- 経路追従
- 交通ルールへの準拠

**Perception**

Perceptionモジュールは、センサデータから周囲環境を認識します。以下のタスクを実行します。

- 物体検出
- レーン検出
- 道路境界検出

**Visualization**

Visualizationモジュールは、PlanningとPerceptionの出力データを視覚化します。これにより、開発者はシステムのパフォーマンスを監視できます。

**Reference Documents**

- [Autowareユーザーガイド](https://www.autoware.ai/user-guides/)
- [Autoware開発者ガイド](https://www.autoware.ai/developer-guides/)
- [Autoware APIドキュメント](https://www.autoware.ai/api-docs/)

| 名称                                        | タイプ                                  | 説明                        |
| ----------------------------------------- | ------------------------------------- | ---------------------------------- |
| `/system/processing_time_checker/metrics` | `tier4_metric_msgs::msg::MetricArray` | すべてのモジュールの処理時間 |

## パラメータ

{{ json_to_markdown("system/autoware_processing_time_checker/schema/processing_time_checker.schema.json") }}

`output_metrics = true` の場合、ノードはシャットダウン時にその有効期間中に測定された処理時間の統計を `<ros2_logging_directory>/autoware_metrics/<ノード名>-<タイムスタンプ>.json` に書き出します。

## 前提 / 制約事項

未定。

