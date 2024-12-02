# autoware_pointcloud_preprocessor

## 目的

`autoware_pointcloud_preprocessor`は、次のフィルタを含むパッケージです。

- 異常点の除去
- クロッピング（切り抜き）
- 点群の連結
- 歪みの補正
- ダウンサンプリング

## 内部動作/アルゴリズム

各フィルタのアルゴリズムの詳細については、次のリンクを参照してください。

| フィルター名                   | 説明                                                                          | 詳細                                         |
| ----------------------------- | -------------------------------------------------------------------------------- | -------------------------------------------- |
| concatenate_data              | 複数のポイントクラウドをサブスクライブして、ポイントクラウドに連結する          | [リンク](docs/concatenate-data.md)              |
| crop_box_filter               | 1 回のスキャン中に自己車両の移動によって生じるポイントクラウドの歪みを補正する | [リンク](docs/crop-box-filter.md)               |
| distortion_corrector          | 1 回のスキャン中に自己車両の移動によって生じるポイントクラウドの歪みを補正する | [リンク](docs/distortion-corrector.md)          |
| downsample_filter             | 入力ポイントクラウドをダウンサンプリングする                                 | [リンク](docs/downsample-filter.md)             |
| outlier_filter                | ハードウェアの問題、雨滴、小さな虫によって生じるノイズとしてポイントを除去する | [リンク](docs/outlier-filter.md)                |
| passthrough_filter            | 与えられたフィールド（例：x、y、z、強度）の範囲外のポイントを除去する         | [リンク](docs/passthrough-filter.md)            |
| pointcloud_accumulator        | 一定時間分ポイントクラウドを累積する                                         | [リンク](docs/pointcloud-accumulator.md)        |
| vector_map_filter             | ベクトルマップを使用して、車線の外のポイントを除去する                       | [リンク](docs/vector-map-filter.md)             |
| vector_map_inside_area_filter | パラメータで与えられたタイプを持つベクトルマップ領域内のポイントを除去する | [リンク](docs/vector-map-inside-area-filter.md) |

## 入出力

### 入力

| 名前              | タイプ                            | 説明       |
| ----------------- | ------------------------------- | ----------------- |
| `~/input/points`  | `sensor_msgs::msg::PointCloud2` | 比較対象ポイント  |
| `~/input/indices` | `pcl_msgs::msg::Indices`        | 比較対象インデックス |

### 出力

Autoware.Autoを構築するためのドキュメントへようこそ。このドキュメントでは、Autoware.Autoの構成要素であるPlanningモジュールの概要を紹介します。

**Planningモジュール**

Planningモジュールは、自動運転システムの「脳」として機能します。周囲の環境を認識し、リアルタイムで安全で効率的な経路計画を行います。このモジュールは、自己位置推定、経路計画、経路最適化の3つの主要なサブコンポーネントで構成されています。

**自己位置推定**

このサブコンポーネントは、センサーデータを使用して自車位置を正確かつリアルタイムで推定します。

**経路計画**

このサブコンポーネントは、現在の位置から目的地までの安全で最適な経路を生成します。

**経路最適化**

このサブコンポーネントは、リアルタイムに経路を'post resampling'し、障害物や交通状況の変化に応じて経路を最適化します。

**Planningモジュールの責任**

* リアルタイムの経路計画
* 障害物回避
* 安全性マージンの維持
* 効率的な経路の生成
* 動的環境への適応

**依存関係**

Planningモジュールは、PerceptionモジュールとControlモジュールに依存しています。

* Perceptionモジュールは、 Planningモジュールに周囲の環境に関する情報を提供します。
* Controlモジュールは、Planningモジュールによって生成された経路を実装します。

**インターフェース**

Planningモジュールは、ROSトピックを通じて他のモジュールと通信します。主なトピックを次に示します。

* `/final_waypoints`：最終的な経路
* `/current_pose`：自車位置
* `/sensor_msgs/PointCloud2`：ポイントクラウドデータ

## 出力ポイント

| 名前              | データ型                            | 説明     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | フィルタされた点 |

## パラメーター

### ノードパラメーター

| 名称              | タイプ   | デフォルト値 | 説明                                                             |
| ----------------- | ------ | ------------- | ---------------------------------------------------------------- |
| input_frame      | 文字列 | " "           | 入力フレームID                                                       |
| output_frame     | 文字列 | " "           | 出力フレームID                                                      |
| max_queue_size   | 整数    | 5             | 入力/出力トピックの最大キューサイズ                                |
| use_indices      | ブール   | false         | ポイントクラウドインデックスを使用するフラグ                           |
| latched_indices  | ブール   | false         | ポイントクラウドインデックスをラッチするフラグ                          |
| approximate_sync | ブール   | false         | 近似同期オプションを使用するフラグ                                  |

## 想定事項 / 既知の制限事項

`autoware::pointcloud_preprocessor::Filter` は、[この問題](https://github.com/ros-perception/perception_pcl/issues/9)により、pcl_perception [1] に基づいて実装されています。

## パフォーマンスの測定

Autowareでは、各LiDARセンサーからの点群データは、知覚パイプラインに入力される前に、センシングパイプラインで前処理されます。前処理段階は、以下の図に示されています。

![pointcloud_preprocess_pipeline.drawio.png](docs%2Fimage%2Fpointcloud_preprocess_pipeline.drawio.png)

パイプライン内の各段階で処理遅延が発生します。ほとんどの場合、メッセージヘッダーと現在の時間間の時間を測定するために `ros2 topic delay /topic_name` を使用しています。このアプローチは、小サイズのメッセージには有効です。ただし、大規模な点群メッセージを処理する場合、この方法は追加の遅延を発生させます。これは主に、これらの大規模な点群メッセージに外部からアクセスすると、パイプラインのパフォーマンスに影響を与えるためです。

センシング/知覚ノードは、プロセス内通信を利用するコンポーザブルノードコンテナ内で動作するように設計されています。これらのメッセージに対する外部サブスクリプション（ros2 topic delay または rviz2 の使用など）は、追加の遅延を引き起こし、外部的にサブスクライブすることでパイプラインを低速化することさえあります。したがって、これらの測定結果は正確ではありません。

この問題を軽減するために、パイプライン内の各ノードがパイプラインの待ち時間データを報告する方法を採用しました。このアプローチにより、プロセス内通信の整合性が確保され、パイプライン内の遅延をより正確に測定できます。

### パイプラインのベンチマーク

パイプライン内のノードは、センサーのポイントクラウド出力からノードの出力までの時間を示す、パイプラインの待ち時間データを報告します。このデータは、パイプラインの健全性と効率性を評価するために不可欠です。

Autowareを実行すると、次のROS 2トピックをサブスクライブすることで、パイプライン内の各ノードのパイプラインの待ち時間データを監視できます。

- `/sensing/lidar/LidarX/crop_box_filter_self/debug/pipeline_latency_ms`
- `/sensing/lidar/LidarX/crop_box_filter_mirror/debug/pipeline_latency_ms`
- `/sensing/lidar/LidarX/distortion_corrector/debug/pipeline_latency_ms`
- `/sensing/lidar/LidarX/ring_outlier_filter/debug/pipeline_latency_ms`
- `/sensing/lidar/concatenate_data_synchronizer/debug/sensing/lidar/LidarX/pointcloud/pipeline_latency_ms`

これらのトピックは、パイプラインの待ち時間データを提供し、LidarXのセンサー出力から各後続ノードまでのパイプラインのさまざまな段階での遅延に関する洞察を与えます。

## (オプション) エラー検出および処理

## (オプション) パフォーマンスの特性評価

## 参照/外部リンク

[1] <https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/filter.cpp>

## (オプション) 将来の拡張/未実装部分

