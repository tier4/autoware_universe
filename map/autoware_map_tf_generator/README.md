# autoware_map_tf_generator

## 目的

このパッケージ内のノードは、RViz内のマップを視覚化する `viewer` フレームをブロードキャストします。

`viewer` フレームを必要とするモジュールはなく、あくまで視覚化のために使用されることに注意してください。

`viewer` フレームの位置を計算するためにサポートされている方法は次のとおりです。

- `pcd_map_tf_generator_node` は、PCD内のすべてのポイントの幾何学的中心を出力します。
- `vector_map_tf_generator_node` は、pointレイヤー内のすべてのポイントの幾何学的中心を出力します。

## 内部動作/アルゴリズム

## 入出力

### 入力

#### autoware_pcd_map_tf_generator

| 名称                  | 種類                            | 説明                                                         |
| --------------------- | ------------------------------- | --------------------------------------------------------------- |
| `/map/pointcloud_map` | `sensor_msgs::msg::PointCloud2` | Pointcloud マップを購読して `viewer` フレームの位置を計算する |

#### autoware_vector_map_tf_generator

このノードは、AutowareのPlanningコンポーネントで使用されるTFトランスフォームを生成します。

**サブスクライブするトピック**

* `/current_pose` (`PoseStamped`)：自車位置
* `/lanelet_map_bin` (`Lanelet2MapBin`)：レイヤーレットマップ

**公開するトピック**

* `/tf` (`tf2_msgs/TFMessage`)：TFトランスフォーム
* `/current_lanelet` (`lanelet_msgs/LaneletMap`)：自車位置が含まれているレーンレット
* `/next_lanelet` (`lanelet_msgs/Lanelet`)：自車位置の次のレーンレット（存在する場合）
* `/vector_lanelet` (`lanelet_msgs/Lanelet`)：`post resampling`の後の自車位置が含まれているレーンレット

| 名称              | タイプ                                    | 説明                                                     |
| ----------------- | --------------------------------------- | --------------------------------------------------------- |
| `/map/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | `viewer` フレーム位置の計算にベクトルマップを使用する |

### 出力

このセクションでは、`post resampling` 高精度地図内で、現在のLoDで利用可能なHDマップ情報を格納し管理する高精度地図モジュールの設計について説明します。

`LocalPlanner`は、車両の自車位置と周囲の環境を考慮して、短期間の予測経路を生成します。

`Planning`コンポーネントは、高精度の道路情報を使用して、経路を安全かつ快適に誘導します。これには、経路上の停止線の検出、速度制限の遵守、障害物の回避が含まれます。

Autowareでは、環境感知コンポーネントは、周囲の環境の認識に使用される。これには、物体検出、道路境界検出、およびレーンマーキング検出が含まれます。

| 名前         | タイプ                     | 説明               |
| ------------ | ------------------------ | ------------------------- |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `viewer` フレームのブロードキャスト |

## パラメータ

### ノードパラメータ

なし

### コアパラメータ

{{ json_to_markdown("map/autoware_map_tf_generator/schema/map_tf_generator.schema.json") }}

## 前提条件/既知の制限

未確定。

