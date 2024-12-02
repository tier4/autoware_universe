## autoware_ground_segmentation

## 目的

`autoware_ground_segmentation`は、入力された点群から地上の点を取り除くノードです。

## 仕組み / アルゴリズム

各地面セグメンテーションアルゴリズムの詳細な説明は、以下のリンクを参照してください。

| フィルター名          | 説明                                                                                                    | 詳細                                     |
| -------------------- | ------------------------------------------------------------------------------------------------------ | ------------------------------------------ |
| ray_ground_filter    | 放射上で直線上に並んだ点間の幾何学的関係に基づいて地面を除去する方法 | [リンク](docs/ray-ground-filter.md)      |
| scan_ground_filter   | `ray_ground_filter`とほぼ同じ方法だが、パフォーマンスをわずかに向上 | [リンク](docs/scan-ground-filter.md)     |
| ransac_ground_filter | 地面を平面に近似することで地面を除去する方法                                     | [リンク](docs/ransac-ground-filter.md) |

## 入出力

### 入力

| 名称              | 型                            | 説明       |
| ----------------- | ------------------------------- | ----------------- |
| `~/input/points`  | `sensor_msgs::msg::PointCloud2` | リファレンスポイント  |
| `~/input/indices` | `pcl_msgs::msg::Indices`        | リファレンスインデックス |

### 出力

このセクションでは、Autoware.Auto Autonomous Valet Parking Planning / Motion Planning フレームワークのアーキテクチャについて説明します。

アーキテクチャは、次のコンポーネントで構成されています。

* **Plan Evaluator:**
    * `post resampling`されたパスを評価し、点数を付けます。
    * 各パスには、スコアが関連付けられています。
* **Trajectory Optimizer:**
    * 特定の経路に対して、最適な制御入力を生成します。
    * 制御入力は、加速度と角速度として表されます。
* **Path Planning:**
    * 環境マップを使用して、パスを生成します。
    * パスの形状は、点と曲線で表現されます。
* **Smoother:**
    * パスを滑らかにし、車の動力学的制約を考慮に入れます。
* **Collision Checker:**
    * パスと環境との衝突を確認します。
    * 衝突が検出されると、パスは廃棄されます。
* **Map:**
    * 環境を表現します。
    * 地図には、道路、障害物、標識などの情報が含まれます。
* **Current Pose:**
    * 車の現在の位置と向きを表現します。
    * Current Poseは、Planning / Motion Planningの重要な入力です。

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | フィルタされた点群 |

## パラメータ

### ノードパラメータ

| 名前                  | タイプ   | デフォルト値 | 説明                               |
| --------------------- | ------ | ------------- | -------------------------------------- |
| `input_frame`          | 文字列 | " "           | 入力フレーム ID                      |
| `output_frame`         | 文字列 | " "           | 出力フレーム ID                     |
| `has_static_tf_only`   | ブール   | false         | TF を一度だけリッスンするフラグ      |
| `max_queue_size`       | 整数    | 5             | 入力/出力トピックの最大キューサイズ |
| `use_indices`          | ブール   | false         | ポイントクラウドのインデックスを使用 |
| `latched_indices`      | ブール   | false         | ポイントクラウドのインデックスをラッチ |
| `approximate_sync`     | ブール   | false         | 近似同期オプションを使用           |

## 仮定/既知の制限

`autoware::pointcloud_preprocessor::Filter` は [この問題](https://github.com/ros-perception/perception_pcl/issues/9) のため、pcl_perception [1] に基づいて実装されています。

## 参考文献/外部リンク

[1] <https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/filter.cpp>

