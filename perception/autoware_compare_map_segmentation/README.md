- Input pointcloud (`PointXYZIF` or `PointCloud2` type)
- Elevation map (`Image` type, 8bit PNG format)
- `height_diff_thresh` (float, default: 0.2)

#### Output
- Ground filtered (`PointXYZIF`, `PointCloud2` type)

### Distance Based Compare Map Filter

#### Input
- Input pointcloud (`PointXYZIF` or `PointCloud2` type)
- Map pointcloud (`PointCloud2` type)
- `search_radius` (float; default: 1.0)

#### Output
- Ground filtered (`PointXYZIF`, `PointCloud2` type)

### Voxel Based Approximate Compare Map Filter

#### Input
- Input pointcloud (`PointXYZIF` or `PointCloud2` type)
- Map pointcloud (`PointCloud2` type)
- `downsample_resolution` (float; default: 0.1)

#### Output
- Ground filtered (`PointXYZIF`, `PointCloud2` type)

### Voxel Based Compare Map Filter

#### Input
- Input pointcloud (`PointXYZIF` or `PointCloud2` type)
- Map pointcloud (`PointCloud2` type)
- `voxel_resolution` (float; default: 0.2)
- `max_distance_in_voxel` (float; default: 0.2)

#### Output
- Ground filtered (`PointXYZIF`, `PointCloud2` type)

### Voxel Distance based Compare Map Filter

#### Input
- Input pointcloud (`PointXYZIF` or `PointCloud2` type)
- Map pointcloud (`PointCloud2` type)
- `voxel_resolution` (float; default: 0.1)
- `max_distance_in_voxel` (float; default: 0.2)
- `search_radius` (float; default: 1.0)

#### Output
- Ground filtered (`PointXYZIF`, `PointCloud2` type)

## Usage

### Compare Elevation Map Filter

The filter can be used by adding the following lines to the launch file:

```xml
<node pkg="autoware_compare_map_segmentation" type="compare_elevation_map_filter" name="compare_elevation_map_filter">
  <remap from="/input" to="points_raw" />
  <remap from="/output" to="points_ground" />
  <rosparam>height_diff_thresh: 0.2</rosparam>
</node>
```

### Distance Based Compare Map Filter

The filter can be used by adding the following lines to the launch file:

```xml
<node pkg="autoware_compare_map_segmentation" type="distance_based_compare_map_filter" name="distance_based_compare_map_filter">
  <remap from="/input" to="points_raw" />
  <remap from="/output" to="points_ground" />
  <remap from="/map_points" to="points_map" />
  <rosparam>search_radius: 1.0</rosparam>
</node>
```

### Voxel Based Approximate Compare Map Filter

The filter can be used by adding the following lines to the launch file:

```xml
<node pkg="autoware_compare_map_segmentation" type="voxel_based_approximate_compare_map_filter" name="voxel_based_approximate_compare_map_filter">
  <remap from="/input" to="points_raw" />
  <remap from="/output" to="points_ground" />
  <remap from="/map_points" to="points_map" />
  <rosparam>downsample_resolution: 0.1</rosparam>
</node>
```

### Voxel Based Compare Map Filter

The filter can be used by adding the following lines to the launch file:

```xml
<node pkg="autoware_compare_map_segmentation" type="voxel_based_compare_map_filter" name="voxel_based_compare_map_filter">
  <remap from="/input" to="points_raw" />
  <remap from="/output" to="points_ground" />
  <remap from="/map_points" to="points_map" />
  <rosparam>voxel_resolution: 0.2</rosparam>
  <rosparam>max_distance_in_voxel: 0.2</rosparam>
</node>
```

### Voxel Distance based Compare Map Filter

The filter can be used by adding the following lines to the launch file:

```xml
<node pkg="autoware_compare_map_segmentation" type="voxel_distance_based_compare_map_filter" name="voxel_distance_based_compare_map_filter">
  <remap from="/input" to="points_raw" />
  <remap from="/output" to="points_ground" />
  <remap from="/map_points" to="points_map" />
  <rosparam>voxel_resolution: 0.1</rosparam>
  <rosparam>max_distance_in_voxel: 0.2</rosparam>
  <rosparam>search_radius: 1.0</rosparam>
</node>
```

## Parameters

### Compare Elevation Map Filter

- `height_diff_thresh` (float; default: 0.2): The height difference threshold for filtering points.

### Distance Based Compare Map Filter

- `search_radius` (float; default: 1.0): The radius used to search for neighboring points in the map pointcloud.

### Voxel Based Approximate Compare Map Filter

- `downsample_resolution` (float; default: 0.1): The resolution used to downsample the map pointcloud.

### Voxel Based Compare Map Filter

- `voxel_resolution` (float; default: 0.2): The resolution used to create the voxel grid of the map pointcloud.
- `max_distance_in_voxel` (float; default: 0.2): The maximum distance between a point and the center of a voxel to be considered as part of the ground.

### Voxel Distance based Compare Map Filter

- `voxel_resolution` (float; default: 0.1): The resolution used to create the voxel grid of the map pointcloud.
- `max_distance_in_voxel` (float; default: 0.2): The maximum distance between a point and the center of a voxel to be considered as part of the ground.
- `search_radius` (float; default: 1.0): The radius used to search for neighboring points in the map pointcloud for points that do not belong to any voxel grid.

| 名前                    | タイプ                            | 説明      |
| ----------------------- | ------------------------------- | ---------- |
| `~/input/points`        | `sensor_msgs::msg::PointCloud2` | 基準点 |
| `~/input/elevation_map` | `grid_map::msg::GridMap`        | 標高マップ    |

## 自動運転ソフトウェアのドキュメント

#### 概要

このドキュメントでは、Autoware向け自動運転ソフトウェアの概要、システムアーキテクチャ、コンポーネント、モジュールについて説明します。

#### システムアーキテクチャ

Autowareシステムアーキテクチャは、以下の中核モジュールで構成されています。

- **Perception**（認識）: センサーからのデータを処理し、周囲の環境を認識します。
- **Planning**（計画）: 車両の経路と操作を計画します。
- **Control**（制御）: 計画に従って車両を制御します。

#### コンポーネントとモジュール

**Perception**

- **Sensor Framework**（センサーフレームワーク）: センサーデータの取り込み、処理、キャリブレーションを行います。
- **Obstacle Detection**（障害物検出）: レーザーレーダー、カメラ、レーダーデータから障害物を検出します。
- **Lane Detection**（車線検出）: 道路上の車線を検出します。

**Planning**

- **Path Planning**（経路計画）: 目的地までの経路を計画します。
- **Behavior Planning**（行動計画）: 経路上の障害物やその他の車両を回避するための操作を計画します。
- **Trajectory Planning**（軌道計画）: 車両の軌道と速度を計画します。

**Control**

- **Longitudinal Control**（縦方向制御）: ブレーキとアクセルを制御し、速度と加速度を維持します。
- **Lateral Control**（横方向制御）: ステアリングを制御し、車両を目標軌道上に維持します。

#### データ処理パイプライン

Autowareのデータ処理パイプラインは以下のように構成されています。

1. センサーデータの取り込みとキャリブレーション
2. 障害物、車線、その他のオブジェクトの認識
3. 目的地と経路の計画
4. 操作の計画（操舵角、加減速）
5. 車両制御（ステアリング、ブレーキ、アクセル）

#### `post resampling`

`post resampling`は、Perceptionパイプラインの一部分であり、認識された障害物とオブジェクトの座標をリサンプリングして、現在の姿勢に一致させます。

#### 関連資料

- [Autowareドキュメント](https://autoware.org/documentation/)
- [Autoware GitHubレポジトリ](https://github.com/autowarefoundation/autoware.ai)

| 名称                   | 種類                                         | 説明                   |
| ---------------------- | ----------------------------------------- | ----------------------- |
| `~/output/points`       | `sensor_msgs::msg::PointCloud2`         | フィルタ済み点群 |

#### パラメータ

| 名称                 | 型   | 説明                                                                         | デフォルト値 |
| :------------------- | :----- | :------------------------------------------------------------------------------- | :------------ |
| `map_layer_name`     | 文字列 | 標高マップレイヤーの名称                                                      | elevation     |
| `map_frame`          | float  | 標高マップがサブスクライブされる前に一時的に使用されるマップのフレーム ID | map           |
| `height_diff_thresh` | float  | 高さの違いがこの値 [m] を下回っているポイントを削除                         | 0.15          |

### その他のフィルタ

#### 入力

| 名前                                | 種類                                  | 説明                                                  |
| ------------------------------------ | ------------------------------------ | ---------------------------------------------------- |
| `~/input/points`                   | `sensor_msgs::msg::PointCloud2`     | レファレンスポイント                                |
| `~/input/map`                     | `sensor_msgs::msg::PointCloud2`     | マップ（静的マップローディングの場合）               |
| `/localization/kinematic_state`     | `nav_msgs::msg::Odometry`         | 自車位置（動的マップローディングの場合）           |

#### 出力

**自動運転ソフトウェア**

このドキュメントでは、Autoware自動運転ソフトウェアの設計とアーキテクチャについて説明します。

### 概要

Autowareは、自動運転車のためのオープンソースソフトウェアスタックです。モジュール式のアーキテクチャにより、開発者はアプリケーション固有の機能を簡単に統合できます。

### アーキテクチャ

Autowareのアーキテクチャは、次のコンポーネントで構成されます。

- **Planning**モジュール：経路の生成と通行不可能領域の回避を担当します。
- **Perception**モジュール：センサーデータから周囲環境を認識します。
- **Localization**モジュール：自車位置を決定します。
- **Control**モジュール：車両の制御を担当します。

### Planningコンポーネント

Planningコンポーネントは、次のサブモジュールで構成されます。

- **Path Planning**：経路の生成を担当します。
- **Motion Planning**：車両の動作の生成を担当します。

### Perceptionコンポーネント

Perceptionコンポーネントは、次のサブモジュールで構成されます。

- **Camera**：カメラデータからオブジェクトを検出します。
- **Lidar**：LiDARデータから点群を生成します。
- **Radar**：レーダーデータからオブジェクトを検出します。

### Localizationコンポーネント

Localizationコンポーネントは、次のサブモジュールで構成されます。

- **GPS**：衛星データから自車位置を推定します。
- **IMU**：慣性データから自車位置を推定します。

### Controlコンポーネント

Controlコンポーネントは、次のサブモジュールで構成されます。

- **Vehicle Control**：車両のステアリング、ブレーキ、アクセルを制御します。
- **Motion Control**：車両の速度と加速度を制御します。

### データフロー

Autowareのデータフローは次のとおりです。

1. Perceptionコンポーネントは、センサーデータから周囲環境を認識します。
2. Localizationコンポーネントは、自車位置を決定します。
3. Planningコンポーネントは、経路を生成して車両の動作を生成します。
4. Controlコンポーネントは、車両を制御してPlanningコンポーネントの要求を実現します。

### 応用例

Autowareは、次のような幅広い応用事例で使用できます。

- **自動運転車**
- **自律移動ロボット**
- **ドローン**
- **AGV**

| 名前              | タイプ                            | 説明     |
| ----------------- | ------------------------------- | ----------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | フィルタリング済み点群 |

#### パラメータ

| 名前 | タイプ | 説明 | デフォルト値 |
| :-------------------------- | :----- | :--------------------------------------------------------------------------------------------------------------------- | :------------ |
| `use_dynamic_map_loading` | bool | マップの読み込みモード選択、`true` は動的マップの読み込み、`false` は静的マップの読み込み、分割なしのマップポイントクラウドに推奨 | true |
| `distance_threshold` | float | 入力ポイントとマップポイントを比較するための距離閾値 [m] | 0.5 |
| `map_update_distance_threshold` | float | マップの更新が必要になる車両の移動距離の閾値 (動的マップの読み込みで) [m] | 10.0 |
| `map_loader_radius` | float | 読み込む必要のあるマップの半径 (動的マップの読み込みで) [m] | 150.0 |
| `timer_interval_ms` | int | マップの更新が必要かをチェックするためのタイマー間隔 (動的マップの読み込みで) [ms] | 100 |
| `publish_debug_pcd` | bool | デバッグ用の `debug/downsampled_map/pointcloud` にボクセル化された更新マップを公開するかどうか。追加の計算コストが発生する可能性あり | false |
| `downsize_ratio_z_axis` | double | z 軸で `voxel_leaf_size` と近傍点距離閾値を減らす正の比率 | 0.5 |

## Assumptions / Known limits

## (Optional) エラー検出と処理

## (Optional) パフォーマンスの特性評価

## (Optional) リファレンス/外部リンク

## (Optional) Future extensions / Unimplemented parts

