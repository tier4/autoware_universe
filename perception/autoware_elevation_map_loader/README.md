# autoware_elevation_map_loader

## 目的

このパッケージはautoware_compare_map_segmentation用の標高マップを提供します。

## 内部動作/アルゴリズム

サブスクライブされたpointcloud_mapとvector_mapから標高マップを生成し、それをパブリッシュします。
生成された標高マップをローカルに保存し、次回からロードします。

各セルの標高値は、最も低いクラスターの点のzの平均値です。
標高値のないセルは、近隣のセルの値を使用してペイントインできます。

<p align="center">
  <img src="./media/elevation_map.png" width="1500">
</p>

## 入出力

### 入力

| 名前                          | 型                                          | 説明                                    |
| ----------------------------- | -------------------------------------------- | ---------------------------------------- |
| `input/pointcloud_map`        | `sensor_msgs::msg::PointCloud2`             | 点群マップ                               |
| `input/vector_map`            | `autoware_map_msgs::msg::LaneletMapBin`      | （オプション）lanelet2 マップのバイナリデータ |
| `input/pointcloud_map_metadata` | `autoware_map_msgs::msg::PointCloudMapMetaData` | （オプション）点群マップのメタデータ     |

### 出力

**自動運転ソフトウェア・アーキテクチャ**

このドキュメントでは、Autowareの自動運転ソフトウェアアーキテクチャの概要を示します。

**コンポーネント**

Autowareは、以下を含むさまざまなコンポーネントで構成されています。

* **Perception (認識)**: センサーデータからの障害物の検出と分類
* **Localization (定位)**: 自車位置の推定
* **Planning (経路計画)**: 自動運転の方針の生成
* **Control (制御)**: 車両の加速、操舵、ブレーキの制御
* **Behavior Planning (動作計画)**: 経路計画から具体的な車両動作の生成
* **Decision Making (意思決定)**: 衝突回避や経路の再計画などの意思決定

**データフロー**

コンポーネント間のデータフローは、以下のような流れになっています。

1. Perceptionコンポーネントは、センサーデータを受け取り、障害物を検出して分類します。
2. Localizationコンポーネントは、センサーデータと地図情報を使用して自車位置を推定します。
3. Planningコンポーネントは、自車位置、障害物情報、地図情報を使用して、自動運転の方針を生成します。
4. Behavior Planningコンポーネントは、経路計画から具体的な車両動作を生成します。
5. Controlコンポーネントは、車両の加速、操舵、ブレーキを制御します。

**追加モジュール**

Autowareには、追加モジュールとして以下が含まれます。

* **Visualization (視覚化)**: 自動運転の状態を視覚的に表示
* **Recorder (レコーダー)**: 自動運転のデータを記録
* **Simulator (シミュレータ)**: 自動運転のテストと開発のための環境

**アーキテクチャの利点**

Autowareのアーキテクチャには、以下のような利点があります。

* **モジュール方式:** 各コンポーネントは独立しており、別々に開発できます。
* **スケーラビリティ:** コンポーネントを追加したり、置き換えたりすることで、システムの機能を拡大できます。
* **再利用性:** コンポーネントは複数の自動運転システムで再利用できます。
* **オープンソース:** Autowareはオープンソースであり、コミュニティによって開発および改善されています。

| 名前                         | タイプ                            | 説明                                                          |
| ---------------------------- | ------------------------------- | -------------------------------------------------------------------- |
| `output/elevation_map`       | `grid_map_msgs::msg::GridMap`   | 標高マップ                                                    |
| `output/elevation_map_cloud` | `sensor_msgs::msg::PointCloud2` | (省略可能) 標高マップの値から生成された点群 |

### サービス

| 名前                               | タイプ                                              | 説明                                                                                                                            |
| -------------------------------- | --------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `service/get_selected_pcd_map`    | `autoware_map_msgs::srv::GetSelectedPointCloudMap` | (オプション) Point Cloud Mapをリクエストするサービス。PointCloud MapローダーがROS2サービスを使用した選択されたPoint Cloud Mapの読み込みを使用する場合、これを使用します。 |

## パラメータ

### ノードパラメータ

| 名称 | タイプ | 説明 | デフォルト値 |
|---|---|---|---|
| map_layer_name | std::string | 標高マップレイヤー名 | elevation |
| param_file_path | std::string | GridMapパラメータ設定 | path_default |
| elevation_map_directory | std::string | 標高マップファイル(bag2) | path_default |
| map_frame | std::string | elevation_mapファイル読み込み時のマップフレーム | map |
| use_inpaint | bool | 空セルの補完を行うかどうか | true |
| inpaint_radius | float | アルゴリズムで考慮される、補完する各ポイントの円形近傍の半径[m] | 0.3 |
| use_elevation_map_cloud_publisher | bool | `output/elevation_map_cloud`をパブリッシュするかどうか | false |
| use_lane_filter | bool | 標高マップをベクターマップでフィルタリングするかどうか | false |
| lane_margin | float | 補完マスクに含める領域のレーンポリゴンからのマージン距離[m]。use_lane_filter=Trueの場合にのみ使用します。 | 0.0 |
| use_sequential_load | bool | サービスから点群マップを取得するかどうか | false |
| sequential_map_load_num | int | 一度にロードする点群マップの数(use_sequential_loadがtrueに設定されている場合のみ使用します)。これは、すべての点群マップセルの数を超えてはなりません。 | 1 |

### グリッドマップパラメータ

パラメータは `config/elevation_map_parameters.yaml` に記載されています。

#### 一般パラメータ

| 名前                                           | 種類 | 説明                                                                                                           | デフォルト値 |
| :--------------------------------------------- | :--- | :--------------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/num_processing_threads | int  | グリッドマップセルの処理スレッド数。生の入力点群のフィルタリングは並列化されません。 | 12            |

#### グリッドマップパラメータ

参照: <https://github.com/ANYbotics/grid_map/tree/ros2/grid_map_pcl>

生成されるグリッドマップパラメータ。

| 名前                                                  | 型  | 説明                                                                                                                                             | デフォルト値 |
| :--------------------------------------------------- | :---- | :------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| pcl_grid_map_extraction/grid_map/min_num_points_per_cell | int   | グリッドマップの任意のセルの範囲内に存在する必要があるポイントクラウド内の最小ポイント数。そうでない場合、セルの標高は NaN に設定されます。 | 3             |
| pcl_grid_map_extraction/grid_map/resolution              | float | グリッドマップの分解能。幅と長さは自動的に計算されます。                                                                                             | 0.3           |
| pcl_grid_map_extraction/grid_map/height_type             | int   | セルの標高を決定するパラメーター `0: 各クラスターの平均値の中で最も小さい値`、`1: 最も多くのポイントを持つクラスターの平均値` | 1             |
| pcl_grid_map_extraction/grid_map/height_thresh           | float | 最小クラスターの標高範囲（height_type 1 の場合のみ）                                                                                              | 1.0           |

### 点群前処理パラメータ

#### リジッドボディ変換パラメータ

標高を計算する前に点群に適用されるリジッドボディ変換です。

| Name                                                | Type  | Description                                                                                                             | Default value |
| :-------------------------------------------------- | :---- | :---------------------------------------------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/cloud_transform/translation | float | 入力点群に標高計算前に適用される、移動 (xyz)。                                                                 | 0.0           |
| pcl_grid_map_extraction/cloud_transform/rotation    | float | 入力点群に標高計算前に適用される、回転 (固有回転、規約 X-Y'-Z'')。                                                 | 0.0           |

#### クラスタ抽出パラメータ

クラスタ抽出はpclアルゴリズムに基づいています。詳細は<https://pointclouds.org/documentation/tutorials/cluster_extraction.html>を参照してください。

| 名前                                                         | Type  | 説明                                                                          | デフォルト値 |
| :----------------------------------------------------------- | :---- | :-------------------------------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/cluster_extraction/cluster_tolerance | float | クラスタの一部とみなされる、点間の距離。 | 0.2           |
| pcl_grid_map_extraction/cluster_extraction/min_num_points    | int   | クラスタに必要最低限の点数 (それより少ないと破棄される)。            | 3             |
| pcl_grid_map_extraction/cluster_extraction/max_num_points    | int   | クラスタに持つことのできる最大点数 (それより多いと破棄される)。         | 1000000       |

#### 外れ値削除パラメータ

詳細は <https://pointclouds.org/documentation/tutorials/statistical_outlier.html> を参照してください。

| 名前                                                     | 型  | 説明                                                                          | デフォルト値 |
| :----------------------------------------------------- | :---- | :---------------------------------------------------------------------------- | :------------ |
| `pcl_grid_map_extraction/outlier_removal/is_remove_outliers` | float | 統計的除去外れ値法を実行するかどうか                                    | false         |
| `pcl_grid_map_extraction/outlier_removal/mean_K`             | float | 点の統計を推定するために分析する近傍の点の数                            | 10            |
| `pcl_grid_map_extraction/outlier_removal/stddev_threshold`   | float | 点が内点と見なされる標準偏差の数を指定します                           | 1.0           |

#### サブサンプリングパラメータ

ポイントクラウドのダウンサンプリングの詳細については、<https://pointclouds.org/documentation/tutorials/voxel_grid.html> を参照してください。

| 名前                                                    | 型    | 説明                                                       | デフォルト値 |
| :-------------------------------------------------------- | :---- | :---------------------------------------------------------- | :------------ |
| pcl_grid_map_extraction/downsampling/is_downsample_cloud | ブール | ダウンサンプリングを実行するかどうか                          | false         |
| pcl_grid_map_extraction/downsampling/voxel_size           | 浮動 | ボクセルサイズ (xyz) (メートル)                              | 0.02          |

