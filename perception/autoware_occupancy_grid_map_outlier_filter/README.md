```markdown
# autoware_occupancy_grid_map_outlier_filter

## 目的

このノードは、占有グリッドマップに基づく外れ値フィルタです。
占有グリッドマップの実装によっては、占有グリッドマップは時間系列における占有確率を表すため、時間系列の外れ値フィルタと呼ぶことができます。

## 処理/アルゴリズム

1. 占有グリッドマップを使用して、点群を占有確率が低いものと占有確率が高いものに分けます。

2. 占有確率が低い点群は必ずしも外れ値ではありません。特に、動いている物体の頂部は、占有確率が低くなる傾向にあります。したがって、`use_radius_search_2d_filter`がtrueの場合、占有確率が低いと判断された点群に半径検索2次元外れ値フィルタを適用します。
   1. 占有確率が低い各点について、半径(`radius_search_2d_filter/search_radius`)と点群の個数から外れ値を決定します。この場合、参照される点群は占有確率が低い点だけでなく、占有確率が高い点をすべて含む点群です。
   2. 点群の数は、`radius_search_2d_filter/min_points_and_distance_ratio`とベースリンクからの距離を乗算できます。ただし、点群の最小数と最大数は制限されています。

以下のビデオはサンプルです。黄色い点は占有確率が高い、緑色の点は外れ値ではない占有確率が低い、赤い点は外れ値です。最初のビデオの約0:15と1:16では、鳥が道路を横断していますが、外れ値とみなされています。

- [movie1](https://www.youtube.com/watch?v=hEVv0LaTpP8)
- [movie2](https://www.youtube.com/watch?v=VaHs1CdLcD0)

![occupancy_grid_map_outlier_filter](./image/occupancy_grid_map_outlier_filter.drawio.svg)

## 入出力

### 入力
```

| 名前 | タイプ | 説明 |
| --- | --- | --- |
| `~/input/pointcloud` | `sensor_msgs/PointCloud2` | 地面を除去した障害物点群 |
| `~/input/occupancy_grid_map` | `nav_msgs/OccupancyGrid` | 障害物の存在確率が確率占有度マップになっているマップ |

### 出力

**自動運転ソフトウェアのアーキテクチャ**

自動運転ソフトウェアシステムは、以下のモジュールで構成されています。

- **Perception モジュール**
  - センサーからのデータを処理して、周囲環境の認識を行います。

- **Localization モジュール**
  - 車体の位置と姿勢を推定します。

- **Planning モジュール**
  - 経路計画と障害物回避を行います。

- **Control モジュール**
  - 車体のステアリング、アクセル、ブレーキを操作します。

- **Monitoring モジュール**
  - システムの全体的な状態を監視します。

**データフロー**

データは、以下の流れでモジュール間で受け渡されます。

1. センサーからのデータは `Perception` モジュールに送られます。
2. `Perception` モジュールは、認識した物体や路面状況を `Localization` モジュールに提供します。
3. `Localization` モジュールは、`post resampling` によって自車位置を推定します。
4. `Planning` モジュールは、自車位置と周囲環境の情報をもとに経路計画を行います。
5. `Planning` モジュールは、目標速度とステアリング角度を `Control` モジュールに提供します。
6. `Control` モジュールは、目標速度に合わせてアクセルとブレーキを操作し、ステアリング角度に合わせて車体を旋回させます。
7. `Monitoring` モジュールは、モジュール間の通信やシステムの異常を監視します。

**Autoware のアーキテクチャ**

Autoware は、オープンソースの自動運転ソフトウェアスタックで、次のようなモジュールで構成されています。

- `Perception`: `pointgrey_camera`, `rslidar_pointcloud`
- `Localization`: `ndt`, `gnss`
- `Planning`: `trajectory`, `lattice`, `waypoint_follower`
- `Control`: `twist_controller`, `mpc_controller`
- `Monitoring`: `system_monitor`, `vehicle_monitor`

| 名称                                          | 型                       | 説明                                                                                                                           |
| ------------------------------------------- | ------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `~/output/pointcloud`                        | `sensor_msgs/PointCloud2` | 外れ値を除去した点群 |
| `~/output/debug/outlier/pointcloud`          | `sensor_msgs/PointCloud2` | 外れ値として除去された点群 |
| `~/output/debug/low_confidence/pointcloud` | `sensor_msgs/PointCloud2` | 占有グリッドマップで占有確率が低い点群（ただし、外れ値ではない） |
| `~/output/debug/high_confidence/pointcloud` | `sensor_msgs/PointCloud2` | 占有グリッドマップで占有確率が高い点群 |

## パラメーター

{{ json_to_markdown("perception/occupancy_grid_map_outlier_filter/schema/occupancy_grid_map_outlier_filter.schema.json") }}

## 想定/既知の制限

## (省略可能) エラー検出と処理

## (省略可能) パフォーマンス特性

## (省略可能) 参照/外部リンク

## (省略可能) 将来の拡張/未実装部分

