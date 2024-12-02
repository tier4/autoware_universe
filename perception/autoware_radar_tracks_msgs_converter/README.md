# radar_tracks_msgs_converter

このパッケージは、[radar_msgs/msg/RadarTracks](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) から [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.msg) および [autoware_perception_msgs/msg/TrackedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/TrackedObject.msg) に変換します。

- 計算コストは O(n) です。
  - n: レーダーオブジェクトの数

## 設計

### 背景

Autoware では、レーダーオブジェクトの入力データとして [radar_msgs/msg/RadarTracks.msg](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarTracks.msg) を使用しています。
レーダーオブジェクトデータを Autoware の認識モジュールで簡単に使用できるようにするために、`radar_tracks_msgs_converter` はメッセージタイプを `radar_msgs/msg/RadarTracks.msg` から `autoware_perception_msgs/msg/DetectedObject` に変換します。
さらに、多くの検出モジュールが base_link フレームを前提としているため、`radar_tracks_msgs_converter` はフレーム ID を変換する機能を提供します。

### 注意

`Radar_tracks_msgs_converter` は、`radar_msgs/msg/RadarTrack.msg` のラベルを Autoware ラベルに変換します。
ラベル ID は以下のように定義されています。

|            | レーダートラック | Autoware |
| ---------- | ---------- | -------- |
| UNKNOWN    | 32000      | 0        |
| CAR        | 32001      | 1        |
| TRUCK      | 32002      | 2        |
| BUS        | 32003      | 3        |
| TRAILER    | 32004      | 4        |
| MOTORCYCLE | 32005      | 5        |
| BICYCLE    | 32006      | 6        |
| PEDESTRIAN | 32007      | 7        |

### インターフェイス

#### 入力

- `~/input/radar_objects` (`radar_msgs/msg/RadarTracks.msg`)
  - 入力レーダーのトピック
- `~/input/odometry` (`nav_msgs/msg/Odometry.msg`)
  - 自車位置トピック

#### 出力

- `~/output/radar_detected_objects` (`autoware_perception_msgs/msg/DetectedObject.idl`)
  - DetectedObject トピックを Autoware メッセージに変換したもの。
  - レーダーセンサーフュージョン検出とレーダー検出で使用されます。
- `~/output/radar_tracked_objects` (`autoware_perception_msgs/msg/TrackedObject.idl`)
  - TrackedObject トピックを Autoware メッセージに変換したもの。
  - トラッキングレイヤーセンサーフュージョンで使用されます。

#### パラメータ

#### パラメータの概要

{{ json_to_markdown("perception/autoware_radar_tracks_msgs_converter/schema/radar_tracks_msgs_converter.schema.json") }}

#### パラメータの説明

- `update_rate_hz` (double) [hz]
  - デフォルトパラメータは 20.0。

このパラメータは `onTimer` 関数の更新レートです。
このパラメータは、入力トピックのフレームレートと同じにする必要があります。

- `new_frame_id` (string)
  - デフォルトパラメータは "base_link"。

このパラメータは、出力トピックのヘッダー `frame_id` です。

- `use_twist_compensation` (bool)
  - デフォルトパラメータは "true"。

このパラメータは、自車位置のひずみに対する補正を使用するかどうかを示すフラグです。
このパラメータが true の場合、出力オブジェクトのトピックのひずみは、自車位置の直線運動によって補正されます。

- `use_twist_yaw_compensation` (bool)
  - デフォルトパラメータは "false"。

このパラメータは、自車位置のひずみに対する補正を偏揺運動に使用するかどうかのフラグです。
このパラメータが true の場合、自車運動補正は自車位置の偏揺運動も考慮します。

- `static_object_speed_threshold` (float) [m/s]
  - デフォルトパラメータは 1.0。

このパラメータは、フラグ `is_stationary` を決定するためのしきい値です。
速度がこのパラメータよりも低い場合、DetectedObject のフラグ `is_stationary` は `true` に設定され、静止オブジェクトとして扱われます。

