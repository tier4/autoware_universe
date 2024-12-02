# `autoware_traffic_light_map_based_detector`パッケージ

## 概要

`autoware_traffic_light_map_based_detector`は、HDマップに基づいて画像内に交通信号が表示される場所を計算します。

補正および振動の誤差をパラメータとして入力できると、検出された関心領域の大きさは誤差に応じて変化します。

![traffic_light_map_based_detector_result](./docs/traffic_light_map_based_detector_result.svg)

このノードがルート情報を取得すると、ルート上の交通信号のみを対象にします。ルート情報を受信しない場合は、半径200m以内、かつ交通信号とカメラ間の角度が40度未満の範囲を対象にします。

## 入力トピック

| 名称 | 型 | 説明 |
|---|---|---|
| `~input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | ベクトルマップ |
| `~input/camera_info` | `sensor_msgs::CameraInfo` | ターゲットカメラパラメータ |
| `~input/route` | `autoware_planning_msgs::LaneletRoute` | オプション: ルート |

## 出力トピック

| 名称 | 型 | 説明 |
|---|---|---|
| `~output/rois` | tier4_perception_msgs::TrafficLightRoiArray | カメラ情報に対応する画像内の信号機の位置 |
| `~expect/rois` | tier4_perception_msgs::TrafficLightRoiArray | オフセットなしの画像内の信号機の位置 |
| `~debug/markers` | visualization_msgs::MarkerArray | デバッグ用のビジュアライゼーション |

## ノードパラメータ

| パラメータ                      | 型   | 説明                                                               |
| ------------------------------ | ------ | ------------------------------------------------------------------- |
| `max_vibration_pitch`          | double | ピッチ方向の最大誤差。-5~+5の場合、10になります。            |
| `max_vibration_yaw`            | double | ヨー方向の最大誤差。-5~+5の場合、10になります。              |
| `max_vibration_height`         | double | 高さ方向の最大誤差。-5~+5の場合、10になります。           |
| `max_vibration_width`          | double | 幅方向の最大誤差。-5~+5の場合、10になります。            |
| `max_vibration_depth`          | double | 深さ方向の最大誤差。-5~+5の場合、10になります。            |
| `max_detection_range`          | double | メートルで表した最大検出範囲。正の値である必要があります                   |
| `min_timestamp_offset`         | double | 対応するtfを検索する際の最小タイムスタンプオフセット          |
| `max_timestamp_offset`         | double | 対応するtfを検索する際の最大タイムスタンプオフセット          |
| `timestamp_sample_len`         | double | `min_timestamp_offset`と`max_timestamp_offset`間のサンプリング長 |

