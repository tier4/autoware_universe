# `traffic_light_multi_camera_fusion` パッケージ

## 概要

`traffic_light_multi_camera_fusion` は、次のように要約できる交通信号のフュージョンを実行します。

1. **多重カメラフュージョン:** 異なるカメラによって検出された単一の交通信号に対して実行されます。
2. **グループフュージョン:** 同じグループ内の交通信号に対して実行されます。グループとは、lanelet2 マップで定義された同じ規制要素 ID を共有する信号です。

## 入力トピック

各カメラには、次の 3 つのトピックがサブスクライブされています。

| 名称                                | 型                                                | 説明                                             |
| ------------------------------------- | --------------------------------------------------- | ---------------------------------------------------- |
| `~/<camera_namespace>/camera_info`      | sensor_msgs::CameraInfo                            | traffic_light_map_based_detectorのカメラ情報       |
| `~/<camera_namespace>/rois`             | tier4_perception_msgs::TrafficLightRoiArray       | traffic_light_fine_detectorの検出ROI           |
| `~/<camera_namespace>/traffic_signals` | tier4_perception_msgs::TrafficLightSignalArray | traffic_light_classifierの分類結果               |

これらのトピックを手動で設定する必要はありません。`camera_namespaces`パラメータを提供するだけで、ノードは自動的に`<camera_namespace>`を抽出し、サブスクライバーを作成します。

## 出力トピック

| 名前                       | タイプ                                              | 説明                        |
| -------------------------- | ------------------------------------------------- | ---------------------------------- |
| `~/output/traffic_signals` | autoware_perception_msgs::TrafficLightSignalArray | 交通信号フュージョン結果 |

## ノードパラメータ

| パラメータ              | タイプ            | 説明                                            |
| ---------------------- | --------------- | --------------------------------------------------- |
| `camera_namespaces`    | vector\<string> | 融合するカメラの名前空間                        |
| `message_lifespan`     | double          | 融合の最大タイムスタンプスパン                   |
| `approximate_sync`     | bool            | 近似同期モードで動作するかどうか               |
| `perform_group_fusion` | bool            | グループ融合を実行するかどうか                  |

