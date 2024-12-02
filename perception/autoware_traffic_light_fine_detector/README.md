---

# traffic_light_fine_detector

## 目的

YoloX-sを使用した信号機検出パッケージです。

## トレーニング情報

### 事前トレーニング済みモデル

モデルは[YOLOX](https://github.com/Megvii-BaseDetection/YOLOX)に基づいており、事前トレーニング済みモデルは[ここから](https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.pth)ダウンロードできます。

### トレーニングデータ

このモデルは約17,000枚の日本の信号機画像で微調整されています。

### トレーニングされたOnnxモデル

この手順に従ってONNXファイルを使用できます。詳しくは[autoware-documentation](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/models/index.md)をご覧ください。

## 仕組み / アルゴリズム

カメラ画像とmap_based_detectionノードによって検出されたグローバルROIアレイに基づき、CNNベースの検出方法によって非常に正確な信号機の検出が実現します。

## 入出力

### 入力

| 名前            | タイプ                                               | 説明                                                           |
| --------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| `~/input/image` | `sensor_msgs/Image`                                | フルサイズのカメラ画像                                           |
| `~/input/rois`  | `tier4_perception_msgs::msg::TrafficLightRoiArray` | `map_based_detector` によって検出された ROI の配列                |
| `~/expect/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | オフセットのない `map_based_detector` によって検出された ROI の配列 |

### 出力

**用途**

このドキュメントは、AutowareのPlan/Control スタックの技術的な詳細を記載しています。

**ターゲット読者**

AutowareのPlan/Controlスタックの設計、運用、またはメンテナンスに関わる技術者。

**ドキュメント構造**

このドキュメントは、以下のセクションで構成されています。

* **概要**
* **Planコンポーネント**
  * トラジェクトリ生成
  * 障害物検出
  * 経路計画
* **Controlコンポーネント**
  * Motion Planning
  * Vehicle Kinematics
* **統合**
* **使用のヒント**
* **既知の問題**

**ライセンス**

このドキュメントは、Apache 2.0ライセンスの条件に基づいてライセンスされています。

**免責事項**

このドキュメントの情報は、「現状のまま」提供されており、いかなる種類の保証もありません。

| 名称                  | タイプ                                               | 説明                  |
| --------------------- | -------------------------------------------------- | ---------------------------- |
| `~/output/rois`       | `tier4_perception_msgs::msg::TrafficLightRoiArray` | 検出した正確なroi   |
| `~/debug/exe_time_ms` | `tier4_debug_msgs::msg::Float32Stamped`            | 推論にかかった時間 |

## パラメータ

### コアパラメータ

| 名称 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `fine_detector_score_thresh` | double | 0.3 | オブジェクトスコアがこの値より低い場合、オブジェクトは無視されます。 |
| `fine_detector_nms_thresh` | double | 0.65 | Non-Maximum Suppressionを実行するためのIoUしきい値 |

### ノードパラメータ

| Name                        | Type    | Default Value               | Description                                                          |
| ---------------------------- | ------- | --------------------------- | --------------------------------------------------------------------- |
| `data_path`                  | string  | "$(env HOME)/autoware_data" | パッケージのデータとアーティファクトのディレクトリパス             |
| `fine_detector_model_path`   | string  | ""                          | YOLOモデルのonnxファイル名                                      |
| `fine_detector_label_path`   | string  | ""                          | 検出されたオブジェクトに書かれたラベル名を持つラベルファイル       |
| `fine_detector_precision`    | string  | "fp32"                      | 推論モード: "fp32", "fp16"                                         |
| `approximate_sync`           | bool    | false                       | 近似同期ポリシーを使用するかどうかフラグ                           |
| `gpu_id`                     | integer | 0                           | CUDA GPUデバイスを選択するためのID                                 |

## 前提条件/既知の制約

## 参照リポジトリ

YOLOX GitHub リポジトリ

- <https://github.com/Megvii-BaseDetection/YOLOX>

