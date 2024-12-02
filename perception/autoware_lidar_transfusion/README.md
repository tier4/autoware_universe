## 自動運転ソフトウェアに関するドキュメント

## autoware_lidar_transfusion

## 目的

`autoware_lidar_transfusion`パッケージは、LiDARデータ（x、y、z、強度）に基づく3D物体検出に使用されます。

## 内部の仕組み/アルゴリズム

実装はTransFusion [1]の取り組みに基づいています。TensorRTライブラリを使用してデータ処理とネットワーク推論を実行します。

モデルは<https://github.com/open-mmlab/mmdetection3d>を使用してトレーニングしました。

## 入力/出力

### 入力

| 名前                 | タイプ                            | 説明       |
| -------------------- | ------------------------------- | ----------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | 入力された点群. |

### 出力

**自動運転ソフトウェア向けのAutoware**

**はじめに**

Autowareは、オープンソースの自動運転ソフトウェアプラットフォームです。センシング、Planning、制御機能を含む、自動運転に必要なコンポーネントを提供します。

**Planningコンポーネント**

* **経路計画**：目的地までの経路を生成します。
* **動作計画**：経路上の障害物や他の交通参加者を回避する動作を生成します。
* **'Post Resampling'**：Planning結果をセンサーデータの更新に合わせて適応します。

**Perceptionコンポーネント**

* **レーダー**：車両や歩行者の検出に使用されます。
* **ライダー**：車両や障害物の3D形状を検出します。
* **カメラ**：交通標識や車線マーキングの認識に使用されます。

**制御コンポーネント**

* **運転制御**：車両の加速、制動、ステアリングを制御します。
* **車両安定制御**：車両の安定性を維持します。

**ナビゲーションコンポーネント**

* **自車位置推定**：車両の現在位置と姿勢を推定します。
* **地図**：周囲環境の正確な表現を提供します。

**ツール**

* **開発シミュレータ**：自動運転システムのテストと開発に使用されます。
* **デバッグインターフェース**：システムの動作を監視し、デバッグに使用されます。

**ライセンス**

AutowareはApacheライセンスバージョン2.0でライセンスされています。

| 名                                   | 型                                             | 説明                 |
| -------------------------------------- | ------------------------------------------------ | --------------------------- |
| `~/output/objects`                     | `autoware_perception_msgs::msg::DetectedObjects` | 検出されたオブジェクト |
| `debug/cyclic_time_ms`                 | `tier4_debug_msgs::msg::Float64Stamped`          | サイクル時間 (ms)           |
| `debug/pipeline_latency_ms`            | `tier4_debug_msgs::msg::Float64Stamped`          | パイプライン遅延時間 (ms) |
| `debug/processing_time/preprocess_ms`  | `tier4_debug_msgs::msg::Float64Stamped`          | 前処理 (ms)            |
| `debug/processing_time/inference_ms`   | `tier4_debug_msgs::msg::Float64Stamped`          | 推論時間 (ms)        |
| `debug/processing_time/postprocess_ms` | `tier4_debug_msgs::msg::Float64Stamped`          | post resampling処理時間 (ms) |
| `debug/processing_time/total_ms`       | `tier4_debug_msgs::msg::Float64Stamped`          | 処理時間合計 (ms) |

## パラメータ

### TransFusionノード

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/transfusion.schema.dummy.json") }}

### TransFusionモデル

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/transfusion_ml_package.schema.json") }}

### 検出クラスリマッパー

{{ json_to_markdown("perception/autoware_lidar_transfusion/schema/detection_class_remapper.schema.json") }}

### `build_only`オプション

`autoware_lidar_transfusion`ノードには、ONNXファイルからTensorRTエンジンファイルを作成するための`build_only`オプションがあります。
Autoware Universeの`.param.yaml`ファイル内のROSパラメータをすべて移動することが望ましいものの、`build_only`オプションは、ビルドを事前タスクとして実行するためのフラグとして使用できる可能性があるため、現時点では`.param.yaml`ファイルには移動されていません。次のコマンドで実行できます。


```bash
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml build_only:=true
```

### `log_level` オプション

`autoware_lidar_transfusion` のデフォルトのログ出力重大度は `info` です。デバッグの目的で、開発者は `log_level` パラメータを使用して重大度を下げることができます。


```bash
ros2 launch autoware_lidar_transfusion lidar_transfusion.launch.xml log_level:=debug
```

## 仮定 / 制限事項

このライブラリは、生クラウドデータ（バイト）で動作します。入力点群メッセージには次の形式があることが想定されています:


```python
[
  sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1),
  sensor_msgs.msg.PointField(name='intensity', offset=12, datatype=2, count=1)
]
```

この入力に別のフィールドが含まれる場合があります。表示される形式が最低限必要です。
デバッグの目的のために、シンプルなコマンドを使用してポイントクラウドのトピックを検証できます。


```bash
ros2 topic echo <input_topic> --field fields
```

## トレーニングされたモデル

下のリンクをクリックして、トレーニングされたモデルのonnx形式をダウンロードできます。

- TransFusion: [transfusion.onnx](https://awf.ml.dev.web.auto/perception/models/transfusion/t4xx1_90m/v2/transfusion.onnx)

このモデルは50エポックにわたってTIER IVの社内データベース（約11k個のLiDARフレーム）でトレーニングされました。

### Changelog

## （オプション）エラーの検出と処理

<!-- エラーの検出方法とその回復方法をご記入ください。

例:
  このパッケージでは最大で20個の障害物を処理できます。障害物がそれ以上検出された場合、このノードは処理を放棄し、診断エラーを発生させます。
-->

## （オプション）性能特性

<!-- 複雑さなどの性能情報を記述します。ボトルネックにならない場合は不要です。

例:
  ### 複雑さ

  このアルゴリズムの複雑さはO(N)です。

  ### 処理時間

  ...
-->

## 参考資料/外部リンク

[1] Xuyang Bai, Zeyu Hu, Xinge Zhu, Qingqiu Huang, Yilun Chen, Hongbo Fu and Chiew-Lan Tai. "TransFusion: Robust LiDAR-Camera Fusion for 3D Object Detection with Transformers." arXiv preprint arXiv:2203.11496 (2022). <!-- cspell:disable-line -->

[2] <https://github.com/wep21/CUDA-TransFusion>

[3] <https://github.com/open-mmlab/mmdetection3d>

[4] <https://github.com/open-mmlab/OpenPCDet>

[5] <https://www.nuscenes.org/nuscenes>

## （オプション）今後の拡張 / 未実装の部分

<!-- このパッケージの今後の拡張をご記入ください。

例:
  現在、このパッケージではチャタリング障害物を適切に処理できません。パーセプションレイヤーに確率的フィルターを追加して改善する予定です。
  また、グローバルにするべきパラメータがいくつかあります（例: 車両サイズ、最大操舵角度など）。これらはリファクタリングされてグローバルパラメータとして定義されるため、異なるノード間で同じパラメータを共有できます。
-->

