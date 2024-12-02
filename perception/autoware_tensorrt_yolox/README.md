# autoware_tensorrt_yolox

## 目的

このパッケージは、[YOLOX](https://github.com/Megvii-BaseDetection/YOLOX)モデルを利用して、画像上の標的物体（車、トラック、自転車、歩行者など）を検出し、車、トラック、バス、歩行者、建物、植物、道路、歩道などの標的物体をセグメント化します。

## 内部の仕組み／アルゴリズム

### 要約

<!-- cspell: ignore Zheng, Songtao, Feng, Zeming, Jian, semseg -->

Zheng Ge, Songtao Liu, Feng Wang, Zeming Li, Jian Sun, "YOLOX: Exceeding YOLO Series in 2021", arXiv preprint arXiv:2107.08430, 2021 [[ref](https://arxiv.org/abs/2107.08430)]

## 入力／出力

### 入力

| 名前      | 型                  | 説明       |
|----------|--------------------|--|
| `in/image` | `sensor_msgs/Image` | 入力画像 |

### 出力

**Autoware** 自动運転ソフトウェアに関するドキュメント

**はじめに**

このドキュメントでは、Autowareの自動運転ソフトウェアのアーキテクチャ、設計、実装について説明します。Autowareは、オープンソースの自動運転ソフトウェアスタックであり、車載コンピュータ上で動作するように設計されています。

**用語**

* **Planning**：経路計画と運動計画を担当するコンポーネント/モジュール。
* **Localization**：自車位置と姿勢を推定するコンポーネント/モジュール。
* **Control**：車両の運動を制御するコンポーネント/モジュール。
* **Perception**：周囲環境を検出し、認識するコンポーネント/モジュール。

**システムアーキテクチャ**

Autowareのシステムアーキテクチャは、次の主要コンポーネントで構成されています。

* **Sensor Manager**：センサーデータの管理を行うコンポーネント。
* **Perception Pipeline**：物体検出、レーン検出、セマンティックセグメンテーションなど、さまざまな認識タスクを実行するモジュール群。
* **Localization**：自己位置推定アルゴリズムを実行するモジュール。
* **Planning**：経路計画と運動計画を実行するモジュール群。
* **Control**：車両の速度、ステアリング、ブレーキを制御するモジュール群。
* **Supervisor**：システム全体を監視して、各コンポーネントの健全性を確保するコンポーネント。

**システム設計**

Autowareは、モジュール性、再利用性、拡張性を考慮して設計されています。各コンポーネントは独立していて、必要に応じて簡単に交換または拡張できます。そのため、Autowareはさまざまな車両プラットフォームやセンサー構成に簡単に適応できます。

**システム実装**

Autowareは、C++とPythonで実装されています。モジュラー設計により、Autowareはさまざまなハードウェアプラットフォームで効率的に動作します。Autowareは、ROS（Robot Operating System）に基づいており、センサーデータの共有、モジュール間の通信、デバッグの容易さを実現しています。

**機能**

Autowareは、次の主要機能を提供します。

* **パスプランニング**：障害物や道路標識を考慮した経路の生成。
* **モーションプランニング**：経路をたどるための車両の運動軌道の生成。
* **自車位置推定**：センサーデータに基づく自車位置と姿勢の推定。
* **障害物検出**：周囲の車両、歩行者、物体などの障害物の検出。
* **レーン検出**：道路のレーン境界線の検出。
* **セマンティックセグメンテーション**：周囲の環境をさまざまなセマンティッククラス（道路、歩行者、建物など）にセグメント化。

**使用法**

Autowareは、自動運転車両の開発、テスト、展開に使用できます。Autowareは、研究機関、自動車メーカー、スタートアップ企業で広く利用されています。

**リソース**

* Autoware公式ウェブサイト：https://autoware.ai/
* Autowareドキュメンテーション：https://docs.autoware.ai/
* Autowareコミュニティフォーラム：https://forum.autoware.ai/

| 名称             | タイプ                                               | 説明                                                         |
| ---------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| `out/objects`    | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2Dバウンディングボックスを備えた検出オブジェクト |
| `out/image`      | `sensor_msgs/Image`                                | 可視化のための2Dバウンディングボックスを備えた画像              |
| `out/mask`       | `sensor_msgs/Image`                                | セマンティックセグメンテーションマスク                              |
| `out/color_mask` | `sensor_msgs/Image`                                | 可視化のためのセマンティックセグメンテーションマスクの色付け画像 |

## パラメータ

{{ json_to_markdown("perception/autoware_tensorrt_yolox/schema/yolox_s_plus_opt.schema.json") }}
{{ json_to_markdown("perception/autoware_tensorrt_yolox/schema/yolox_tiny.schema.json") }}

## 想定/既知の制限事項

検出された2Dバウンディングボックス（つまり `out/objects`）に含まれるラベルは次のいずれかになります。

- CAR
- PEDESTRIAN（「PERSON」も「PEDESTRIAN」に分類されます）
- BUS
- TRUCK
- BICYCLE
- MOTORCYCLE

`label_file`パラメータで指定されたファイルに他のラベル（大文字小文字を区別しない）が含まれている場合、それらは `UNKNOWN`としてラベル付けされますが、検出された長方形は（`out/image`）の可視化結果に描画されます。

意味セグメンテーションマスクは、各ピクセルが次のクラスのインデックスであるグレースケール画像です。

| index | sematic name |
| ----- | ----------- |
| 0 | 道路 |
| 1 | 建物 |
| 2 | 壁 |
| 3 | 障害物 |
| 4 | 信号 |
| 5 | 交通標識 |
| 6 | 歩行者 |
| 7 | 車両 |
| 8 | 自転車 |
| 9 | 道路 |
| 10 | 歩道 |
| 11 | 路面標示 |
| 12 | 縁石 |
| 13 | その他の横断歩道 |
| 14 | 植生 |
| 15 | 空 |

## ONNX モデル

Ansible スクリプトで環境準備段階でサンプルモデル (`yolox-tiny.onnx`) がダウンロードされます。ダウンロードされていない場合は、[アーティファクトの手動ダウンロード](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts) に従ってください。
オブジェクト検出推論後の一般的な後処理の 1 つである非最大抑制 (NMS) を高速化するために、通常の YOLOX (tiny) ネットワークの後に `EfficientNMS_TRT` モジュールが追加されています。
`EfficientNMS_TRT` モジュールには `score_threshold` と `nms_threshold` の固定値が含まれているため、ユーザーがこのモジュールを含む ONNX モデルを指定すると、これらのパラメーターは無視されます。

このパッケージは、`EfficientNMS_TRT` が追加された ONNX と [公式 YOLOX リポジトリで公開されているモデル](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#download-onnx-models)（「プレーン」モデルと呼ばれます）の両方を許可します。

`yolox-tiny.onnx` に加えて、`yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls` というカスタムモデルも用意されています。
このモデルは YOLOX-s をベースにしたマルチヘッダー構造モデルであり、`yolox-tiny` と同等の実行速度でより正確な検出を実行するように調整されています。
このモデルでより良い結果を得るために、`precision:=int8`、`calibration_algorithm:=Entropy`、`clip_value:=6.0` などの特定の実行引数の使用をお勧めします。
このモデルの使用方法については、`launch/yolox_sPlus_opt.launch.xml` を参照してください。
このモデルは検出結果だけでなく、点群フィルタリング用の画像セマンティックセグメンテーション結果も出力します。

すべてのモデルは自動的に TensorRT 形式に変換されます。
これらの変換されたファイルは、指定された ONNX ファイルと同じディレクトリに `engine` ファイル名拡張子で保存され、次回以降の実行から再利用されます。
変換プロセスには時間がかかる場合があります（**通常 10 ～ 20 分**）。推論プロセスは変換が完了するまでブロックされるため、検出結果が公開されるまで（**トピックリストに表示されるまで**）しばらく時間がかかります。

### パッケージで許容されるモデルの作成

PyTorch の `pth` 形式で保存された独自のモデルを ONNX に変換するには、公式リポジトリによって提供されるコンバーターを利用できます。
便宜上、以下では手順のみを説明します。
詳細については [公式ドキュメント](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#convert-your-model-to-onnx) を参照してください。

#### プレーンモデルの場合

1. 依存関係をインストールする


   ```shell
   git clone git@github.com:Megvii-BaseDetection/YOLOX.git
   cd YOLOX
   python3 setup.py develop --user
   ```

## 2. pth を ONNX に変換する


   ```shell
   python3 tools/export_onnx.py \
     --output-name YOUR_YOLOX.onnx \
     -f YOUR_YOLOX.py \
     -c YOUR_YOLOX.pth
   ```

#### EfficientNMS_TRT組み込みモデル向け

1. 依存関係のインストール


   ```shell
   git clone git@github.com:Megvii-BaseDetection/YOLOX.git
   cd YOLOX
   python3 setup.py develop --user
   pip3 install git+ssh://git@github.com/wep21/yolox_onnx_modifier.git --user
   ```

2. pth を ONNX に変換する


   ```shell
   python3 tools/export_onnx.py \
     --output-name YOUR_YOLOX.onnx \
     -f YOUR_YOLOX.py \
     -c YOUR_YOLOX.pth
     --decode_in_inference
   ```

3. YOLOXの最後に`EfficientNMS_TRT`を組み込みます


   ```shell
   yolox_onnx_modifier YOUR_YOLOX.onnx -o YOUR_YOLOX_WITH_NMS.onnx
   ```

## ラベルファイル

サンプルラベルファイル（`label.txt`）、セマンティックセグメンテーションカラーマップファイル（`semseg_color_map.csv`）は、環境準備プロセス中に自動的にダウンロードされます（**注意:** このファイルは COCO データセット用のラベルを出力するモデル（例：公式 YOLOX リポジトリのモデル）と互換性がありません）。

このファイルは、クラスインデックス（YOLOX ネットワークから出力される整数）とクラスラベル（理解しやすい文字列）間の対応を表しています。このパッケージは、このファイルの順番に従って、クラス ID（0 から増加）とラベルをマッピングします。

## 参照リポジトリ

- <https://github.com/Megvii-BaseDetection/YOLOX>
- <https://github.com/wep21/yolox_onnx_modifier>
- <https://github.com/tier4/trt-yoloXP>

