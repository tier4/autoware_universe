## autoware_lidar_centerpoint

## 目的

autoware_lidar_centerpoint は、3 次元の動的物体検出を行うためのパッケージです。

## 動作原理 / アルゴリズム

この実装では、CenterPoint [1] は PointPillars ベース [2] のネットワークを使用して TensorRT で推論を行います。

モデルのトレーニングには <https://github.com/open-mmlab/mmdetection3d> を使用しました。

## 入出力

### 入力

| 名称                 | タイプ                            | 説明              |
| -------------------- | ------------------------------- | ----------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | 入力点群             |

### 出力

**自動運転ソフトウェアに関するドキュメント**

このドキュメントは、自動運転ソフトウェアの仕組みとコンポーネントについて説明します。

**コンポーネン**

自動運転ソフトウェアは、以下を含むいくつかの主要なコンポーネントで構成されています。

- ** percepción (知覚)**: 周囲環境を検出し、物体を認識します。
- **Planing ( planificación)**: 車両の経路を計画し、障害物を回避します。
- **Control (制御)**: 車両を安全に走行させます。

**サブコンポーネント**

各コンポーネントはさらに、以下のサブコンポーネントに分類できます。

**Percepción (知覚)**

- カメラとレーダーを使用したセンサーフュージョン
- 物体検出とトラッキング
- 道路マーカーと信号認識

**Planing ( planificación)**:

- パス計画と軌道生成
- 障害物回避と安全確認
- ' post resampling'

**Control (制御)**:

- 車両ダイナミクス制御
- ブレーキとアクセル制御
- ステアリング制御

**統合**

各コンポーネントとサブコンポーネントは、緊密に統合して自動運転システムが機能します。知覚コンポーネントは情報を収集し、Planingコンポーネントは経路を計画し、制御コンポーネントは車両の走行を制御します。

**Autoware**

Autowareは、オープンソースの自動運転ソフトウェアプラットフォームです。このプラットフォームは、自動運転システムを開発するためのモジュールコレクションを提供します。

| 名前                       | タイプ                                             | 説明          |
| -------------------------- | ------------------------------------------------ | -------------------- |
| `~/output/objects`         | `autoware_perception_msgs::msg::DetectedObjects` | 検出されたオブジェクト     |
| `debug/cyclic_time_ms`     | `tier4_debug_msgs::msg::Float64Stamped`          | サイクルタイム (msg)    |
| `debug/processing_time_ms` | `tier4_debug_msgs::msg::Float64Stamped`          | 処理時間 (ms) |

## パラメータ

### MLモデルパラメータ

これらのパラメータはONNXファイルに関連づけられており、学習フェーズ中に事前定義されます。このパラメータを変更するときは、ONNXファイルも必ず変更してください。また、ONNXファイルを更新する際には、常にこれらの値を確認することを忘れないでください。

| 名前                                   | タイプ         | デフォルト値                                     | 説明                                                                 |
| -------------------------------------- | ------------ | ------------------------------------------------ | --------------------------------------------------------------------- |
| `model_params.class_names`             | string配列 | ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"] | モデル出力のクラス名リスト                                           |
| `model_params.point_feature_size`      | int          | `4`                                              | 点群内の1点あたりの特徴量の数                                      |
| `model_params.max_voxel_size`          | int          | `40000`                                          | 最大ボクセル数                                                       |
| `model_params.point_cloud_range`       | double配列 | [-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]            | 検出範囲 [min_x, min_y, min_z, max_x, max_y, max_z] [m]             |
| `model_params.voxel_size`              | double配列 | [0.32, 0.32, 10.0]                               | ボクセルごとのサイズ [x, y, z] [m]                                  |
| `model_params.downsample_factor`       | int          | `1`                                              | 座標のダウンサンプル係数                                           |
| `model_params.encoder_in_feature_size` | int          | `9`                                              | エンコーダーへの入力特徴量の数                                       |
| `model_params.has_variance`            | bool         | `false`                                          | モデルが各バウンディングボックスの姿勢の分散と姿勢の出力を含む場合に真 |
| `model_params.has_twist`               | bool         | `false`                                          | モデルが各バウンディングボックスの速度と姿勢の出力を含む場合に真     |

### 主要パラメータ

| 名前                                             | タイプ         | デフォルト値 | 説明                                                       |
| ------------------------------------------------ | ------------ | ------------ | ------------------------------------------------------------ |
| `encoder_onnx_path`                              | 文字列       | `""`        | VoxelFeatureEncoder ONNX ファイルへのパス                    |
| `encoder_engine_path`                            | 文字列       | `""`        | VoxelFeatureEncoder TensorRT エンジンのファイルへのパス        |
| `head_onnx_path`                                 | 文字列       | `""`        | DetectionHead ONNX ファイルへのパス                          |
| `head_engine_path`                               | 文字列       | `""`        | DetectionHead TensorRT Engine のファイルへのパス              |
| `build_only`                                     | ブール       | `false`     | TensorRT エンジンのファイルが作成された後にノードをシャットダウンする |
| `trt_precision`                                  | 文字列       | `fp16`      | TensorRT 推論の精度: `fp32` または `fp16`                     |
| `post_process_params.score_threshold`            | double       | `0.4`       | スコアがしきい値より小さい検出されたオブジェクトは無視される |
| `post_process_params.yaw_norm_thresholds`        | リスト[double] | `[0.3, 0.3, 0.3, 0.3, 0.0]` | ヤオのノルムの距離しきい値の配列[rad]                               |
| `post_process_params.iou_nms_target_class_names` | リスト[文字列] | -           | IoU ベースの非最大抑制のターゲットクラス                      |
| `post_process_params.iou_nms_search_distance_2d` | double       | -           | 2 つのオブジェクトがこの値より離れている場合、NMS は適用されません |
| `post_process_params.iou_nms_threshold`          | double       | -           | IoU ベースの非最大抑制の IoU しきい値                         |
| `post_process_params.has_twist`                  | ブール       | `false`     | モデルが出力値を示すかどうか                                 |
| `densification_params.world_frame_id`            | 文字列       | `map`       | 複数フレームの点群を融合する世界フレームの ID                 |
| `densification_params.num_past_frames`           | 整数         | `1`         | 現在のフレームと融合する過去のフレームの数                    |

### `build_only` オプション

`autoware_lidar_centerpoint` ノードには、ONNX ファイルから TensorRT エンジンファイルを構築するための `build_only` オプションがあります。  Autoware ユニバースの `.param.yaml` ファイル内のすべての ROS パラメータを移動することを推奨しますが、`build_only` オプションは、構築を事前タスクとして実行するフラグとして使用できるため、現時点では `.param.yaml` ファイルに移動されていません。次のコマンドで実行できます。


```bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=/home/autoware/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix autoware_lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
```

## 前提条件 / 制限事項

- `object.existence_probability` には DNN の分類信頼度の値が格納されており、確率ではありません。

## 学習済みモデル

以下のリンクをクリックして、学習済みモデルの onnx 形式をダウンロードできます。

- Centerpoint: [pts_voxel_encoder_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx), [pts_backbone_neck_head_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx)
- Centerpoint tiny: [pts_voxel_encoder_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx), [pts_backbone_neck_head_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx)

`Centerpoint` は `nuScenes` (~28,000 lidar フレーム) [8] と TIER IV の内部データベース (~11,000 lidar フレーム) で 60 エポック学習されました。
`Centerpoint tiny` は `Argoverse 2` (~110,000 lidar フレーム) [9] と TIER IV の内部データベース (~11,000 lidar フレーム) で 20 エポック学習されました。

## CenterPoint モデルの学習と Autoware へのデプロイ

### 概要

このガイドでは、**mmdetection3d** リポジトリを使用して CenterPoint モデルを学習し、Autoware へシームレスにデプロイする方法について説明します。

### インストール

#### 前提条件のインストール

**ステップ 1.** [公式サイト](https://mmpretrain.readthedocs.io/en/latest/get_started.html) から Miniconda をダウンロードしてインストールします。

**ステップ 2.** Conda 仮想環境を作成してアクティベートします。


```bash
conda create --name train-centerpoint python=3.8 -y
conda activate train-centerpoint
```

**ステップ 3.** PyTorch のインストール

PyTorch がすでにインストールされていて、CUDA 11.6 と互換性があることを確認してください。これは、最新の Autoware の要件です。


```bash
conda install pytorch==1.13.1 torchvision==0.14.1 pytorch-cuda=11.6 -c pytorch -c nvidia
```

#### mmdetection3dのインストール

**ステップ 1.** MIMを使用してMMEngine、MMCV、MMDetectionをインストールする


```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**ステップ 2.** mmdetection3d の分岐リポジトリのインストール

mmdetection3d リポジトリのフォークにいくつかの有益な機能強化を導入しました。
特に、オリジナルの論文との互換性を維持するために、PointPillar z ボクセル特徴量入力をオプションにしました。
さらに、機能を追加するために PyTorch から ONNX へのコンバータと T4 形式リーダーを統合しました。


```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### Dockerを使用してTraining Repositoryを使用する

または、Dockerを使用してmmdetection3dリポジトリを実行することもできます。Dockerfileを使用してmmdetection3dリポジトリとその依存関係を持つDockerイメージを構築します。

mmdetection3dリポジトリのforkをクローンします


```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
```

以下のコマンドを実行して Docker イメージを構築します。


```bash
cd mmdetection3d
docker build -t mmdetection3d -f docker/Dockerfile .
```

Dockerコンテナを実行します:


```bash
docker run --gpus all --shm-size=8g -it -v {DATA_DIR}:/mmdetection3d/data mmdetection3d
```

### NuScenes データセットをトレーニング用に準備する

**手順 1.** [公式サイト](https://www.nuscenes.org/download) から NuScenes データセットをダウンロードし、任意のフォルダに展開します。

**注意:** NuScenes データセットは大きく、ディスク容量を多く必要とします。ダウンロードする前に十分な容量があることを確認してください。

**手順 2.** データセットフォルダへのシンボリックリンクを作成する


```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**ステップ 3.** NuScenesデータを準備します。次のコマンドを実行します。


```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes
```

### NuScenes データセットを使用した CenterPoint のトレーニング

#### Config ファイルの準備

CenterPoint モデルを NuScenes データセットでトレーニングするための設定を記載した設定ファイルは、`mmdetection3d/projects/AutowareCenterPoint/configs` に位置しています。この設定ファイルは、[この mmdetection3D からの CenterPoint 設定ファイル](https://github.com/autowarefoundation/mmdetection3d/blob/5c0613be29bd2e51771ec5e046d89ba3089887c7/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py) の派生バージョンです。
このカスタム設定では、**use_voxel_center_z パラメータ** が **False** に設定され、ボクセル中心の z 座標が無効になり、元の論文の仕様に沿ってモデルが Autoware と互換性を持つようになります。さらに、フィルタサイズは **[32, 32]** に設定されます。

CenterPoint モデルは、設定ファイル内のさまざまなパラメーターを変更することで、特定の要件に合わせて調整できます。
これには、前処理操作、トレーニング、テスト、モデルアーキテクチャ、データセット、オプティマイザー、学習率スケジューラなどに関連する調整が含まれます。

#### トレーニングの開始


```bash
python tools/train.py projects/AutowareCenterPoint/configs/centerpoint_custom.py --work-dir ./work_dirs/centerpoint_custom
```

#### トレーニング済みモデルの評価

評価用に、以下を含む車両から取得したサンプルデータセットを用意しました。
LiDARセンサー：1 x ベロダイン VLS128、4 x ベロダイン VLP16、1 x ロボセンス RS Bpearl
このデータセットには 600 個の LiDAR フレームが含まれ、5 つの異なるクラス、6905 台の乗用車、3951 人の歩行者、75 人の自転車利用者、162 台のバス、326 台のトラックの 3D アノテーションが含まれています。サンプルデータセットでは、フレームは 1 秒間に 2 フレームとしてアノテーションされています。このデータセットは、モデルのトレーニング、評価、微調整など、さまざまな目的に使用できます。T4 フォーマットで構成されています。

##### サンプルデータセットのダウンロード


```bash
wget https://autoware-files.s3.us-west-2.amazonaws.com/dataset/lidar_detection_sample_dataset.tar.gz
#Extract the dataset to a folder of your choice
tar -xvf lidar_detection_sample_dataset.tar.gz
#Create a symbolic link to the dataset folder
ln -s /PATH/TO/DATASET/ /PATH/TO/mmdetection3d/data/tier4_dataset/
```

##### データセットの準備とトレーニング済みモデルの評価

トレーニング、評価、テスト用の`.pkl`ファイルを作成します。

データセットはT4Datasetの仕様に従ってフォーマットされ、バージョンの1つとして「sample_dataset」が指定されました。


```bash
python tools/create_data.py T4Dataset --root-path data/sample_dataset/ --out-dir data/sample_dataset/ --extra-tag T4Dataset --version sample_dataset --annotation-hz 2
```

***評価の実行***


```bash
python tools/test.py projects/AutowareCenterPoint/configs/centerpoint_custom_test.py /PATH/OF/THE/CHECKPOINT  --task lidar_det
```

評価結果が低くなる可能性があります。サンプルデータセットとトレーニングデータセット間のセンサー方式が異なるためです。モデルのトレーニングパラメータは元々、車両の上部に1つのLiDARセンサーを使用するNuScenesデータセット用に調整されています。一方、提供されるサンプルデータセットには、車両のベースリンク位置に配置された連結された点群が含まれています。

### AutowareへのCenterPointモデルのデプロイ

#### PyTorch製のCenterPointモデルをONNXフォーマットに変換する

`autoware_lidar_centerpoint`実装では、ボクセルエンコーダとCenterPointモデルのバックボーン/ネック/ヘッドの2つのONNXモデルが必要で、前処理処理などのネットワークの他の側面は外部で実装されます。`mmdetection3d`リポジトリのフォークでは、CenterPointモデルをAutoware互換のONNXフォーマットに変換するスクリプトを含められています。`mmdetection3d/projects/AutowareCenterPoint`ファイルで見つけることができます。


```bash
python projects/AutowareCenterPoint/centerpoint_onnx_converter.py --cfg projects/AutowareCenterPoint/configs/centerpoint_custom.py --ckpt work_dirs/centerpoint_custom/YOUR_BEST_MODEL.pth --work-dir ./work_dirs/onnx_models
```

#### カスタムモデルの設定ファイルを作成

autoware_lidar_centerpointノードの設定ファイルディレクトリに、**centerpoint_custom.param.yaml**という新しい設定ファイルを作成します。設定ファイルのパラメータ（point_cloud_range、point_feature_size、voxel_sizeなど）をトレーニング設定ファイルに合わせて設定します。


```yaml
/**:
  ros__parameters:
    class_names: ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"]
    point_feature_size: 4
    max_voxel_size: 40000
    point_cloud_range: [-51.2, -51.2, -3.0, 51.2, 51.2, 5.0]
    voxel_size: [0.2, 0.2, 8.0]
    downsample_factor: 1
    encoder_in_feature_size: 9
    # post-process params
    circle_nms_dist_threshold: 0.5
    iou_nms_target_class_names: ["CAR"]
    iou_nms_search_distance_2d: 10.0
    iou_nms_threshold: 0.1
    yaw_norm_thresholds: [0.3, 0.3, 0.3, 0.3, 0.0]
```

#### lidar_centerpoint ノードの起動


```bash
cd /YOUR/AUTOWARE/PATH/Autoware
source install/setup.bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml  model_name:=centerpoint_custom  model_path:=/PATH/TO/ONNX/FILE/
```

### Changelog

#### v1 (2022/07/06)

| 名前                 | URL                                                                                                        | 説明                                                                                                                                   |
| --------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `centerpoint`        | [pts_voxel_encoder][v1-encoder-centerpoint] <br> [pts_backbone_neck_head][v1-head-centerpoint]              | このパッケージの実装上の制限のため、1つの変更があります。`PillarFeatureNet`の`num_filters=[32, 32]`                               |
| `centerpoint_tiny` | [pts_voxel_encoder][v1-encoder-centerpoint-tiny] <br> [pts_backbone_neck_head][v1-head-centerpoint-tiny] | `v0`の`default`と同じモデル                                                                                                                |

これらの変更は次の構成と比較されています: [この構成](https://github.com/tianweiy/CenterPoint/blob/v0.2/configs/waymo/pp/waymo_centerpoint_pp_two_pfn_stride1_3x.py)。

#### v0 (2021/12/03)

| 名称      | URL                                                                                   | 説明                                                                                                                                            |
| --------- | -------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `default` | [pts_voxel_encoder][v0-encoder-default] <br> [pts_backbone_neck_head][v0-head-default] | オリジナルのCenterPointアーキテクチャから2つ変更があります。`num_filters=[32]`の`PillarFeatureNet`と`ds_layer_strides=[2, 2, 2]`の`RPN` |

## (省略可能) エラーの検出と処理

<!-- エラーの検出方法と、それらからの復旧方法を記載します。

例:
  このパッケージは最大20個の障害物を処理できます。障害物がさらに多く見つかった場合、このノードは処理を中止し、診断エラーを発生させます。
-->

## (省略可能) パフォーマンスの特性

<!-- 複雑性などのパフォーマンス情報を記載します。ボトルネックにならない場合は、記載する必要はありません。

例:
  ### 複雑性

  このアルゴリズムはO(N)です。

  ### 処理時間

  ...
-->

## 参考文献/外部リンク

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).

[2] Lang, Alex H., et al. "PointPillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://github.com/tianweiy/CenterPoint>

[4] <https://github.com/open-mmlab/mmdetection3d>

[5] <https://github.com/open-mmlab/OpenPCDet>

[6] <https://github.com/yukkysaito/autoware_perception>

[7] <https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars>

[8] <https://www.nuscenes.org/nuscenes>

[9] <https://www.argoverse.org/av2.html>

## (省略可能) 今後の拡張機能 / 未実装部分

<!-- このパッケージの今後の拡張機能を記載します。

例:
  現在、このパッケージはチャタリング障害物への対応が適切ではありません。知覚層に確率的フィルタを追加して改善する予定です。
  また、グローバルであるべきパラメータがいくつかあります(例: 車両サイズ、最大操舵角など)。これらはリファクタリングされグローバルパラメータとして定義されるため、さまざまなノードで同じパラメータを共有できます。
-->

[v0-encoder-default]: https://awf.ml.dev.web.auto/perception/models/pts_voxel_encoder_default.onnx
[v0-head-default]: https://awf.ml.dev.web.auto/perception/models/pts_backbone_neck_head_default.onnx
[v1-encoder-centerpoint]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_voxel_encoder_centerpoint.onnx
[v1-head-centerpoint]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_backbone_neck_head_centerpoint.onnx
[v1-encoder-centerpoint-tiny]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_voxel_encoder_centerpoint_tiny.onnx
[v1-head-centerpoint-tiny]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_backbone_neck_head_centerpoint_tiny.onnx

## Acknowledgment: deepen.aiの3Dアノテーションツールへの貢献

## 法的通知

_nuScenesデータセットは、クリエイティブ・コモンズの帰属-非営利-継承4.0国際一般公開ライセンスに基づき、非営利目的での利用を目的として全般に公開されています。追加の利用条件は<https://www.nuscenes.org/terms-of-use>に掲載されています。商用ライセンスについてのお問い合わせは<nuscenes@motional.com>までお問い合わせください。_

