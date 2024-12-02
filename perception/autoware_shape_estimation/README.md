# autoware_shape_estimation

## 目的

このノードは、ラベルに従ってポイントクラウドクラスタが合う洗練されたオブジェクト形状（バウンディングボックス、円筒、凸包）を計算します。

## 内部動作/アルゴリズム

### フィッティングアルゴリズム

- バウンディングボックス

  - L字型フィッティング：詳細は以下のリファレンスを参照してください
  - MLベースの形状フィッティング：詳細は以下のMLベースの形状フィッティング実装セクションを参照してください

- 円筒

  `cv::minEnclosingCircle`

- 凸包

  `cv::convexHull`

## 入出力

### 入力

| 名称    | タイプ                                                     | 説明                           |
| ------- | -------------------------------------------------------- | ------------------------------------- |
| `input` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ラベル済みクラスタを持つ検出对象 |

### 出力
Autowareでは、Planningコンポーネントが、センサー入力から生成された周囲環境の認識結果を使用して、車両の経路と速度プロフィールを決定します。Planningコンポーネントは、次のサブモジュールで構成されます。

- **Motion Planner：**障害物を避けながら、現在の位置から目標地点への経路を生成します。
- **Behavior Planner：**経路に沿って車両の速度と加速度を決定します。
- **Trajectory Generator：**モーションプランとビヘイビアプランの出力を基に、車両の軌跡**post resampling**を生成します。

| 名称             | タイプ                                                      | 説明                                  |
| ---------------- | --------------------------------------------------------- | ------------------------------------- |
| `output/objects` | `autoware_perception_msgs::msg::DetectedObjects` | 洗練された形状を備えた検出された対象 |

## パラメータ

{{ json_to_markdown("perception/autoware_shape_estimation/schema/shape_estimation.schema.json") }}

## MLベース形状実装

モデルはポイントクラウドとオブジェクトラベル（カメラ検出/Apolloインスタンスセグメンテーションによって提供される）を入力として、オブジェクトの3Dバウンディングボックスを出力します。

MLベース形状推定アルゴリズムは、PointNetモデルをバックボーンとして使用してオブジェクトの3Dバウンディングボックスを推定します。モデルは、ヌシーンズデータセットの車両ラベル（車、トラック、バス、トレーラー）でトレーニングされます。

実装されたモデルは、入力ポイントクラウドを正規空間に変換するためのSTN（空間変換ネットワーク）と、オブジェクトの3Dバウンディングボックスを予測するためのPointNetと連結されています。バウンディングボックス推定の部分は「RGB-Dデータからの3Dオブジェクト検出のためのFrustum PointNets」論文を参考にしています。

モデルは、各オブジェクトに対して次の出力を予測します。

- オブジェクト中心のx、y、z座標
- オブジェクトの進行方向角度の分類結果（角度の分類に12ビンを使用 - それぞれ30度）
- オブジェクトの進行方向角度の残差
- オブジェクトサイズの分類結果
- オブジェクトサイズの残差

### MLベース形状推定モデルのトレーニング

モデルをトレーニングするには、3Dバウンディングボックスのグラウンドトゥルースアノテーションが必要です。mmdetection3dリポジトリを使用して3Dオブジェクト検出アルゴリズムをトレーニングする場合、これらのグラウンドトゥルースアノテーションは保存され、データ拡張に使用されます。これらのアノテーションは、形状推定モデルを効果的にトレーニングするための不可欠なデータセットとして使用されます。

### データセットの準備

#### MMDetection3Dの前提条件のインストール

**ステップ1.** [公式サイト](https://mmpretrain.readthedocs.io/en/latest/get_started.html)からMinicondaをダウンロードしてインストールします。

**ステップ2.** conda仮想環境を作成し、アクティベートします。


```bash
conda create --name train-shape-estimation python=3.8 -y
conda activate train-shape-estimation
```

**ステップ 3.** PyTorch をインストール


```bash
conda install pytorch torchvision -c pytorch
```

#### mmdetection3dのインストール

**手順1.** MIMを使用してMMEngine、MMCV、MMDetectionをインストールする


```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**ステップ 2.** Autoware の MMDetection3D フォークをインストール


```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### NuScenes データセットの準備

**ステップ 1.** [公式ウェブサイト](https://www.nuscenes.org/download) から NuScenes データセットをダウンロードし、任意のフォルダに解凍します。

**注意:** NuScenes データセットは大きく、大量のディスク容量が必要です。追加する前に、使用可能な記憶容量が十分にあることを確認してください。

**ステップ 2.** データセットフォルダへのシンボリックリンクを作成する


```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**ステップ3.** 次を実行して、**NuScenes**データを準備します:


```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes --only-gt-database True
```

#### クローン バウンディング ボックス エスティメーター モデル




```bash
git clone https://github.com/autowarefoundation/bbox_estimator.git
```

#### データセットをトレーニングと検証のセットに分割する


```bash

cd bbox_estimator
python3 utils/split_dbinfos.py --dataset_path /path/to/mmdetection3d/data/nuscenes --classes 'car' 'truck' 'bus' 'trailer'  --train_ratio 0.8
```

### モデルのトレーニングと展開

#### モデルのトレーニング


```bash
# Detailed training options can be found in the training script
# For more details, run `python3 train.py --help`
python3 train.py --dataset_path /path/to/mmdetection3d/data/nuscenes
```

#### モデル展開


```bash
# Convert the trained model to ONNX format
python3 onnx_converter.py --weight_path /path/to/best_checkpoint.pth --output_path /path/to/output.onnx
```

`shape_estimation` ノード起動ファイルの `model_path` パラメータに ONNX モデルの出力パスを指定してください。

## 想定事項/既知の制限

未定

## 用語/外部リンク

論文に基づいた L 形フィッティングの実装:


```bibtex
@conference{Zhang-2017-26536,
author = {Xiao Zhang and Wenda Xu and Chiyu Dong and John M. Dolan},
title = {Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners},
booktitle = {2017 IEEE Intelligent Vehicles Symposium},
year = {2017},
month = {June},
keywords = {autonomous driving, laser scanner, perception, segmentation},
}
```

**RGB-Dデータからの3Dオブジェクト検出のためのFrustum PointNets**

```
## Autoware.Autoで Frustum PointNets を使用して 3D オブジェクトを検出する

### 概要

このドキュメントでは、Autoware.Auto で Frustum PointNets を使用して 3D オブジェクトを検出する方法について説明します。Frustum PointNets は、点群データから 3D オブジェクトを検出するための最先端の手法です。

### アーキテクチャ

Frustum PointNets アーキテクチャは、以下のコンポーネントで構成されています。

- **PointNet++ バックボーン:** 入力点群を処理し、特徴量を抽出します。
- **フラストム プロジェクション:** 点群を前方カメラの視野 (`フラストム`) に投影します。
- **フラストム ポイントネット:** 投影された点群から 3D オブジェクト候補を検出します。
- **後処理:** 非極大値抑制 (`post resampling`) を使用して、重複する候補を削除します。

### パイプライン

3D オブジェクト検出パイプラインは、次のステップで構成されています。

1. **点群の前処理:** 点群をノイズ除去および正規化します。
2. **フラストム生成:** 前方カメラの視点からフラストムを生成します。
3. **Frustum PointNets:** フラストムに Frustum PointNets を適用して、3D オブジェクト候補を検出します。
4. **後処理:** 重複する候補を削除します。
5. **3D バウンディングボックスの生成:** 検出されたオブジェクト候補に 3D バウンディングボックスを割り当てます。

### パラメータ

Frustum PointNets のパラメータは、`config/perception/detection/frustum_pointnets.yaml` ファイルで設定できます。重要なパラメータを以下に示します。

- **num_classes:** 検出するオブジェクトクラスの数
- **num_proposal:** 生成するオブジェクト候補の最大数
- **voxel_size:** ポイントクラウドのボクセル化に使用されるサイズ
- **post_resampling_NMS:** 非極大値抑制の閾値

### 結果

Frustum PointNets は、Autoware.Auto で 3D オブジェクトを検出するための効率的かつ正確な手法です。この手法を使用すると、高精度な 3D バウンディングボックスを生成できます。

### その他のリソース

- [Frustum PointNets 論文](https://arxiv.org/abs/1704.06056)
- [Autoware.Auto プルリクエスト](https://github.com/autowarefoundation/autoware.auto/pull/2434)
```


````bibtex
@inproceedings{qi2018frustum,
title={Frustum pointnets for 3d object detection from rgb-d data},
author={Qi, Charles R and Liu, Wei and Wu, Chenxia and Su, Hao and Guibas, Leonidas J},
booktitle={Proceedings of the IEEE conference on computer vision and pattern recognition},
pages={918--927},
year={2018}
}```


````

