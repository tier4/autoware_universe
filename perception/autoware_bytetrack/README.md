# bytetrack

## 目的

`ByteTrack`と呼ばれるコアアルゴリズムは、主に複数オブジェクトのトラッキングを行います。
アルゴリズムは検出スコアが低いものも含むほぼすべての検出ボックスを関連付けるため、これを使用すると、偽陰性の数が減ると予想されます。

[デモ動画](https://github.com/YoshiRi/autoware.universe/assets/3022416/40f4c158-657e-48e1-81c2-8ac39152892d)

## 内部動作/アルゴリズム

### 引用

<!-- cspell: ignore Yifu Peize Jiang Dongdong Fucheng Weng Zehuan Xinggang -->

- Yifu Zhang, Peize Sun, Yi Jiang, Dongdong Yu, Fucheng Weng, Zehuan Yuan, Ping Luo, Wenyu Liu, and Xinggang Wang,
  "ByteTrack: Multi-Object Tracking by Associating Every Detection Box", in the proc. of the ECCV
  2022, [[ref](https://arxiv.org/abs/2110.06864)]
- このパッケージは[このレポジトリ](https://github.com/ifzhang/ByteTrack/tree/main/deploy/TensorRT/cpp)からAutoware向けに移植されたバージョンです (ByteTrackの著者のC++インプリメンテーション)。

### オリジナルコードからの2dトラッキングの変更

論文では、2dトラッキングアルゴリズムは単純なカルマンフィルタであるとだけ述べています。
オリジナルコードは、状態ベクトルとして`左上のコーナー`、`アスペクト比`、`サイズ`を使用します。

これは、アスペクト比がオクルージョンによって変化することがあるため、不安定な場合があります。
そのため、状態ベクトルとして`左上`と`サイズ`を使用します。

カルマンフィルタの設定は、`config/bytetrack_node.param.yaml`のパラメータで制御できます。

## 入出力

### bytetrack_node

#### 入力
- `/detection/lidar_tracking/objects`: Camera object detections. ([message](https://autoware.github.io/autoware.auto/latest/api/message__objects__proto.html))
- `/current_pose`: 自車位置 ([message](https://autoware.github.io/autoware.auto/latest/api/message__pose__proto.html))

#### 出力
- `/perception/object_tracking/objects`: Tracked objects. ([message](https://autoware.github.io/autoware.auto/latest/api/message__objects__proto.html))

| 名称      | タイプ                                               | 説明                                   |
| --------- | -------------------------------------------------- | ------------------------------------------ |
| `in/rect` | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2Dバウンディングボックス付き検出オブジェクト |

#### 出力

| 名前                   | タイプ                                               | 説明                                                |
| ------------------------ | -------------------------------------------------- | ------------------------------------------------------ |
| `out/objects`            | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2D境界ボックスを持つ検出オブジェクト                   |
| `out/objects/debug/uuid` | `tier4_perception_msgs/DynamicObjectArray`         | 各オブジェクトの一意の識別子（UUID）                     |

### bytetrack_visualizer

#### 入力

| 名前 | タイプ | 説明 |
|---|---|---|
| `in/image` | `sensor_msgs/Image` または `sensor_msgs/CompressedImage` | オブジェクト検出が実行される入力画像 |
| `in/rect` | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2D境界ボックスを持つ検出オブジェクト |
| `in/uuid` | `tier4_perception_msgs/DynamicObjectArray` | 各オブジェクトの一意の識別子 (UUID) |

#### 出力

[自動運転ソフトウェアのドキュメント](URL)

---

AutowareのPlanningモジュールは、周囲の状況を認識し、自車位置に基づいて将来の経路を生成する、車両自律運転における重要なコンポーネントである。

このドキュメントでは、Planningモジュールのアーキテクチャ、アルゴリズム、実装の詳細について説明する。

## Planningモジュールのアーキテクチャ

Planningモジュールは、以下のサブモジュールで構成される。

- **Perception:** センサーデータから周囲の状況を認識する。
- **Planning:** Perceptionモジュールからの情報に基づいて将来の経路を生成する。
- **Prediction:** 周囲の車両や歩行者の動きを予測する。
- **Optimization:** 生成された経路を最適化する。

## アルゴリズム

Planningモジュールは、以下のアルゴリズムを使用して将来の経路を生成する。

- **動的計画法:** 最適な経路を見つけるための効率的なアルゴリズム。
- **モデル予測制御:** 未来の車両の動きを予測し、それらに応じて経路を調整する手法。
- **畳み込みニューラルネットワーク:** 交通状況から潜在的な障害物を検出する。

## 実装の詳細

Planningモジュールは、C++で実装されている。以下は、主なクラスと関数のリストである。

- `Planner`: Planningモジュールのメインインターフェース。
- `PathPlanner`: 将来の経路を生成する。
- `PredictionModule`: 周囲の車両や歩行者の動きを予測する。
- `Optimizer`: 生成された経路を最適化する。

## 使用例

Planningモジュールは、Autowareの自動運転システムで広く使用されている。以下は、その使用例のいくつかである。

- `post resampling`後の経路の再計画。
- 交差点での衝突回避。
- 車線変更の計画。

| 名前        | タイプ                | 説明                                                             |
| ----------- | ------------------- | ------------------------------------------------------------------- |
| `out/image` | `sensor_msgs/Image` | 検出境界ボックスと UUID が描画された画像                                |

## パラメーター

### bytetrack_node

| 名前                  | タイプ | デフォルト値 | 説明                                                      |
| --------------------- | ---- | ----------- | --------------------------------------------------------- |
| `track_buffer_length` | int  | 30            | trackletが消失していると見なされるフレーム数 |

### bytetrack_visualizer

| 名前      | 種類 | デフォルト値 | 説明                                                                                    |
| --------- | ---- | ------------- | --------------------------------------------------------------------------------------------- |
| `use_raw` | bool | false         | ノードが `sensor_msgs/Image` を `sensor_msgs/CompressedImage` に変換するかどうかを示すフラグ |

## 仮定と既知の制限

## 参照レポジトリ

- <https://github.com/ifzhang/ByteTrack>

## ライセンス

`lib` ディレクトリ内のコードは [元のコード](https://github.com/ifzhang/ByteTrack/tree/72ca8b45d36caf5a39e949c6aa815d9abffd1ab5/deploy/TensorRT/cpp) からコピーされ、修正されています。
元のコードは MIT ライセンスに準拠しており、下記のように記載されていますが、この移植パッケージには Apache License 2.0 が付与されています。

> MIT ライセンス
>
> Copyright (c) 2021 Yifu Zhang
>
> 本ソフトウェアおよび関連するドキュメンテーション ファイル（「ソフトウェア」）の写しを取得したすべての人に、使用、コピー、修正、マージ、発行、配布、サブライセンス、および/または本ソフトウェアのコピーを販売し、本ソフトウェアが提供される人にそのようにすることを許可する権利を含め、制限なく本ソフトウェアを扱うことが、以下に記載する条件に従って無償で許可されます。
>
> 上記の著作権表示と本許可表示は、本ソフトウェアのすべての写しまたは実質的な部分に含まれます。
>
> 本ソフトウェアは、商品性、特定目的への適合性、非侵害性などの保証を含め、明示的または黙示的を問わず、いかなる種類の保証も行わずに「現状のまま」提供されます。著作者または著作権者は、契約、不法行為、その他にかかわらず、本ソフトウェアまたは本ソフトウェアの使用または他の取引に起因または関連して生じた請求、損害、またはその他の責任について、一切の責任を負いません。

