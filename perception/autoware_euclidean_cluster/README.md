# autoware_euclidean_cluster

## 目的

autoware_euclidean_cluster は、オブジェクトを分類するために点群をより小さな部分にクラスタリングするパッケージです。

このパッケージには、`euclidean_cluster` と `voxel_grid_based_euclidean_cluster` の 2 つのクラスタリング手法があります。

## 仕組み / アルゴリズム

### euclidean_cluster

`pcl::EuclideanClusterExtraction` を点群に適用します。詳細については、[公式ドキュメント](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html) を参照してください。

### voxel_grid_based_euclidean_cluster

1. ボクセルごとの重心は、`pcl::VoxelGrid` によって計算されます。
2. 重心は `pcl::EuclideanClusterExtraction` によってクラスタリングされます。
3. 入力点群は、クラスタリングされた重心を基にクラスタリングされます。

## 入出力

### 入力

| 名前    | タイプ                          | 説明     |
| ------- | ------------------------------- | -------- |
| `input` | `sensor_msgs::msg::PointCloud2` | 入力点群 |

### 出力

**自動運転ソフトウェア**

**ドキュメント URL:** [ここにURLを入力]

**Planningモジュール**

**Planningモジュールの機能**

Planningモジュールには、以下の機能があります。

- 車両の経路計画
- 車両の衝突回避
- 車両の制御

**Planningモジュールの入力**

Planningモジュールには、以下の入力があります。

- 自車位置
- 自車速度
- 周囲環境の認識データ
- 交通規制情報

**Planningモジュールの出力**

Planningモジュールの出力には、以下のものがあります。

- 車両の目標経路
- 車両の加速度・旋回角率の目標値

**Planningモジュールにおける技術的詳細**

Planningモジュールは、以下の技術的詳細を備えています。

- 経路計画アルゴリズム: ダイナミックウェイポイント法
- 障害物回避アルゴリズム: 最適制御法
- `post resampling`技術を使用したセンサデータの処理
- velocity逸脱量とacceleration逸脱量の最小化

Planningモジュールは、Autowareソフトウェアスタックの一部です。

| 名               | タイプ                                                   | 説明                               |
| ---------------- | -------------------------------------------------------- | ---------------------------------- |
| `output`         | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | クラスタ点群                       |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2`                          | 可視化のための着色済みクラスタ点群 |

## パラメータ

### コアパラメータ

#### euclidean_cluster

| 名称               | タイプ     | 説明                                                          |
| ------------------ | ---------- | ------------------------------------------------------------- |
| `use_height`       | ブール     | クラスタリングに point.z を使用                               |
| `min_cluster_size` | 整数       | クラスターが有効と見なされるために必要な最小のポイント数      |
| `max_cluster_size` | 整数       | クラスターが有効と見なされるために必要な最大のポイント数      |
| `tolerance`        | 浮動小数点 | L2 ユークリッド空間における尺度としての空間クラスター許容範囲 |

#### voxel_grid_based_euclidean_cluster

このノードは、voxelグリッド法に基づくEuclideanクラスタリングを使用して、点群をクラスタリングします。このノードの主な目的は、障害物検出のパイプラインのために、点群を障害物クラスタにクラスタリングすることです。

**サブスクライブするトピック**

- `/points_raw`: 入力点群

**パブリッシュするトピック**

- `/voxel_grid_based_euclidean_cluster/output`: クラスタリングされた点群

**パラメータ**

- **voxel_leaf_size:** クラスタリングに使用するボクセルサイズ[m]
- **euclidean_cluster_tolerance:** クラスタリングに使用するユークリッド距離公差[m]
- **min_cluster_size:** クラスタとして認識されるためにクラスタに含まれる必要のある最小ポイント数
- **max_cluster_size:** 1つのクラスタに含まれることができる最大ポイント数
- **post resampling:** 点群をサンプルするのかどうか (True/False)
- **sample_size:** サンプルするポイントの数。`post resampling` が True の場合にのみ使用されます。
- **target_frame:** クラスタリングされた点群の目標フレーム。`current pose` または `sensor` のいずれか。

| 名称                          | 型    | 説明                                                  |
| ----------------------------- | ----- | ----------------------------------------------------- |
| `use_height`                  | bool  | クラスタリングに point.z を使用する                   |
| `min_cluster_size`            | int   | 有効とみなされるためにクラスタが必要とする最小点の数  |
| `max_cluster_size`            | int   | 有効とみなされるためにクラスタが必要とする最大点の数  |
| `tolerance`                   | float | L2 ユークリッド空間の指標としての空間クラスタ許容誤差 |
| `voxel_leaf_size`             | float | x と y のボクセルリーフサイズ                         |
| `min_points_number_per_voxel` | int   | ボクセルに必要な最低点の数                            |

## 想定/既知の制限

<!-- 想定事項と実装上の制約を記載。

例:
  このアルゴリズムは障害物が静止していることを前提としているため、車両が回避を開始した後で急激に移動した場合、障害物に衝突する可能性があります。
  また、このアルゴリズムはブラインドスポットを考慮しません。一般的に、近すぎる障害物はセンサーの性能上の制約により可視化できないため、障害物に対して十分な余裕を取ってください。
-->

## (省略可) エラーの検出と処理

<!-- エラーの検出方法と回復方法を記載。

例:
  このパッケージでは最大20個の障害物に対処できます。障害物がそれ以上見つかった場合、このノードは処理を放棄し、診断エラーを発生させます。
-->

## (省略可) パフォーマンスの特性評価

<!-- 複雑度などのパフォーマンス情報を記載。ボトルネックにならない場合、記載不要。

例:
  ### 複雑度

  このアルゴリズムはO(N)です。

  ### 処理時間

  ...
-->

## (省略可) リファレンス/外部リンク

<!-- 実装時に参考にしたリンクを記載。

例:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (省略可) 今後の拡張機能/未実装部分

`voxel_grid_based_euclidean_cluster`の`use_height`オプションはまだ実装されていません。
