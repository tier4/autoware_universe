# autoware_euclidean_cluster

## 目的

autoware_euclidean_clusterは、点を小さな部分にクラスタリングしてオブジェクトを分類するためのパッケージです。

このパッケージには2つのクラスタリング方式があります。`euclidean_cluster`と`voxel_grid_based_euclidean_cluster`です。

## 内部動作/アルゴリズム

### euclidean_cluster

`pcl::EuclideanClusterExtraction`が点に適用されます。詳細は[公式ドキュメント](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html)を参照してください。

### voxel_grid_based_euclidean_cluster

1. 各ボクセルの重心は`pcl::VoxelGrid`によって計算されます。
2. 重心は`pcl::EuclideanClusterExtraction`によってクラスタリングされます。
3. 入力点はクラスタリングされた重心に基づいてクラスタリングされます。

## 入出力

### 入力

| 名前 | タイプ | 説明 |
|---|---|---|
| `input` | `sensor_msgs::msg::PointCloud2` | 入力ポイントクラウド |

### 出力

****Plannerの仕様**

**目的**

Plannerは、目的地の地点から現在の軌跡を生成します。

**インターフェース**

* **入力**
    * 自車位置
    * 地図
    * 目的地
    * `post resampling` の予測パス
    * 障害物
* **出力**
    * 経路計画された轨跡 (最適な轨跡)

**処理フロー**

1. 地図から経路を生成する。
2. 経路を予測軌跡に分割する。
3. 障害物を考慮して各セグメントを対応する車両ダイナミクスモデルに最適化する。
4. 各最適化されたセグメントを元の経路に再統合する。

**Planningモジュール**

Plannerは、以下を含む複数のモジュールで構成されています。

* **Path Planning:** グローバルな経路生成を担当。
* **Behavior Planning:** 局所的な軌跡生成を担当。
* **Motion Planning:** 実行可能な軌跡生成を担当。
* **Trajectory Optimization:** 軌跡の最適化を担当。

**Autowareとの統合**

Plannerは、Autowareの他のコンポーネントと通信し、以下を行います。

* **Localization:** 自車位置を取得。
* **Mapping:** 地図を取得。
* **Object Detection:** 障害物を検出。
* **Prediction:** 障害物の未来位置を予測。

**アルゴリズム**

Plannerは、以下を含むさまざまなアルゴリズムを使用しています。

* **A*:** 経路検索に使用。
* **DWA:** `post resampling` の予測パスに使用。
* **QP:** 軌跡最適化に使用。
* **MPC:** 予測制御に使用。

| 名前             | 型                                                      | 説明                                 |
| ---------------- | -------------------------------------------------------- | --------------------------------------- |
| `output`         | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | クラスターの点群                          |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2`                          | 可視化のための色付きクラスターの点群 |

## パラメータ

### コアパラメータ

#### euclidean_cluster

| 名前 | タイプ | 説明 |
|---|---|---|
| `use_height` | bool | `post resampling`されたクラスタリングに点のz座標を使用する |
| `min_cluster_size` | int | 有効とみなされるクラスタに含める必要がある最小点の数 |
| `max_cluster_size` | int | 有効とみなされるクラスタに含める必要がある最大点の数 |
| `tolerance` | float | L2ユークリッド空間での測定値としての空間クラスタ許容誤差 |

#### voxel_grid_based_euclidean_cluster

将来的にサポートされるであろうPlanningコンポーネント/モジュール用のeuclideanクラスタリングアルゴリズムのvoxel-based実装です。

| 名称                          | 型  | 説明                                                                             |
| ----------------------------- | ----- | ------------------------------------------------------------------------------------- |
| `use_height`                  | ブール  | `post resampling`用ポイントの`z`値を使用する                                           |
| `min_cluster_size`            | 整数 | クラスタで許容される最小ポイント数。これを下回るとクラスタは無効とみなされる       |
| `max_cluster_size`            | 整数 | クラスタで許容される最大ポイント数。これを超えるとクラスタは無効とみなされる       |
| `tolerance`                   | 浮動小数 | クラスタの空間許容値（L2ユークリッド空間での測定値）                            |
| `voxel_leaf_size`             | 浮動小数 | `x`軸と`y`軸のボクセルリーフサイズ                                              |
| `min_points_number_per_voxel` | 整数 | ボクセルごとの最小ポイント数                                                     |

## 仮定/既知の制限

<!-- 実装の仮定と制限を記述します。

例:
  このアルゴリズムでは障害物が移動しないと想定しているため、車両が障害物の回避を開始した後で障害物が急激に移動すると、衝突する可能性があります。
  また、このアルゴリズムは死角を考慮しません。一般的に、センシングの性能の制限により近すぎる障害物は見えないため、障害物に対して十分な余裕を取ってください。
-->

## (省略可能) エラーの検出と処理

<!-- エラーの検出方法と、エラーからの回復方法を記述します。

例:
  このパッケージは最大20個の障害物に対応できます。それ以上の障害物が検出された場合、このノードは処理を放棄し、診断エラーを発生させます。
-->

## (省略可能) パフォーマンス特性

<!-- 複雑性などのパフォーマンス情報を記述します。ボトルネックにならない場合は不要です。

例:
  ### 複雑性

  このアルゴリズムはO(N)です。

  ### 処理時間

  ...
-->

## (省略可能) 参照/外部リンク

<!-- 実装時に参照したリンクを記述します。

例:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (省略可能) 今後の拡張/未実装部分

`voxel_grid_based_euclidean_cluster`の`use_height`オプションは、まだ実装されていません。

