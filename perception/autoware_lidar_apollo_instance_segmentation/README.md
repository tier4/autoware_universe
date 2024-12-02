**autoware_lidar_apollo_instance_segmentation**

![Peek 2020-04-07 00-17](https://user-images.githubusercontent.com/8327598/78574862-92507d80-7865-11ea-9a2d-56d3453bdb7a.gif)

## 目的

このノードは、CNNベースのモデルと障害物クラスタリング方法に基づいて、LiDARセンサーからの3D点群データを、自動車、トラック、自転車、歩行者などの障害物にセグメントします。

## 内部動作/アルゴリズム

Apolloによる[オリジナルデザイン](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/3d_obstacle_perception.md)を参照してください。

## 入出力

### 入力
- `points2_apollo` (`SensorMeasurements`): LiDARポイントデータ
- `current pose` (`geometry_msgs/Pose`): 自車位置
- `polygon` (`autoware_msgs/DynamicPolygonArray`): 交通規制情報(オプション)

### 出力
- `detected_objects` (`autoware_msgs/DetectedObjectsArray`): 検出された障害物情報

### パラメータ

`Voxel Grid Filter`
- `voxel_grid_size` (`double`): ボクセルグリッドサイズ [m]
- `voxel_grid_leaf_size` (`double`): ボクセルグリッドリーフサイズ [m]

`Voxel Feature Extractor`
- `voxel_feature_extractor.use_height` (`bool`): 高さを利用するかどうか
- `voxel_feature_extractor.use_intensity` (`bool`): 強度を利用するかどうか
- `voxel_feature_extractor.use_normal` (`bool`): 法線を利用するかどうか
- `voxel_feature_extractor.use_shape` (`bool`): 形状を利用するかどうか
- `voxel_feature_extractor.use_semantic` (`bool`): セマンティックを利用するかどうか
- `voxel_feature_extractor.intensity_threshold` (`double`): 強度閾値
- `voxel_feature_extractor.shape_threshold` (`double`): 形状閾値
- `voxel_feature_extractor.semantic_threshold` (`double`): セマンティック閾値

`Frustum Point Nets`
- `frustum_pointnets.num_output_features` (`int`): 出力フィーチャーの数
- `frustum_pointnets.use_auxiliary_loss` (`bool`): 補助損失を使用するかどうか
- `frustum_pointnets.use_center_loss` (`bool`): 中心損失を使用するかどうか
- `frustum_pointnets.use_size_loss` (`bool`): サイズ損失を使用するかどうか
- `frustum_pointnets.use_heading_loss` (`bool`): ヘディング損失を使用するかどうか
- `frustum_pointnets.use_semantic_loss` (`bool`): セマンティック損失を使用するかどうか
- `frustum_pointnets.use_rotation_loss` (`bool`): 回転損失を使用するかどうか
- `frustum_pointnets.use_translation_loss` (`bool`): 並進損失を使用するかどうか
- `frustum_pointnets.use_velocity_loss` (`bool`): 速度損失を使用するかどうか
- `frustum_pointnets.center_loss_weight` (`double`): センター損失の重み
- `frustum_pointnets.size_loss_weight` (`double`): サイズ損失の重み
- `frustum_pointnets.heading_loss_weight` (`double`): ヘディング損失の重み
- `frustum_pointnets.semantic_loss_weight` (`double`): セマンティック損失の重み
- `frustum_pointnets.rotation_loss_weight` (`double`): 回転損失の重み
- `frustum_pointnets.translation_loss_weight` (`double`): 並進損失の重み
- `frustum_pointnets.velocity_loss_weight` (`double`): 速度損失の重み

`Clustering`
- `clustering.min_pts` (`int`): クラスタリングにおける最小点数
- `clustering.max_pts` (`int`): クラスタリングにおける最大点数
- `clustering.min_x` (`double`): クラスタリングにおける最小x座標 [m]
- `clustering.max_x` (`double`): クラスタリングにおける最大x座標 [m]
- `clustering.min_y` (`double`): クラスタリングにおける最小y座標 [m]
- `clustering.max_y` (`double`): クラスタリングにおける最大y座標 [m]
- `clustering.min_z` (`double`): クラスタリングにおける最小z座標 [m]
- `clustering.max_z` (`double`): クラスタリングにおける最大z座標 [m]

`Center Filter`
- `center_filter.min_distance` (`double`): センターフィルターにおける最小距離 [m]
- `center_filter.max_distance` (`double`): センターフィルターにおける最大距離 [m]
- `center_filter.max_center_count` (`int`): センターフィルターにおける最大センター数
- `center_filter.max_point_distance` (`double`): センターフィルターにおける最大ポイント距離 [m]
- `center_filter.max_point_count` (`int`): センターフィルターにおける最大ポイント数

### 補足情報

- このノードは、ベロダイナの16チャンネルデータを想定しています。`'post resampling'`の`'Field name'`を`'intensity'`に設定する必要があります。

| 名称               | タイプ                      | 説明                        |
| ------------------ | ------------------------- | ---------------------------------- |
| `input/pointcloud` | `sensor_msgs/PointCloud2` | ライダーセンサーからの点群データ |

### 出力

**はじめに**

このドキュメントは、Autowareの自動運転ソフトウェアにおけるplanningコンポーネン/モジュールのアーキテクチャの概要を提供します。

**コンポーネン/モジュール**

アーキテクチャは次のコンポーネン/モジュールで構成されています。

- **reference_path_generator (RPG):** 自車位置を考慮して、目標経路を生成します。
- **velocity_planner (VP):** 目標速度を生成します。
- **adaptive_lateral_controller (ALC):** 横方向の制御を実行します。
- **stationarity_checker (SC):** 静止した障害物を検出します。
- **trajectory_follower (TF):** 目標経路と速度に従って、車両を制御します。
- **mapping:** 環境に関する情報を提供します。
- **localization:** 自車位置を推定します。
- **perception:** 障害物や交通状況を検出します。

**アプローチ**

planningコンポーネン/モジュールは、次のアプローチを使用します。

- **モデル予測制御 (MPC):** 未来の制御入力を計算します。
- **確率的計画:** 不確実性を考慮します。
- `post resampling`手法:** 感知データの不確実性を軽減します。

**実装**

このアーキテクチャは、ROS (Robot Operating System) で実装されています。

**インターフェース**

planningコンポーネン/モジュールは、次のインターフェースを介して他のモジュールと通信します。

- **Topic:** ROSトピックを使用してデータの送受信を行います。
- **Service:** ROSサービスを使用して、機能の呼び出しを行います。

**利点**

このアーキテクチャには、次の利点があります。

- モジュール性が高い
- 再利用しやすい
- 拡張しやすい

**今後の展開**

今後の展開として、以下の機能が計画されています。

- 交通信号の制御
- 車両間通信
- センサーフュージョン

| 名称                          | タイプ                                                 | 説明                                                |
| ----------------------------- | ------------------------------------------------------ | --------------------------------------------------- |
| `output/labeled_clusters`     | `tier4_perception_msgs/DetectedObjectsWithFeature` | ラベル付きのポイントクラウドクラスタを持った検出オブジェクト |
| `debug/instance_pointcloud` | `sensor_msgs/PointCloud2`                          | 視覚化用のセグメント分割されたポイントクラウド            |

## パラメータ

### ノードパラメータ

なし

### コアパラメータ

| 名称                   | 種類   | デフォルト値       | 説明                                                                     |
| ---------------------- | ------ | -------------------- | --------------------------------------------------------------------------- |
| `score_threshold`      | double | 0.8                 | 検出オブジェクトのスコアがこの値よりも低い場合、オブジェクトは無視されます。 |
| `range`                | int    | 60                  | フィーチャマップの片側の長さの半分。 [m]                               |
| `width`                | int    | 640                 | フィーチャマップのグリッド幅。                                              |
| `height`               | int    | 640                 | フィーチャマップのグリッド高さ。                                             |
| `engine_file`          | string | "vls-128.engine"    | CNNモデルのTensorRTエンジンファイルの名前。                              |
| `prototxt_file`        | string | "vls-128.prototxt"  | CNNモデルのprototxtファイルの名前。                                      |
| `caffemodel_file`      | string | "vls-128.caffemodel" | CNNモデルのcaffemodelファイルの名前。                                    |
| `use_intensity_feature` | bool   | true                | 点群の距離特徴を使用するフラグ。                                         |
| `use_constant_feature` | bool   | false               | 点群の距離と距離の特徴を使用するフラグ。                                |
| `target_frame`         | string | "base_link"         | 点群データはこのフレームに変換されます。                                   |
| `z_offset`             | int    | 2                   | ターゲットフレームからのzオフセット。 [m]                               |
| `build_only`           | bool   | `false`             | TensorRTエンジンファイルのビルド後にノードをシャットダウンする。         |

## 想定 / 既知の制限

CNN モデル用のトレーニング コードはありません。

### 注意

このパッケージは、3 つの外部コードを使用しています。トレーニングされたファイルはアポロによって提供されており、ビルド時に自動的にダウンロードされます。

元の URL

- VLP-16:
  <https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel>
- HDL-64:
  <https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel>
- VLS-128:
  <https://github.com/ApolloAuto/apollo/raw/91844c80ee4bd0cc838b4de4c625852363c258b5/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel>

サポートされているライダーは Velodyne 16、64、128 です。ただし、Velodyne 32 やその他のライダーも高い精度で使用できます。

1. [アポロ 3D 障害物知覚の説明](https://github.com/ApolloAuto/apollo/blob/r7.0.0/docs/specs/3d_obstacle_perception.md)


   ```txt
   /******************************************************************************
   * Copyright 2017 The Apollo Authors. All Rights Reserved.
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   * http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   *****************************************************************************/
   ```

2. [tensorRTWrapper](https://github.com/lewes6369/tensorRTWrapper):
   libディレクトリで使用します。


   ```txt
   MIT License

   Copyright (c) 2018 lewes6369

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   ```

3. [autoware_perception 説明](https://github.com/k0suke-murakami/autoware_perception/tree/feature/integration_baidu_seg/lidar_apollo_cnn_seg_detect)


   ```txt
   /*
   * Copyright 2018-2019 Autoware Foundation. All rights reserved.
   *
   * Licensed under the Apache License, Version 2.0 (the "License");
   * you may not use this file except in compliance with the License.
   * You may obtain a copy of the License at
   *
   *     http://www.apache.org/licenses/LICENSE-2.0
   *
   * Unless required by applicable law or agreed to in writing, software
   * distributed under the License is distributed on an "AS IS" BASIS,
   * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   * See the License for the specific language governing permissions and
   * limitations under the License.
   */
   ```

### 特別御礼

- [Apollo Project](https://github.com/ApolloAuto/apollo)
- [lewes6369](https://github.com/lewes6369)
- [Autoware Foundation](https://github.com/autowarefoundation/autoware)
- [竹内浩介](https://github.com/kosuke55) (TIER IV)

