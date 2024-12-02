# yabloc_image_processing

このパッケージは画像処理に関するいくつかの実行可能ノードを含みます。

- [line_segment_detector](#line_segment_detector)
- [graph_segmentation](#graph_segmentation)
- [segment_filter](#segment_filter)
- [undistort](#undistort)
- [lanelet2_overlay](#lanelet2_overlay)
- [line_segments_overlay](#line_segments_overlay)

## line_segment_detector

### 目的

このノードはグレースケール画像から全ての線分を抽出します。

### 入出力

#### 入力
```
/input/image: sensor_msgs/image
```

#### 出力
```
/output/segments: autoware_msgs/line_segments
```

---

#### パラメータ
```
flip_vertical: bool  (default: false)
```

### ロジック

このノードは、以下を実行します。

1. グレースケール画像を受信します。
2. 画像の水平反転を実行します（`flip_vertical=true` の場合）。
3. Cannyエッジ検出を実行します。
4. Hough変換を実行して線分を検出します。
5. 検出した線分を公開します。

| 名前                   | タイプ                      | 説明                        |
| -------------------- | ------------------------- | -------------------------- |
| `input/image_raw`     | `sensor_msgs::msg::Image` | 歪みのない画像                |

#### 出力

このドキュメントは、AutowareにおけるPlanningコンポーネントの技術的詳細を説明しています。

Planningコンポーネントは、現在の状態とランタイム環境を考慮して、車両の経路を決定します。以下にその機能を説明します。

* **経路生成:** 現在の自車位置と目的地を基に、安全かつ効率的な経路を生成します。
* **軌道生成:** 生成された経路上の目標軌道を作成します。
* **軌跡生成:** 目標軌道に従った軌跡を生成します。
* **衝突回避:** 予想される物体との衝突を回避するための緊急経路を生成します。

**経路生成アルゴリズム**

Planningコンポーネントは、以下を含むさまざまな経路生成アルゴリズムをサポートしています。

* **波面伝搬:** 2Dまたは3Dの空間で波面を伝搬させて経路を生成します。
* **動的計画法:** 問題を小さなサブ問題に分割して、最適な経路を計算します。
* **グラフ探索:** グラフ構造を使用して経路を探索します。

**軌道生成アルゴリズム**

Planningコンポーネントは、以下を含むさまざまな軌道生成アルゴリズムをサポートしています。

* **ステアリング制御器:** 理想的な軌道に従うようにステアリング角を計算します。
* **速度制御器:** 理想的な速度に従うように加速とブレーキを制御します。
* **ラテラルおよび縦断制御器:** ステアリングと速度を協調的に制御して、滑らかな軌跡を生成します。

**軌跡生成手法**

Planningコンポーネントは、軌跡を生成するために、'post resampling'手法を使用します。この手法は、軌道上のポイントを再サンプルして、軌跡がより滑らかになるようにします。

**衝突回避**

Planningコンポーネントは、以下を含むさまざまな衝突回避メカニズムをサポートしています。

* **予測:** センサデータを使用して、周囲の物体の動きを予測します。
* **軌跡適応:** 障害物との衝突のリスクを軽減するために、車両の軌跡を適応させます。
* **緊急経路生成:** 回避不可能な衝突に対して迅速に対応する緊急経路を生成します。

| 名前                              | 種類                            | 説明                                                                                                                                                               |
| --------------------------------- | ------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `output/image_with_line_segments` | `sensor_msgs::msg::Image`       | 線分をハイライトした画像                                                                                                                                 |
| `output/line_segments_cloud`      | `sensor_msgs::msg::PointCloud2` | 点群として検出された線分。各点に x、y、z、normal_x、normal_y、normal_z が含まれるが、normal_z は常に空になる。 |

## graph_segmentation

### 목적

이 노드는 [graph-based-segmentation](https://docs.opencv.org/4.5.4/dd/d19/classcv_1_1ximgproc_1_1segmentation_1_1GraphSegmentation.html)를 사용하여 도로 표면 영역을 추출합니다.

### 입력/출력

#### 입력

| 名称              | タイプ                      | 説明       |
| ----------------- | ------------------------- | ----------------- |
| `input/image_raw` | `sensor_msgs::msg::Image` | 非歪曲化画像 |

#### アウトプット

- 制御構造
  - ドメイン
  - レベル
  - ドメインコントローラ
- ソフトウェアアーキテクチャ
  - Autoware Open Architecture
  - Perception/Planning
    - Planningモジュール
    - Plan Evaluation
    - Path Tracking
    - post resampling
    - Behavior Tree
    - Route Planning
  - Localization
    - GPS/IMU
    - Mapping
    - Localization
  - Vehicle Control
    - MPC
    - Vehicle Dynamic Model
    - Brake Controller
    - Steer Controller
  - System Management
    - Rosbags
    - Webview
    - Visualization
    - Rosbridge
  - Toolchain
    - Autoware.AI Platform
- ご利用情報
  - ライセンス
  - コミュニティ
  - チュートリアル
  - Webフォーム
- 開発の仕組み
- これまでの実績
- ホワイトペーパー
- 参照情報
  - 論文
  - コード
  - Webサイト

| 名前                                 | タイプ                        | 説明                                                         |
|--------------------------------------|----------------------------|---------------------------------------------------------------|
| `output/mask_image`                  | `sensor_msgs::msg::Image` | 路面の領域として決定されたセグメントがマスクされた画像        |
| `output/segmented_image`             | `sensor_msgs::msg::Image` | 可視化用のセグメント化された画像                              |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_image_processing/schema/graph_segment.schema.json") }}

## segment_filter

### 目的

このノードは、`graph_segment`と`lsd`の結果を統合して、路面標示を抽出します。

### 入出力

#### 入力

| 名前                          | 型                            | 説明                                                  |
| --------------------------- | ------------------------------- | -------------------------------------------------------- |
| `input/line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | 検出された線分                                       |
| `input/mask_image`          | `sensor_msgs::msg::Image`       | 道路の路面として判断されたセグメントがマスクされた画像 |
| `input/camera_info`         | `sensor_msgs::msg::CameraInfo`  | 歪みのないカメラ情報                                 |

#### 出力

Autowareにおける自動運転ソフトウェアの開発プロセスを、以下に示します。

1. **センサーデータの収集**
   - 車載センサーからセンサーデータを収集する。
2. **センサーデータの処理**
   - センサーデータからオブジェクト、レーン、および他の重要な情報を検出する。
3. **地図データの統合**
   - 地図データとセンサーデータを統合し、環境をより正確にモデル化する。
4. **自車位置の推定**
   - 自車位置と姿勢を推定する。
5. **経路計画**
   - 現在地から目的地までの安全で効率的な経路を計画する。
6. **動作計画**
   - 経路計画に基づいて、車両を制御するための動作計画を作成する。
7. **車両制御**
   - 動作計画を実行し、車両を制御する。

このプロセスは反復的に行われます。センサーデータの処理と自車位置の推定からのフィードバックに基づいて、経路計画と動作計画が調整されます。これにより、変化する環境に適応できる、より堅牢で安全な自動運転システムが実現します。

| 名前                                 | タイプ                              | 説明                                                     |
| ------------------------------------- | --------------------------------- | -------------------------------------------------------- |
| `output/line_segments_cloud`           | `sensor_msgs::msg::PointCloud2`   | 視覚化用のフィルタされた線分                                |
| `output/projected_image`               | `sensor_msgs::msg::Image`         | 視覚化用の投影されたフィルタリング済み線分                    |
| `output/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`   | 投影されたフィルタリング済み線分                              |

### パラメーター

{{ json_to_markdown("localization/yabloc/yabloc_image_processing/schema/segment_filter.schema.json") }}

## undistort

### 目的

このノードは、画像のサイズ変更と歪み補正を同時に実行します。

### 入出力

#### 入力

| 名称                         | タイプ                                | 説明             |
| ---------------------------- | ----------------------------------- | ----------------------- |
| `input/camera_info`          | `sensor_msgs::msg::CameraInfo`      | カメラ情報             |
| `input/image_raw`            | `sensor_msgs::msg::Image`           | 生のカメラ画像        |
| `input/image_raw/compressed` | `sensor_msgs::msg::CompressedImage` | 圧縮されたカメラ画像 |

このノードは圧縮画像と生画像の両方のトピックを購読します。
生画像が一度でも購読されると、圧縮画像は購読されなくなります。
これは Autoware 内での不要な解凍を避けるためです。

#### 出力

| 名称 | 型 | 説明 |
|---|---|---|
| `output/camera_info` | `sensor_msgs::msg::CameraInfo` | リサイズされたカメラ情報 |
| `output/image_raw` | `sensor_msgs::msg::CompressedImage` | レンズ補正とサイズ変更済みの画像 |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_image_processing/schema/undistort.schema.json") }}

#### tf_staticオーバーライドについて

<details><summary>クリックして開く</summary><div>

一部のノードでは `/sensing/camera/traffic_light/image_raw/compressed` の frame_id （例 `/traffic_light_left_camera/camera_optical_link`）に対して `/base_link` から `/tf_static` が必要です。tf_staticが正しいかどうかは以下のコマンドで確認できます。


```shell
ros2 run tf2_ros tf2_echo base_link traffic_light_left_camera/camera_optical_link
```

プロトタイプの車両の使用、正確なキャリブレーションデータの不足、またはその他の避けられない理由により、間違った `/tf_static` がブロードキャストされた場合、`override_camera_frame_id` で frame_id を指定すると便利です。
空でない文字列を指定すると、`/image_processing/undistort_node` は camera_info の frame_id を書き換えます。
たとえば、次のように異なる tf_static を指定できます。


```shell
ros2 launch yabloc_launch sample_launch.xml override_camera_frame_id:=fake_camera_optical_link
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link \
  --child-frame-id fake_camera_optical_link \
  --roll -1.57 \
  --yaw -1.570
```

</div></details>

## lanelet2_overlay

### 目的

このノードは、推定された自車位置に基づいて、カメラ画像にlanelet2を重ね合わせます。

### 入出力

#### 入力

| 名称                                   | 型                                 | 説明                                              |
| -------------------------------------- | ----------------------------------- | --------------------------------------------------- |
| `input/pose`                           | `geometry_msgs::msg::PoseStamped` | 推定自車位置                                  |
| `input/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`   | 非道路標示を含む投影線分群                    |
| `input/camera_info`                    | `sensor_msgs::msg::CameraInfo`    | 歪みのないカメラ情報                              |
| `input/image_raw`                      | `sensor_msgs::msg::Image`         | 歪みのないカメラ画像                              |
| `input/ground`                         | `std_msgs::msg::Float32MultiArray` | 地面傾斜                                          |
| `input/ll2_road_marking`               | `sensor_msgs::msg::PointCloud2`   | 路面標示に関するLanelet2要素                    |
| `input/ll2_sign_board`                 | `sensor_msgs::msg::PointCloud2`   | 交通標識に関するLanelet2要素                    |

## 自動運転ソフトウエアのドキュメント

### Overview
Autowareによる自動運転ソフトウエアは、自律走行車両の様々な機能やタスクに対応するコンポーネントから構成されています。
このドキュメントでは、これらのコンポーネントとその相互作用について説明します。

### Planning
Planningコンポーネントは、車両の目標経路を生成し、リアルタイムで更新します。高精度地図データとセンサからの情報を利用して、障害物を回避し、交通ルールに従う安全な経路を計画します。

### Control
Controlコンポーネントは、車両の運動を制御するためにPlanningコンポーネントから受け取った目標経路に従います。目標の経路を追従するようステアリング、アクセル、ブレーキを調整します。

### Perception
Perceptionコンポーネントは、センサからのデータを処理し、車両の周囲環境を認識します。物体の検出、分類、追跡を行い、周囲の状況に関する情報を提供します。

### Localization
Localizationコンポーネントは、車両の自車位置と向きを決定します。GNSS、IMU、オドメータなどのセンサを利用して、正確な位置情報を提供します。

### Post-processing
'post resampling'コンポーネントは、センサからのデータのタイムスタンプを調整し、各コンポーネントに同時刻のデータを供給します。これにより、すべてのコンポーネントが整合したデータに基づいて動作できるようになります。

### Communication
Communicationコンポーネントは、車両と外部システムの間の通信を処理します。他の車両との通信、交通インフラとの接続、リモート監視への対応を行います。

### その他のコンポーネント
これらの中核コンポーネントに加えて、次のような追加コンポーネントも含まれます。

- モニタリング：車両のシステムを監視し、異常検出と警告を行います。
- HMI：人間と機械のインターフェースを提供し、ドライバーに対する情報やフィードバックを提供します。
- シミュレーション：自動運転システムのテストや検証のためのシミュレーション環境を提供します。

| 項目 | タイプ | 説明 |
|---|---|---|
| `output/lanelet2_overlay_image` | `sensor_msgs::msg::Image` | lanelet2を重ねた画像 |
| `output/projected_marker` | `visualization_msgs::msg::Marker` | 車線以外の標識を含む3D投影の線分 |

## line_segments_overlay

### 目的

このノードは、カメラ画像上に分類された直線を視覚化する。

### 入出力

#### 入力

| 名称                        | タイプ                            | 説明              |
| --------------------------- | ------------------------------- | ------------------------ |
| `input/line_segments_cloud` | `sensor_msgs::msg::PointCloud2` | 分類された線分 |
| `input/image_raw`           | `sensor_msgs::msg::Image`       | 歪みのないカメラ画像 |

#### 出力

- 開発ガイド [development-guide](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/development-guide.md)
- センサー: Velodyne HDL-64E [HDL-64E_on_rooftop_right](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/sensor/HDL-64E_on_rooftop_right.md)
- ロボットオペレーティングシステム (ROS) ワークスペースの構成 [ROS_Workspace_Configuration](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/installation/ROS_Workspace_Configuration.md)
- 3Dオブジェクト検出 [3D_Object_Detection](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/perception/3D_Object_Detection.md)
- カットオフアンテナ [cut-off-antenna](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/sensor/cut-off-antenna.md)
- パスプランニング [path_planning](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/planning/path_planning.md)
- 制御 [control](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/control/control.md)
- モデルの予測アルゴリズム [model_predictive_algorithm](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/control/model_predictive_algorithm.md)
- タイヤの「スリップ角度」 [slip_angle](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/control/slip_angle.md)
- クラウドの追跡 [cloud_tracking](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/perception/cloud_tracking.md)
- シャーシ [chassis](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/interface/chassis.md)
- Planning Routing Manager [planning_routing_manager](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/planning/planning_routing_manager.md)
- `post resampling`処理 [post_resampling_process](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/perception/post_resampling_process.md)
- 自車位置 [current_pose](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/docs/perception/current_pose.md)

| 名前                                          | タイプ                      | 説明                          |
| ------------------------------------------- | ------------------------- | ------------------------------------ |
| `output/image_with_colored_line_segments` | `sensor_msgs::msg::Image` | 強調された線分でハイライトされた画像 |

