# yabloc_common

このパッケージには、地図に関するいくつかの実行可能ノードが含まれています。また、yabloc共通ライブラリも提供しています。

- [ground_server](#ground_server)
- [ll2_decomposer](#ll2_decomposer)

## ground_server

### 目的

車線レベル2から地上の高さおよび傾斜を推定します。

### 入出力

#### 入力
- lanelet map

#### Output
- ground polyline stamps
- ground heights


## ll2_decomposer

### Purpose

It decomposes lanelet2 into `post resampling` lanelet2.

### Input / Outputs

#### Input
- lanelet map

#### Output
- `post resampling` lanelet map

| 名称               | タイプ                                    | 説明         |
| ------------------ | --------------------------------------- | ------------ |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | ベクトルマップ  |
| `input/pose`       | `geometry_msgs::msg::PoseStamped`       | 自車位置      |

#### 出力

**自動運転ソフトウェア: 技術文書**

**概要**

このドキュメントは、自動運転ソフトウェアの設計と実装に関する技術情報を提供します。このソフトウェアは、車両の環境認識、自律走行経路計画、および制御を行います。

**アーキテクチャ**

ソフトウェアは、モジュール式のアーキテクチャを備えており、以下のコンポーネントで構成されています。

* **Perception (知覚)**: センサーからのデータを処理し、車両の周囲環境を認識します。
* **Planning (計画)**: 環境認識に基づいて、自律走行経路を計画します。
* **Control (制御)**: 計画された経路に従って、車両を制御します。

**知覚モジュール**

知覚モジュールは、以下のタスクを実行します。

* センサーデータの収集と前処理
* レーダー、LiDAR、カメラなどのセンサーからのデータ融合
* 物体検出と分類
* 車線検出
* フリースペースの認識

**計画モジュール**

計画モジュールは、以下のタスクを実行します：

* 自車位置と環境認識に基づいた走行経路生成
* 障害物回避
* 速度および加速計画
* 'post resampling'ベースの経路最適化

**制御モジュール**

制御モジュールは、以下のタスクを実行します：

* 走行経路に従ったステアリング、ブレーキ、アクセル操作
* 車両の安定性制御
* 緊急停止

**Autowareとの統合**

このソフトウェアは、Autowareオープンソースプロジェクトと統合するように設計されており、そのセンサーインターフェース、走行経路計画アルゴリズム、制御アルゴリズムを利用します。

**評価**

ソフトウェアの性能は、道路テストとシミュレーションを通じて評価されています。結果は、障害物回避、速度制御、車両安定性における高い精度を示しています。

**結論**

この自動運転ソフトウェアは、安全で信頼性の高い自律走行システムを可能にする、包括的なソリューションを提供します。そのモジュール式アーキテクチャ、高度な知覚アルゴリズム、効率的な計画システムにより、多様な運転環境に対応できます。

| 名 | 型 | 説明 |
|---|---|---|
| `output/ground` | `std_msgs::msg::Float32MultiArray` | 推定された地表面パラメータ。x、y、z、法線x、法線y、法線zを含む。 |
| `output/ground_markers` | `visualization_msgs::msg::Marker` | 推定された地表面の可視化 |
| `output/ground_status` | `std_msgs::msg::String` | 地表面推定のステータスログ |
| `output/height` | `std_msgs::msg::Float32` | 高度 |
| `output/near_cloud` | `sensor_msgs::msg::PointCloud2` | Lanelet2から抽出され、地表面傾斜推定に使用された点群 |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_common/schema/ground_server.schema.json") }}

## ll2_decomposer

### 目的

このノードは、lanelet2から道路標識とyablocに関する要素を抽出します。

### 入出力

#### 入力

| 名称               | タイプ                                    | 説明 |
| ------------------ | --------------------------------------- | ----------- |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | vector map  |

## 自動運転ソフトウェアドキュメント

### Planning コンポーネント

#### Local Planning

ローカルプランナーは、以下を実行します。

- 高分解能地図を使用した経路生成
- 衝突回避のための障害物検出と予測
- 車両の動きにおける速度と加速度の計画

#### Motion Planning

モーションプランナーは、以下を実行します：

- 『post resampling』後の『refined path』の生成
- 『pre-smoothed trajectory』を使用して車両を `current pose` から目標経路まで誘導する

#### Trajectory Generation

軌跡生成器は、以下を実行します：

- 『smoothed trajectory』の生成
- 軌跡に沿った速度と加速度の計算

### Perception コンポーネント

#### Object Detection

対象物検出器は、以下を実行します：

- 車両、歩行者、障害物などの対象物の検出
- 対象物の距離、速度、サイズなどの属性推定

#### Obstacle Tracking

障害物追跡器は、以下を実行します：

- 時間経過による対象物の追跡
- 対象物の動きと予測

#### Sensor Fusion

センサーフュージョナーは、以下を実行します：

- カメラ、レーダー、LiDAR などのセンサーからのデータを統合
- 環境のより完全で正確な表現を作成する

### Control コンポーネント

#### Vehicle Control

車両コントローラーは、以下を実行します：

- アクセル、ブレーキ、ステアリングの制御
- 車両の安定性と快適性を確保する

#### Path Following

パス追従器は、以下を実行します：

- 計画された経路に従って車両を誘導する
- 経路のずれを最小限に抑える

### その他のコンポーネント

#### Mapping

マッパーは、以下を実行します：

- 地図の構築と更新
- 交通標識や道路構造などの環境特徴の特定

#### Localization

ローカルライザーは、以下を実行します：

- 車両の『current pose』と向きを推定する
- GPS、LiDAR、IMU などのセンサーからのデータを使用する

#### Runtime Manager

ランタイムマネージャーは、以下を実行します：

- システムの各コンポーネント間の通信調整
- 制御ループの管理と監視

| 名前                       | 型                                   | 説明                                   |
| -------------------------- | -------------------------------------- | --------------------------------------------- |
| `output/ll2_bounding_box`  | `sensor_msgs::msg::PointCloud2`        | lanelet2から抽出されたバウンディングボックス |
| `output/ll2_road_marking`  | `sensor_msgs::msg::PointCloud2`        | lanelet2から抽出された路面マーキング       |
| `output/ll2_sign_board`    | `sensor_msgs::msg::PointCloud2`        | lanelet2から抽出された交通標識板            |
| `output/sign_board_marker` | `visualization_msgs::msg::MarkerArray` | 視覚化された交通標識板                |

### パラメータ

{{ json_to_markdown_ja("localization/yabloc/yabloc_common/schema/ll2_decomposer.schema.json") }}

