# yabLoc_particle_filter

このパッケージにはパーティクルフィルタ関連の実行可能ノードがいくつか含まれています。

- [particle_predictor](#particle_predictor)
- [gnss_particle_corrector](#gnss_particle_corrector)
- [camera_particle_corrector](#camera_particle_corrector)

## particle_predictor

### 目的

- このノードはパーティクルの予測更新と再サンプリングを実行します。
- コレクタノードによって決定された粒子の重みを遡って反映します。

### 入力 / 出力

#### 入力

| 名前 | タイプ | 説明 |
|---|---|---|
| `input/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | パーティクルの初期位置を指定する |
| `input/twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | 予測更新の線速度と角速度 |
| `input/height` | `std_msgs::msg::Float32` | 地上高 |
| `input/weighted_particles` | `yabloc_particle_filter::msg::ParticleArray` | 補正ノードで重み付けされたパーティクル |

#### 出力

Autowareで利用可能な安全パラメータのリストは次のとおりです。

* **車間距離パラメータ**
    * `min_following_distance` (`[m]`)：Autowareが維持しようとする前方の車両との最小距離
    * `min_following_distance_max_speed` (`[m]`)：`min_following_distance`パラメータを最大速度として適用する速度しきい値
    * `time_headway` (`[sec]`)：周囲車両に対する適切な車間距離を計算するためにAutowareが使用する時間間隔
* **横方向安全距離パラメータ**
    * `min_lateral_offset` (`[m]`)：障害物に対してAutowareが維持しようとする最小横方向オフセット
    * `min_lateral_offset_max_speed` (`[m]`)：`min_lateral_offset`パラメータを最大速度として適用する速度しきい値
* **速度パラメーター**
    * `max_speed` (`[km/h]`)：Autowareトラジェク\トリージェネレーターが生成するトラック可能な最大速度
    * `max_accel` (`[m/s^2]`)：車両に入力できる最大加速度
    * `max_decel` (`[m/s^2]`)：車両に入力できる最大減速度
    * `search_speed` (`[km/h]`)：経路計画を実行するときにAutowareが使用する検索速度（経路計画参照）
* **経路計画パラメーター**
    * `max_path_points`：経路計画モジュールが1つのパスに収容できる最大経路ポイント数
    * `smoothing_factor`：経路計画モジュールが生成するパスを滑らかにするために使用する平滑化係数
* **障害物検出パラメータ**
    * `min_detection_height` (`[m]`)：障害物検出が有効になる最小高さ
* **Planningパラメータ**
    * `lanechange_frequency` (`[m]`)：Autowareが車線変更を実行する頻度
    * `min_lanechange_speed` (`[km/h]`)：Autowareが車線変更を実行する最小速度
    * `lane_change_check_distance` (`[m]`)：Autowareが前方の車線変更の可能性をチェックし始める距離
    * `safe_stop_distance` (`[m]`)：Autowareが進行中の軌跡を停止するために必要な最小の安全距離
* **制御パラメータ**
    * `brake_decel_rate` (`[m/s^2]`)：自動運転システムが自律ブレーキを実行するために使用する減速度率
* **その他**
    * `debug_mode`：デバッグモードを有効にするかどうかを制御します (`[bool]`)
    * `post resampling`：経路計画後処理を有効にするかどうかを制御します (`[bool]`)

| 名前 | 種類 | 説明 |
|---|---|---|
| output/pose_with_covariance | geometry_msgs::msg::PoseWithCovarianceStamped | 粒子のセントロイドと共分散 |
| output/pose | geometry_msgs::msg::PoseStamped | 粒子のセントロイドと共分散 |
| output/predicted_particles | yabloc_particle_filter::msg::ParticleArray | 予測ノードによって重み付けられた粒子 |
| debug/init_marker | visualization_msgs::msg::Marker | 初期位置のデバッグ可視化 |
| debug/particles_marker_array | visualization_msgs::msg::MarkerArray | 粒子の可視化（`visualize`がTrueの場合に公開） |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/predictor.schema.json") }}

### サービス

| 名称                 | タイプ                     | 説明                                           |
| -------------------- | ------------------------ | ----------------------------------------------- |
| `yabloc_trigger_srv` | `std_srvs::srv::SetBool` | YABLOC推定の有効化と無効化                     |

## gnss_particle_corrector

### 目的

- このノードはGNSSを使用してパーティクルの重みを推定します。
- 以下の2つの種類の入力をサポートしています: `ublox_msgs::msg::NavPVT`と`geometry_msgs::msg::PoseWithCovarianceStamped`

### 入出力

#### 入力

| 名前                         | 型                                            | 説明                                        |
| ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| `input/height`               | `std_msgs::msg::Float32`                        | 地面からの高さ                                      |
| `input/predicted_particles`  | `yabloc_particle_filter::msg::ParticleArray`    | 予測粒                                        |
| `input/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | GNSS 測定 `use_ublox_msg` が false の場合使用 |
| `input/navpvt`               | `ublox_msgs::msg::NavPVT`                       | GNSS 測定 `use_ublox_msg` が true の場合使用  |

#### 出力

**自動運転ソフトウェアのドキュメント**

**要約**

このドキュメントでは、Autoware自動運転ソフトウェアのアーキテクチャ、コンポーネント、およびインターフェースについて説明します。このソフトウェアは、車両に搭載されたセンサーからのデータを処理し、環境を認識し、走行の意思決定と制御を行います。

**アーキテクチャ**

Autowareのアーキテクチャはモジュール方式で設計されており、各モジュールは特定の機能を担当しています。これにより、ソフトウェアの保守性と拡張性が向上します。主なモジュールを以下に示します。

* **センサーモジュール:** センサーからのデータを処理し、生のデータを認識可能な形式に変換します。
* **認識モジュール:** センサーデータを使用して、周囲の環境を認識します。これには、物体の検出、分類、トラッキングなどがあります。
* **予測モジュール:** 認識された物体の動きを予測し、将来の衝突の可能性を特定します。
* **Planningモジュール:** 予測に基づいて、車両の走行経路と速度を計画します。
* **制御モジュール:** Planningモジュールから出された経路と速度に従って、車両の運動を制御します。

**インターフェース**

各モジュールは、他のモジュールとデータ交換するためのインターフェースを定義しています。インターフェースは、ROS（Robot Operating System）トピックまたはサービスコールに基づいています。これにより、モジュール間の疎結合が実現し、ソフトウェアのモジュール化と再利用性が向上します。

**センサー**

Autowareは、さまざまなセンサータイプをサポートしています。これらには、以下のものが含まれます。

* LiDAR
* カメラ
* レーダー
* IMU

**認識**

認識モジュールは、センサーデータを使用して環境を認識します。主な認識アルゴリズムを以下に示します。

* 物体検出: YOLO、Faster R-CNN
* 物体分類: ResNet、MobileNet
* 物体追跡: Kalmanフィルタ、Particle Filter

**予測**

予測モジュールは、認識された物体の動きを予測します。主な予測アルゴリズムを以下に示します。

* カルマンフィルタ
* 粒子フィルタ
* アディティブノイズモデル

**Planning**

Planningモジュールは、予測に基づいて車両の走行経路と速度を計画します。主なPlanningアルゴリズムを以下に示します。

* ダイナミックプログラミング
* RRT（Rapidly-exploring Random Tree）
* LQR（Linear-quadratic Regulator）

**制御**

制御モジュールは、Planningモジュールから出された経路と速度に従って、車両の運動を制御します。主な制御アルゴリズムを以下に示します。

* PID制御
* 状態フィードバック制御
* モデル予測制御

**追加機能**

Autowareは、以下の追加機能も提供しています。

* レイヤー統合: センサーデータを異なるモダリティから融合して、より堅牢な認識を提供します。
* ロкалиゼーション:車両の自車位置と姿勢を推定します。
* 'post resampling'による高精度なセンサー校正: センサー間のオフセット補正を向上させます。
* 安全性監視: 車両挙動を監視して、安全性を確保します。
* HMI (Human-Machine Interface): ドライバーとの相互作用を提供します。

| 名前                           | 型                                           | 説明                                                    |
| ------------------------------ | -------------------------------------------- | --------------------------------------------------------- |
| `output/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray` | 重み付き粒子                                            |
| `debug/gnss_range_marker`      | `visualization_msgs::msg::MarkerArray`       | GNSS の重み分布                                          |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray`       | 粒子ビジュアライゼーション. `visualize` が true の場合に発行 |

### パラメーター

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/gnss_particle_corrector.schema.json") }}


## カメラパーティクルコレクタ

### 目的
- このノードはGNSSを使用してパーティクルの重みを求めます。

### 入出力

#### 入力

| 名称                                 | タイプ                                       | 説明                                                   |
| ----------------------------------- | ------------------------------------------ | -------------------------------------------------------- |
| `input/predicted_particles`         | `yabloc_particle_filter::msg::ParticleArray` | 予測されたパーティクル                                   |
| `input/ll2_bounding_box`            | `sensor_msgs::msg::PointCloud2`            | 路面マーキングをラインセグメントに変換したもの             |
| `input/ll2_road_marking`            | `sensor_msgs::msg::PointCloud2`            | 路面マーキングをラインセグメントに変換したもの             |
| `input/projected_line_segments_cloud` | `sensor_msgs::msg::PointCloud2`            | 投影されたラインセグメント                                |
| `input/pose`                        | `geometry_msgs::msg::PoseStamped`          | self位置周辺のエリアマップを取得するための参照             |

#### 出力

**自動運転ソフトウェアの用語と定義**

| 用語 | 定義 |
|---|---|
| Planning | 走行経路や行動を決定するモジュール |
| Localization | 自車位置を推定するモジュール |
| Perception | 周囲環境を認識するモジュール |
| Control | 車両を制御するモジュール |
| Sensor | 周囲環境を測定する装置 |
| Actuator | 車両を制御するために使用される装置 |
| Localization | 自車位置を推定するモジュール |
| Autoware | オープンソースの自動運転ソフトウェアプラットフォーム |
| `post resampling` | 走行経路最適化の再サンプルフェーズ |
| `open space` | 走行経路内の障害物がない領域 |

| 名称                           | タイプ                                           | 説明                                               |
| ------------------------------ | ------------------------------------------------ | --------------------------------------------------------- |
| `output/weighted_particles`    | `yabloc_particle_filter::msg::ParticleArray` | 重み付け粒子                                         |
| `debug/cost_map_image`         | `sensor_msgs::msg::Image`                      | lanelet2から作成したコストマップ                      |
| `debug/cost_map_range`         | `visualization_msgs::msg::MarkerArray`         | コストマップ境界                                       |
| `debug/match_image`            | `sensor_msgs::msg::Image`                      | 投影された線分画像                                   |
| `debug/scored_cloud`           | `sensor_msgs::msg::PointCloud2`                  | 重み付け3D線分                                        |
| `debug/scored_post_cloud`      | `sensor_msgs::msg::PointCloud2`                  | 'post resampling'の非確実性のある重み付け3D線分       |
| `debug/state_string`           | `std_msgs::msg::String`                        | ノードの状態を表す文字列                                |
| `debug/particles_marker_array` | `visualization_msgs::msg::MarkerArray`         | パーティクル可視化。`visualize`がTrueの場合のみ公開される |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_particle_filter/schema/camera_particle_corrector.schema.json") }}

### サービス

| 名前 | 型 | 説明 |
|---|---|---|
| `switch_srv` | `std_srvs::srv::SetBool` | 補正の有効化および無効化 |

