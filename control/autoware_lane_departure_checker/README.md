## 車線逸脱チェッカー

**車線逸脱チェッカー** は、車両が経路に従っているかどうかを確認します。経路に従っていない場合は、`diagnostic_updater` でステータスを報告します。

## 特徴

このパッケージには、以下の機能が含まれます。

- **車線逸脱:** 制御モジュールからの出力（予測経路）に基づいて、自車位置が車線境界から逸脱する可能性があるかどうかを確認します。
- **経路逸脱:** 自車位置が経路から逸脱していないかどうかを確認します。横方向、縦方向、ヨー方向の逸脱を確認します。
- **路側逸脱:** 制御の出力から生成された自車の足跡が、路側境界を超えるかどうかを確認します。

## 内部動作 / アルゴリズム

### 共分散による足跡の拡張方法

1. 車両座標系における誤差楕円（共分散）の標準偏差を計算します。

   1. 共分散を車両座標系に変換します。

   $$
   \begin{align}
   \left( \begin{array}{cc} x_{vehicle}\\ y_{vehicle}\\ \end{array} \right) = R_{map2vehicle}  \left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
   \end{align}
   $$

   車両座標系における共分散を計算します。

   $$
   \begin{align}
   Cov_{vehicle} &= E \left[
   \left( \begin{array}{cc} x_{vehicle}\\ y_{vehicle}\\ \end{array} \right) (x_{vehicle}, y_{vehicle}) \right] \\
   &= E \left[ R\left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
   (x_{map}, y_{map})R^t
   \right] \\
   &= R E\left[ \left( \begin{array}{cc} x_{map}\\ y_{map}\\ \end{array} \right)
   (x_{map}, y_{map})
   \right] R^t \\
   &= R Cov_{map} R^t
   \end{align}
   $$

   2. 拡張する縦方向の長さは、`Cov_{vehicle}(0,0)` で表される $x_{vehicle}$ の周辺分布に対応します。同様に、横方向の長さは `Cov_{vehicle}(1,1)` で表されます。Wikipedia の参考 [こちら](https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Marginal_distributions)。

2. 標準偏差に `footprint_margin_scale` を乗算した値に基づいて足跡を拡張します。

## インターフェイス

### 入力

- /localization/kinematic_state [`nav_msgs::msg::Odometry`]
- /map/vector_map [`autoware_map_msgs::msg::LaneletMapBin`]
- /planning/mission_planning/route [`autoware_planning_msgs::msg::LaneletRoute`]
- /planning/scenario_planning/trajectory [`autoware_planning_msgs::msg::Trajectory`]
- /control/trajectory_follower/predicted_trajectory [`autoware_planning_msgs::msg::Trajectory`]

### 出力

- [`diagnostic_updater`] lane_departure : 自車位置が車線から逸脱したときに診断レベルを更新します。

## パラメータ

### ノードパラメータ

#### 一般パラメータ

| 名前                        | タイプ | 説明                                                                                                | デフォルト値 |
| :-------------------------- | :----- | :---------------------------------------------------------------------------------------------------- | :------------ |
| will_out_of_lane_checker   | ブール | 自車フットプリントがレーンから逸脱するかどうかを確認するチェッカーを有効化                           | True          |
| out_of_lane_checker        | ブール | 自車フットプリントがレーンから逸脱しているかどうかを確認するチェッカーを有効化                     | True          |
| boundary_departure_checker | ブール | boundary_types_to_detectで指定された境界から自車フットプリントが逸脱するかどうかを確認するチェッカーを有効化 | False         |
| update_rate                | double | 公開する周波数 [Hz]                                                                                   | 10.0          |
| visualize_lanelet          | ブール | レーンレットを視覚化するためのフラグ                                                                    | False         |

#### 車線逸脱の各種パラメータ

#### Lane Departure Adaptive Integral Controller 

このパラメータは、車線逸脱整合積分コントローラのゲインと積分時間定数を指定します。

```markdown
lbc_steer_ki: 0.02
lbc_steer_kp: 0.02
lbc_steer_tau: 0.2
```

#### Planning

これらのパラメータは、レーンキープアシストのPlanning(Plan View)モジュールにおける車線逸脱検出とアシストに関連します。

```markdown
lka_look_ahead_distance: 20.0
lka_safety_distance: 3.0
lka_deviation_distance_thres: 1.0
```

#### LIDAR

これらのパラメータは、LIDARセンサーからのデータ処理に関するものです。

```markdown
max_lidar_collisions_per_point: 10
max_lidar_collisions_per_cluster: 20
max_lidar_collisions_dist: 0.5
```

#### Cone Detection

これらのパラメータは、コーン検出の精度と速度に関連します。

```markdown
cone_height_threshold: 0.1
cone_radius_threshold: 0.3
cone_angle_tolerance: 10.0
cone_detection_range: 30
cone_detect_point_num_threshold: 10
cone_detect_point_rate_threshold: 0.5
```

#### Lane Detection

これらのパラメータは、レーザーセンサーからのデータに基づいてレーンの検出と追跡の精度と速度に関連します。

```markdown
num_lane_candidate_lines: 10
num_lane_candidates_for_lane_detection: 8
max_num_detected_lane_lines: 3
lane_candidate_min_detection_dist: 3.0
lane_candidate_max_detection_angle: 2.0
lane_detect_range: 20.0
lane_length_threshold: 3.0
lane_mark_length_threshold_internal: 0.8
lane_mark_length_threshold_external: 2.0
lane_mark_length_threshold_external_wide: 3.0
lateral_offset_thres_internal: 0.2
```

| 名称                      | 型 | 説明                                       | デフォルト値 |
| :------------------------ | :--- | :------------------------------------------------ | :------------ |
| include_right_lanes       | bool | 境界に右側のレーンレットを含めるフラグ       | False         |
| include_left_lanes        | bool | 境界に左側のレーンレットを含めるフラグ        | False         |
| include_opposite_lanes    | bool | 境界に反対側のレーンレットを含めるフラグ    | False         |
| include_conflicting_lanes | bool | 境界に競合するレーンレットを含めるフラグ | False         |

#### 車線逸脱防止の設定

## Road Obstacle Filter

### `threshold_obstacle_corner_distance`

障害物と車両のコーナーの間隔の閾値。
この値を超えると、障害物として認識されます。

### `threshold_obstacle_corner_angle`

障害物と車両のコーナーの角度の閾値。
この値を超えると、障害物が車両の前方にあります。

### `threshold_obstacle_length`

障害物の長さの閾値。
この値を超えると、障害物が長い障害物と認識されます。

## Prediction Module

### `max_prediction_distance`

予測距離の最大値。

### `duration_to_predict`

予測する持続時間。

## Planning Component

### `min_radius`

最小曲率半径。

### `max_acceleration`

加速度の最大値。

### `max_deceleration`

減速度の最大値。

### `max_lateral_jerk`

横加速度の最大値。

## Optional Filters

### `speed_inv_time`

速度に反比例する時間。

### `speed_time`

速度に比例する時間。

### `enable_post_resampling`

再サンプリング後の有効化。

### `enable_resample`

再サンプリングの有効化。

### `smoothing_gain`

平滑化のゲイン。

### `gradient_filter_length`

勾配フィルタの長さ。

## Autoware Specific Parameters

### `enable_yaw_rate_constraint`

ヨーレート制約の有効化。

### `max_yaw_rate`

最大ヨーレート。

| 名前                     | タイプ                       | 説明                                                 | デフォルト値 |
| :----------------------- | :------------------------- | :---------------------------------------------------------- | :------------ |
| boundary_types_to_detect | std::vector\<std::string\> |boundary_departure_checker で検出する line_string タイプ | [road_border] |

### コアパラメータ

| 名前                       | タイプ   | 説明                                                                                                | デフォルト値 |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------------------- | :------------ |
| footprint_margin_scale     | double | フットプリントの余白を拡張する係数。標準偏差に 1 を乗算。                                                  | 1.0           |
| footprint_extra_margin     | double | 車線逸脱をチェックするときのフットプリントの余白を拡張する係数。                                       | 0.0           |
| resample_interval          | double | 軌跡をリサンプルするときの点間の最小ユークリッド距離 [m]。                                         | 0.3           |
| max_deceleration           | double | ブレーキ距離を計算するときの最大減速度。                                                            | 2.8           |
| delay_time                 | double | ブレーキを動作させるのにかかる遅延時間 [秒]。                                                           | 1.3           |
| max_lateral_deviation      | double | 車両座標系の最大横偏差 [m]。                                                                      | 2.0           |
| max_longitudinal_deviation | double | 車両座標系の最大縦偏差 [m]。                                                                      | 2.0           |
| max_yaw_deviation_deg      | double | 軌跡からの自身のヨー偏差の最大値 [度]。                                                            | 60.0          |

