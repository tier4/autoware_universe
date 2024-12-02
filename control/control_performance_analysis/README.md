## 制御性能分析

## 目的

`control_performance_analysis`は制御モジュールの追従性能を分析し、車両の運転状況を監視するためのパッケージです。

このパッケージは、制御モジュールの結果を定量化するためのツールとして使用されます。
そのため、自動運転のコアロジックには干渉しません。

Planing、制御、車両からのさまざまな入力に基づいて、このパッケージで定義された`control_performance_analysis::msg::ErrorStamped`として分析結果を発行します。

`ErrorStamped`メッセージのすべての結果は、カーブのフレネフレームで計算されます。エラーと速度エラーは、以下の論文を使用して計算されます。

<!-- cspell: ignore Werling Moritz Groell Lutz Bretthauer Georg -->

`Werling, Moritz & Groell, Lutz & Bretthauer, Georg. (2010). Invariant Trajectory Tracking With a Full-Size Autonomous Road Vehicle. IEEE Transactions on Robotics. 26. 758 - 765. 10.1109/TRO.2010.2052325.`

計算に興味がある場合は、Cセクションでエラーとエラー速度の計算を見ることができます「オリエンテーション制御付きの漸近的軌跡追従」。

エラー加速度の計算は、上記の速度計算に基づいて行われます。エラー加速度の計算は以下のとおりです。

![CodeCogsEqn](https://user-images.githubusercontent.com/45468306/169027099-ef15b306-2868-4084-a350-0e2b652c310f.png)

## 入出力

### 入力トピック

| 名前                                     | 型                                       | 説明                                       |
| ---------------------------------------- | ------------------------------------------ | ------------------------------------------- |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs::msg::Trajectory`    | Planningモジュールからの出力経路             |
| `/control/command/control_cmd`           | `autoware_control_msgs::msg::Control`        | Controlモジュールからの出力Controlコマンド   |
| `/vehicle/status/steering_status`        | `autoware_vehicle_msgs::msg::SteeringReport` | 車両からのSteering情報                     |
| `/localization/kinematic_state`          | `nav_msgs::msg::Odometry`                    | オドメトリからのTwistを使用                   |
| `/tf`                                    | `tf2_msgs::msg::TFMessage`                   | TFから自己位置を抽出                        |

### 出力トピック

| 名前 | タイプ | 説明 |
|---|---|---|
| `/control_performance/performance_vars` | `control_performance_analysis::msg::ErrorStamped` | パフォーマンス分析の結果 |
| `/control_performance/driving_status` | `control_performance_analysis::msg::DrivingMonitorStamped` | 走行状況 (加速度、ジャークなど) の監視 |

### 出力

#### control_performance_analysis::msg::DrivingMonitorStamped

| 名前                           | タイプ | 説明 |
| ---------------------------- | ----- | ---------------------------------- |
| `longitudinal_acceleration`  | float | [m / s^2] |
| `longitudinal_jerk`          | float | [m / s^3] |
| `lateral_acceleration`       | float | [m / s^2] |
| `lateral_jerk`               | float | [m / s^3] |
| `desired_steering_angle`     | float | [rad] |
| `controller_processing_time` | float | 直前の2つの制御コマンドメッセージ間のタイムスタンプ [ms] |

#### control_performance_analysis::msg::ErrorStamped

| 名前                                   | 型  | 説明                                                                                                              |
| --------------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------- |
| `lateral_error`                           | float | [m]                                                                                                                |
| `lateral_error_velocity`                  | float | [m / s]                                                                                                              |
| `lateral_error_acceleration`              | float | [m / s²]                                                                                                            |
| `longitudinal_error`                      | float | [m]                                                                                                                |
| `longitudinal_error_velocity`             | float | [m / s]                                                                                                              |
| `longitudinal_error_acceleration`         | float | [m / s²]                                                                                                            |
| `heading_error`                           | float | [rad]                                                                                                                |
| `heading_error_velocity`                  | float | [rad / s]                                                                                                            |
| `control_effort_energy`                   | float | [u * R * u^T]                                                                                                                |
| `error_energy`                            | float | lateral_error² + heading_error²                                                                                        |
| `value_approximation`                     | float | V = xPx' ; Value function from DARE Lyap matrix P                                                                          |
| `curvature_estimate`                      | float | [1 / m]                                                                                                                |
| `curvature_estimate_pp`                   | float | [1 / m]                                                                                                                |
| `vehicle_velocity_error`                  | float | [m / s]                                                                                                                |
| `tracking_curvature_discontinuity_ability` | float | 曲率変化追従能力 [`abs(delta(curvature)) / (1 + abs(delta(lateral_error))`] |

## パラメータ

| 名前                                   | タイプ              | 説明                                                               |
| -------------------------------------- | ------------------- | --------------------------------------------------------------------- |
| `curvature_interval_length`            | double              | 現在カーブの推定に使用                                               |
| `prevent_zero_division_value`          | double              | 0除算回避の値。デフォルトは`0.001`                                   |
| `odom_interval`                        | unsigned integer    | odomメッセージ間のインターバル。カーブを滑らかにするには増加する |
| `acceptable_max_distance_to_waypoint` | double              | 軌道点と車両間の最大距離 [m]                                      |
| `acceptable_max_yaw_difference_rad`    | double              | 軌道点と車両間の最大ヨー角差 [rad]                                |
| `low_pass_filter_gain`                 | double              | ローパスフィルタゲイン                                                |

## 操作方法

1. シミュレーションと制御モジュールを起動したら、`control_performance_analysis.launch.xml` を起動してください。
2. ドライビングモニターとエラー変数がトピックに表示されるはずです。
3. 結果を視覚化したい場合は、`Plotjuggler` を使用し、レイアウトとして `config/controller_monitor.xml` を使用してください。
4. レイアウトをインポートしたら、以下にリストされたトピックを指定してください。

> - /localization/kinematic_state
> - /vehicle/status/steering_status
> - /control_performance/driving_status
> - /control_performance/performance_vars

5. `Plotjuggler` では、統計値（最大値、最小値、平均値）を csv ファイルとしてエクスポートできます。この統計を使用して、制御モジュールを比較します。

## 今後の改善点

- カットオフ周波数、微分方程式、離散状態空間アップデートによる LPF を実装する。

