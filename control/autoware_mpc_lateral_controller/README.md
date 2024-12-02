    - What inputs does it use?
    - What outputs does it produce? -->

### Inputs

- `/planning/decision/trajectory` : The trajectory to be followed.
- `/vehicle/odometry` : The current pose of the vehicle.
- `/config/mpc_lateral_controller` : The configuration of mpc lateral controller node.

### Outputs

- `/lateral_controller/lateral/output` : The computed lateral control command.
- `/lateral_controller/lateral/diagnostics` : The diagnostics of lateral controller.

### API

The node also provides a service to change the current mode of the lateral controller:

- Service: `/lateral_controller/lateral/set_mode`
- Request: `std_srvs/SetBool`
- Response: `std_srvs/SetBool`

## Future plans

<!-- Optional -->
<!-- Things to consider:
    - What improvements do we want to make in the future? -->

- [Future Plan] : Add a state estimatior.

## References

[1] Leone, G., & Coscia, M. (2018, May). Adaptive MPC for path tracking of autonomous vehicles with closed-loop system identification. In 2018 IEEE Intelligent Vehicles Symposium (IV) (pp. 1108-1113). IEEE.
[2] Xu, Y., Li, N., Scudieri, F., & Savaresi, S. M. (2018, June). Robust model predictive control for path tracking with dynamic references and joint state-input constraints. In 2018 Annual American Control Conference (ACC) (pp. 1355-1360). IEEE.

### 入力

[controller_node](../autoware_trajectory_follower_node/README.md) から以下を設定します。

- `autoware_planning_msgs/Trajectory` : 追跡する参照軌跡
- `nav_msgs/Odometry`: 現在のオドメトリ
- `autoware_vehicle_msgs/SteeringReport`: 現在のステアリング

### 出力

コントローラノードに以下を含む LateralOutput を返します。

- `autoware_control_msgs/Lateral`
- LateralSyncData
  - ステアリング角の収束

次のメッセージを発行します。

| 名前 | タイプ | 説明 |
|---|---|---|
| `~/output/predicted_trajectory` | `autoware_planning_msgs::Trajectory` | MPCで計算された予測軌跡。コントローラーがプランニング軌跡から大きく逸脱するなどの緊急時にトラジェクとリーのサイズは空になります。 |

### MPCクラス

`MPC`クラス（`mpc.hpp`で定義）はMPCアルゴリズムとのインタフェースを提供します。
車両モデル、QPソルバー、追従する基準軌跡が（`setVehicleModel()`、`setQPSolver()`、`setReferenceTrajectory()`を使用して）設定されると、現在の操舵、速度、自車位置を関数`calculateMPC()`に提供して、ラテラル制御コマンドを計算できます。

### パラメータの説明

`param/lateral_controller_defaults.param.yaml`で定義されたデフォルトのパラメータは、時速40km未満の運転用にAutonomouStuff Lexus RX 450hに合わせて調整されています。

#### システム

| 名前                      | 型    | 説明                                                                             | デフォルト値 |
| :------------------------ | :------ | :------------------------------------------------------------------------------- | :------------ |
| **traj_resample_dist**        | double  | 再サンプリングのウェイポイント距離 [m]                                        | 0.1           |
| **use_steer_prediction**      | boolean | ステアリング予測を使用するフラグ（ステアリング測定値は使用しない）              | false         |
| **admissible_position_error** | double  | 位置誤差がこの値 [m] より大きい場合、車両を停止                               | 5.0           |
| **admissible_yaw_error_rad**  | double  | ヨー角誤差がこの値 [rad] より大きい場合、車両を停止                             | 1.57          |
| **use_delayed_initial_state** | boolean | 予測軌道の初期状態としてx0_delayedを使用するフラグ                              | true          |

#### パススムージング

| 名前                                | タイプ | 説明                                                                                                                                                                           | デフォルト値 |
| :---------------------------------- | :------ | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| enable_path_smoothing               | boolean | path smoothing flag。path resamplingを使用してresamplingノイズを低減する場合にtrueにする必要があります。                                                              | false         |
| path_filter_moving_ave_num        | int     | path smoothing用の移動平均フィルタのデータポイントの数                                                                                                                          | 25            |
| curvature_smoothing_num_traj      | int     | 軌道のカーブ度の計算に使用されるポイントのインデックス距離:p(i-num)、p(i)、p(i+num)。数値が大きいほど、値のノイズが少なくなります。                                         | 15            |
| curvature_smoothing_num_ref_steer | int     | 基準ステアリングコマンドのカーブ度の計算に使用されるポイントのインデックス距離:p(i-num)、p(i)、p(i+num)。数値が大きいほど、値のノイズが少なくなります。                 | 15            |

#### 軌道延長

| 名称                                  | タイプ    | 説明                                               | デフォルト値 |
| :------------------------------------ | :------ | :-------------------------------------------------- | :------------ |
| extend_trajectory_for_end_yaw_control | boolean | 最終yaw制御のための軌跡拡張フラグ | true          |

#### MPC最適化

| 名前                                                | タイプ | 説明                                                                                                                    | デフォルト値 |
| :-------------------------------------------------- | :----- | :------------------------------------------------------------------------------------------------------------------ | :------------ |
| qp_solver_type                                      | 文字列 | QPソルバーオプション（以下で詳細に説明）                                                                             | "osqp"        |
| mpc_prediction_horizon                              | 整数   | MPCの合計予測ステップ                                                                                                 | 50            |
| mpc_prediction_dt                                   | double | １ステップの予測期間 [s]                                                                                                | 0.1           |
| mpc_weight_lat_error                                | double | 横方向の誤差に対する重み                                                                                               | 1.0           |
| mpc_weight_heading_error                            | double | 進行方向の誤差に対する重み                                                                                             | 0.0           |
| mpc_weight_heading_error_squared_vel                | double | 進行方向の誤差 \* 速度に対する重み                                                                                    | 0.3           |
| mpc_weight_steering_input                           | double | ステアリングの誤差（ステアリングコマンド - 基準ステアリング）に対する重み                                                | 1.0           |
| mpc_weight_steering_input_squared_vel               | double | ステアリングの誤差（ステアリングコマンド - 基準ステアリング）\* 速度に対する重み                                       | 0.25          |
| mpc_weight_lat_jerk                                 | double | 横方向のジャーク（steer(i) - steer(i-1)) \* 速度に対する重み                                                           | 0.1           |
| mpc_weight_steer_rate                               | double | ステアリングレート [rad/s] に対する重み                                                                              | 0.0           |
| mpc_weight_steer_acc                                | double | ステアリングレートの微分の微分 [rad/ss] に対する重み                                                                 | 0.000001      |
| mpc_low_curvature_weight_lat_error                  | double | [曲率の小さい軌道で使用する] 横方向の誤差に対する重み                                                                  | 0.1           |
| mpc_low_curvature_weight_heading_error              | double | [曲率の小さい軌道で使用する] 進行方向の誤差に対する重み                                                                  | 0.0           |
| mpc_low_curvature_weight_heading_error_squared_vel  | double | [曲率の小さい軌道で使用する] 進行方向の誤差 \* 速度に対する重み                                                       | 0.3           |
| mpc_low_curvature_weight_steering_input             | double | [曲率の小さい軌道で使用する] ステアリングの誤差（ステアリングコマンド - 基準ステアリング）に対する重み             | 1.0           |
| mpc_low_curvature_weight_steering_input_squared_vel | double | [曲率の小さい軌道で使用する] ステアリングの誤差（ステアリングコマンド - 基準ステアリング）\* 速度に対する重み | 0.25          |
| mpc_low_curvature_weight_lat_jerk                   | double | [曲率の小さい軌道で使用する] 横方向のジャーク（steer(i) - steer(i-1)) \* 速度に対する重み                               | 0.0           |
| mpc_low_curvature_weight_steer_rate                 | double | [曲率の小さい軌道で使用する] ステアリングレート [rad/s] に対する重み                                                     | 0.0           |
| mpc_low_curvature_weight_steer_acc                  | double | [曲率の小さい軌道で使用する] ステアリングレートの微分の微分 [rad/ss] に対する重み                                      | 0.000001      |
| mpc_low_curvature_thresh_curvature                  | double | "low_curvature"パラメータを使用する曲率の閾値                                                                       | 0.0           |
| mpc_weight_terminal_lat_error                       | double | マトリックス`Q`内のターミナル横方向誤差の重み（MPCの安定性を向上させるため）                                        | 1.0           |
| mpc_weight_terminal_heading_error                   | double | マトリックス`Q`内のターミナル進行方向誤差の重み（MPCの安定性を向上させるため）                                        | 0.1           |
| mpc_zero_ff_steer_deg                               | double | フィードフォワードアングルがゼロになる閾値                                                                             | 0.5           |
| mpc_acceleration_limit                              | double | 車両の加速度に対する制限                                                                                              | 2.0           |
| mpc_velocity_time_constant                          | double | 速度スムージングに使用される時定数                                                                                   | 0.3           |
| mpc_min_prediction_length                           | double | 最小予測長                                                                                                         | 5.0           |

#### 車両モデル

| 名称                                   | タイプ     | 説明                                                                        | デフォルト値        |
| :------------------------------------ | :------- | :--------------------------------------------------------------------------------- | :------------------- |
| vehicle_model_type                   | 文字列   | MPC 予測用の車両モデルタイプ                                                  | "kinematics"         |
| input_delay                          | double   | 遅延補償のためのステアリング入力遅延時間                                     | 0.24                 |
| vehicle_model_steer_tau              | double   | ステアリングダイナミクス時定数 (1 次元近似) [s]                               | 0.3                  |
| steer_rate_lim_dps_list_by_curvature | [double] | 曲率依存のステアリング角度レート制限リスト [deg/s]                          | [40.0, 50.0, 60.0]   |
| curvature_list_for_steer_rate_lim    | [double] | アーリーカーブ方向補間のための曲率リスト (昇順) [/m]                         | [0.001, 0.002, 0.01] |
| steer_rate_lim_dps_list_by_velocity  | [double] | 速度依存のステアリング角度レート制限リスト [deg/s]                           | [60.0, 50.0, 40.0]   |
| velocity_list_for_steer_rate_lim     | [double] | アーリーカーブ方向補間のための速度リスト (昇順) [m/s]                           | [10.0, 15.0, 20.0]   |
| acceleration_limit                   | double   | 軌道速度変更のための加速度制限 [m/ss]                                     | 2.0                  |
| velocity_time_constant               | double   | 軌道速度変更のための速度ダイナミクス時定数 [s]                           | 0.3                  |

#### ノイズ低減用のローパスフィルタ

| 名前                      | タイプ   | 説明                                                         | デフォルト値 |
| :------------------------ | :----- | :------------------------------------------------------------------ | :------------ |
| steering_lpf_cutoff_hz    | double | ステアリング出力コマンドのローパスフィルタのカットオフ周波数 [Hz] | 3.0           |
| error_deriv_lpf_cutoff_hz | double | 誤差微分のローパスフィルタのカットオフ周波数 [Hz]        | 5.0           |

#### ストップ状態

| 名前                                            | 種類    | 説明                                                                                             | デフォルト値 |
| :-------------------------------------------- | :------ | :------------------------------------------------------------------------------------------------- | :------------ |
| stop_state_entry_ego_speed <sup>\*1</sup>     | 実数    | 停止状態のしきい値として使用される自車速度                                                    | 0.001         |
| stop_state_entry_target_speed <sup>\*1</sup> | 実数    | 停止状態のしきい値として使用される目標速度                                                       | 0.001         |
| converged_steer_rad                           | 実数    | ステアリング収束のしきい値                                                                        | 0.1           |
| keep_steer_control_until_converged            | ブール | ステアリングが収束するまでステアリング制御を維持する                                           | true          |
| new_traj_duration_time                        | 実数    | 新しい経路とみなされる時間に関するしきい値                                                  | 1.0           |
| new_traj_end_dist                             | 実数    | 新しい経路とみなされる、経路の終点間の距離に関するしきい値                                  | 0.3           |
| mpc_converged_threshold_rps                   | 実数    | 最適化の出力の収束を確保するためのしきい値 (停止状態で使用)                                   | 0.01          |

(\*1) 不必要な操舵動作を防ぐために、停止状態では前回の値に操舵コマンドが固定されます。

#### ステアリングオフセット

`steering_offset` ネームスペースで定義されています。このロジックはできるだけシンプルに設計されており、設計パラメーターは最小限に抑えられています。

| 項目                                           | タイプ    | 説明                                                                                                 | デフォルト値   |
| :---------------------------------------------- | :------ | :-------------------------------------------------------------------------------------------------- | :------------ |
| enable_auto_steering_offset_removal           | boolean | ステアリングのオフセットを推定して補正を適用する                                                  | true          |
| update_vel_threshold                            | double  | この値より速度が低い場合、オフセット推定にはデータを使用しない                                     | 5.56          |
| update_steer_threshold                          | double  | この値よりステアリング角度が大きい場合、オフセット推定にはデータを使用しない。                         | 0.035         |
| average_num                                      | int     | この数のデータの平均が、ステアリングオフセットとして使用される。                                      | 1000          |
| steering_offset_limit                            | double  | オフセット補正に適用する角度の制限。                                                                     | 0.02          |

##### ダイナミクスモデル（WIP）

| 名前          | 型   | 説明                                | デフォルト値    |
| :------------ | :----- | :---------------------------------- | :-------------- |
| cg_to_front_m | double | baselinkからフロントアクスルまでの距離 [m] | 1.228         |
| cg_to_rear_m  | double | baselinkからリアアクスルまでの距離 [m] | 1.5618        |
| mass_fl       | double | フロント左タイヤにかかる質量 [kg]        | 600           |
| mass_fr       | double | フロント右タイヤにかかる質量 [kg]       | 600           |
| mass_rl       | double | リア左タイヤにかかる質量 [kg]         | 600           |
| mass_rr       | double | リア右タイヤにかかる質量 [kg]        | 600           |
| cf            | double | フロントコーナリングパワー [N/rad]       | 155494.663    |
| cr            | double | リアコーナリングパワー [N/rad]        | 155494.663    |

#### デバッグ

| 名称                       | タイプ    | 説明                                                                         | デフォルト値 |
| :------------------------- | :------ | :------------------------------------------------------------------------------- | :------------ |
| publish_debug_trajectories | ブール | 予測した経路と再サンプリングされた参照経路をデバッグ用に公開           | true          |

### MPCパラメータのチューニング方法

#### 運動情報の設定

まず、ホイールベース（前輪と後輪の間の距離を表す`wheelbase`）や最大タイヤ操舵角（`max_steering_angle`）などの適切な車輌運動学パラメータを設定することが重要です。これらのパラメータは`vehicle_info.param.yaml`に設定する必要があります。

#### 動力学情報のの設定

次に、動力学モデルの適切なパラメータを設定する必要があります。これらには、操舵動力学のタイムコンスタント`steering_tau`とタイムディレイ`steering_delay`、および速度動力学の最大加速度`mpc_acceleration_limit`とタイムコンスタント`mpc_velocity_time_constant`が含まれます。

#### 入力情報の確認

入力情報の正確性を確認することも重要です。後輪中心の速度[m/s]やタイヤの操舵角[rad]などの情報が必要です。入力情報の誤りによって性能が低下したという報告が頻繁にあることに注意してください。たとえば、タイヤ半径の想定外の差によって車両速度にオフセットが発生したり、ステアリングギア比または中間点のずれによってタイヤ角を正確に測定できなかったりする場合があります。複数のセンサー（例：統合された車速とGNSS位置情報、ステアリング角とIMU角速度）から情報と照合し、MPCの入力情報が適切であることを確認することをお勧めします。

#### MPCウェイトのチューニング

次に、MPCのウェイトを調整します。チューニングの簡単なアプローチの1つは、横方向偏差（`weight_lat_error`）のウェイトを一定に保ち、ステアリング発動のウェイト（`weight_steering_input`）を変更しながら、ステアリングの振動と制御精度のトレードオフを観察することです。

ここで、`weight_lat_error`は経路追従における横方向の誤差を抑制し、`weight_steering_input`は、ステアリング角をパスの曲率によって決定される標準値に調整します。`weight_lat_error`が大きい場合、ステアリングが大きく移動して精度が向上しますが、振動が発生する可能性があります。一方、`weight_steering_input`が大きい場合、ステアリングは追従誤差にあまり反応せず、安定した走行は可能ですが、追従精度が低下する可能性があります。

手順は次のとおりです。

1. `weight_lat_error` = 0.1、`weight_steering_input` = 1.0、他のウェイトを0に設定します。
2. 運転中に車両が振動する場合は、`weight_steering_input`を大きく設定します。
3. 追従精度が低い場合は、`weight_steering_input`を小さく設定します。

高速範囲でのみ効果を調整したい場合は、`weight_steering_input_squared_vel`を使用できます。このパラメータは、高速範囲でのステアリングウェイトに対応します。

#### ウェイトの説明

- `weight_lat_error`: 横方向の追従誤差を減らします。これはPIDのPゲインのように機能します。
- `weight_heading_error`: まっすぐ走行します。これはPIDのDゲインのように機能します。
- `weight_heading_error_squared_vel_coeff`：高速範囲でまっすぐ走行します。
- `weight_steering_input`: 追従の振動を減らします。
- `weight_steering_input_squared_vel_coeff`: 高速範囲で追従の振動を減らします。
- `weight_lat_jerk`: 横方向のジャークを減らします。
- `weight_terminal_lat_error`: 安定性のために、通常の横方向ウェイト`weight_lat_error`よりも高い値を設定することをお勧めします。
- `weight_terminal_heading_error`: 安定性のために、通常のヘディングウェイト`weight_heading_error`よりも高い値を設定することをお勧めします。

#### その他のチューニングのヒント

他のパラメータを調整するためのヒントを以下に示します。

- 理論的には、ターミナルウェイト`weight_terminal_lat_error`と`weight_terminal_heading_error`を増やすことで、追従安定性を向上させることができます。この方法は効果的であることが証明されています。
- ターミナルホライゾンを大きくし、`prediction_sampling_time`を小さくすると、追従性能が向上します。ただし、計算コストが高くなります。
- 軌道曲率に応じてウェイトを変更したい場合（たとえば、急カーブを運転していてより大きなウェイトが必要な場合）、`mpc_low_curvature_thresh_curvature`を使用し、`mpc_low_curvature_weight_**`ウェイトを調整します。
- 車両速度と軌道の曲率に基づいてステアリングレート制限を調整したい場合は、`steer_rate_lim_dps_list_by_curvature`、`curvature_list_for_steer_rate_lim`、`steer_rate_lim_dps_list_by_velocity`、`velocity_list_for_steer_rate_lim`の値を変更できます。これにより、高速走行中はステアリングレート制限を強化したり、カーブ中は緩和したりできます。
- ターゲット曲率がギザギザに見える場合は、正確な曲率計算のために`curvature_smoothing`を調整することが非常に重要です。値が大きいほど曲率計算が滑らかになり、ノイズが低減されますが、フィードフォワード計算の遅延が発生し、パフォーマンスが低下する可能性があります。
- `steering_lpf_cutoff_hz`の値を調整して、計算ノイズを強制的に低減することも効果的です。これは、最終層にインストールされた2次バターワースフィルタのカットオフ周波数を指します。カットオフ周波数が低いほどノイズ低減が強くなりますが、操作遅延も発生します。
- 車両が軌道から横方向に継続的に逸脱する場合は、ステアリングセンサーまたは自己位置推定のオフセットが原因であることが最も多いです。これらのバイアスをMPCに入力する前に排除することをお勧めしますが、MPC内でこのバイアスを取り除くことも可能です。これを使用するには、`enable_auto_steering_offset_removal`をtrueに設定し、ステアリングオフセットリムーバーを有効にします。ステアリングオフセット推定ロジックは、ステアリングが中央に近く、高速で走行しているときに機能し、オフセットを取り除きます。
- カーブでのステアリングの開始が遅れる場合は、多くの場合、ステアリングモデルの遅延時間とタイムコンスタントが正しくありません。`input_delay`と`vehicle_model_steer_tau`の値を再確認してください。さらに、デバッグ情報の一部として、MPCはMPCモデルで想定される現在のステアリング角を出力します。そのステアリング角が実際のステアリング角と一致するかどうかを確認してください。

## 参考資料/外部リンク

<!-- オプション -->

- [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", Robotics Institute, Carnegie Mellon University, February 2009.

**自動運転ソフトウェア**

**[Autoware](https://www.autoware.org/)**

### プランニング コンポーネント

- **パス計画 (Path Planning)**:
  - 車両が環境内を安全かつ効率的に移動するための経路を生成する。

- **動作計画 (Motion Planning)**:
  - 車両が障害物や他の車両を避けながら、経路に沿って移動するための速度プロファイルとステアリング アングルを生成する。

- **コントローラー アロケーション (Controller Allocation)**:
  - さまざまなタイプの車両制御（ステアリング、加速、ブレーキなど）をアクチュエーターに割り当て、システム全体の性能を最適化する。

### 知覚 コンポーネント

- **3D 点群処理**
  - センサーから取得した 3D 点群データを分析して、周囲環境のマップを作成する。

- **物体検出**
  - 点群データから車両、歩行者、自転車などの周囲の物体を検出する。

- **セマンティック セグメンテーション**
  - 点群データを分析して、車道、歩道、建物などのシーンのセマンティック セグメンテーションを作成する。

### ローカリゼーション コンポーネント

- **自己位置特定**
  - 車両の現在の位置と姿勢を、センサーデータと地図情報を使用して特定する。

- **マッピング**
  - 車両が移動するときに周囲環境のマップを更新し、自己位置特定の精度を向上させる。

### センサー フュージョン コンポーネント

- **センサー データ フュージョン**
  - 複数のセンサー（レーダー、LiDAR、カメラなど）からのデータを統合して、周囲環境のより完全で正確な表現を作成する。

### システム マネージメント コンポーネント

- **タスク マネージャ**
  - 自動運転システムのさまざまなコンポーネントのタスクと依存関係を管理する。

- **データ レコーダー**
  - 自動運転システムの動作とパフォーマンスに関するデータを記録する。

- **リアルタイム システム**
  - 自動運転システムを安全かつリアルタイムで動作させるための低レイテンシ オペレーティング システムとミドルウェアを提供する。

