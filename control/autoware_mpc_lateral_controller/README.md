# MPC横方向コントローラ

これは`autoware_trajectory_follower_node`パッケージ内の横方向コントローラのノードに関する設計文書です。

## 目的 / ユースケース

<!-- 必須 -->
<!-- 考慮事項:
    - なぜこの機能を実装したのか -->

このノードは、パスに従っているときに横方向の制御コマンド（ステアリング角とステアリング速度）を作成するために使用されます。

## 設計

<!-- 必須 -->
<!-- 考慮事項:
    - どのように動作するのか -->

このノードは、正確なパストラック用に線形モデル予測制御（MPC）の実装を使用しています。 MPCは車両モデルを使用して、制御コマンドから得られる軌道をシミュレーションします。制御コマンドの最適化は2次計画問題（QP）として定式化されます。

さまざまな車両モデルが実装されています。

- kinematics：ステアリング1次の遅れを持つ自転車運動学モデル。
- kinematics_no_delay：ステアリング遅れのない自転車運動学モデル。
- dynamics：スリップ角を考慮する自転車動力学モデル。
  運動学モデルがデフォルトで使用されています。詳細については、参照 [1]を参照してください。

最適化には2次計画問題（QP）ソルバーが使用され、現在2つのオプションが実装されています。

<!-- cspell: ignore ADMM -->

- unconstraint_fast：eigenで拘束のないQPを解くために最小二乗法を使用します。
- [osqp](https://osqp.org/)：次の[ADMM](https://web.stanford.edu/~boyd/papers/admm_distr_stats.html)アルゴリズムを実行します（詳細については、[OSQPの引用](https: //web.stanford.edu/~boyd/papers/admm_distr_stats.html)セクションを参照してください）。

### フィルタリング

適切なノイズ低減にはフィルタリングが必要です。
バターワースフィルタを使用して、MPCの入力として使用されるヨーエラーと横方向エラーを処理し、出力ステアリング角を調整します。
ノイズ低減性能が十分であれば、他のフィルタリング方法を検討できます。
たとえば、移動平均フィルタは適しておらず、フィルタリングなしよりも悪い結果を生じる可能性があります。

## 想定 / 既知の制限

<!-- 必須 -->

リファレンストラジェクトの最初の点が現在の自車位置にあるか、その前方に位置している場合、追跡は正確ではありません。

## 入力 / 出力 / API

- 入力:

  - trajectory :リファレンストラジェクト（`autoware_api_msgs/Trajectory`）
  - vehicle_state :現在の車両のステータス（`autoware_auto_msgs/VehicleState`）
  - current_pose :自車位置（`geometry_msgs/PoseStamped`）
  - prev_pose :前の自車位置（`geometry_msgs/PoseStamped`）
  - planned_trajectory :計画された軌道（`autoware_msgs/PlannedTrajectory`）

- 出力:

  - vehicle_control_cmd :車両制御コマンド（`autoware_auto_msgs/VehicleControlCommand`）

- API:
  - 車両制御コマンドを取得するためのAPIはありません。

### 入力

次のものを [controller_node](../autoware_trajectory_follower_node/README.md) から設定します

- `autoware_planning_msgs/Trajectory` : 追従する基準軌跡
- `nav_msgs/Odometry`: 現在のオドメトリ
- `autoware_vehicle_msgs/SteeringReport`: 現在のステア

### 出力

次の内容を含む LateralOutput を controller node に返します

- `autoware_control_msgs/Lateral`
- LateralSyncData
  - ステア角収束

### MPC クラス

`MPC` クラス (`mpc.hpp` で定義) は、MPC アルゴリズムとのインターフェイスを提供します。
車両モデル、QP ソルバー、および追従する基準軌跡が設定されると (`setVehicleModel()`, `setQPSolver()`, `setReferenceTrajectory()`)
、現在のステア、速度、位相を `calculateMPC()` 関数に提供することで、横方向制御コマンドを計算できます。

### パラメーターの説明

`param/lateral_controller_defaults.param.yaml` で定義された既定のパラメーターは、40km/h 未満の走行に対応するように自律走行用 Lexus RX 450h に調整されています。

#### システム

| 名前                      | タイプ  | 説明                                                               | デフォルト値 |
| :------------------------ | :------ | :----------------------------------------------------------------- | :----------- |
| traj_resample_dist        | double  | 再サンプリングにおけるウェイポイント間の距離 [m]                   | 0.1          |
| use_steer_prediction      | boolean | ステアリング予測を使用するフラグ（ステアリング測定値は使用しない） | false        |
| admissible_position_error | double  | 追従位置誤差がこの値 [m] より大きい場合に車両を停止する            | 5.0          |
| admissible_yaw_error_rad  | double  | 追従ヨー角誤差がこの値 [rad] より大きい場合に車両を停止する        | 1.57         |
| use_delayed_initial_state | boolean | 予測軌道の初期状態として `x0_delayed` を使用するフラグ             | true         |

#### パススムージング

このモジュールでは、`post resampling`時に発生する経路の不連続性につながる目標経路の僅かな 'jerk' を排除します。この不連続性は、経路上の他の車両や障害物との衝突を避けるために必要です。

トリガーは、自車位置の更新、経路の変更、経路の`post resampling`の更新です。

#### レベル0の経路

レベル0のレイヤーでは、Smoothed Path Providerによって、パスのスムージングによって生成された経路が提供されます。

#### レベル1のPlanning

レベル1のPlanningコンポーネントは、以下を行います。

- スムーズな経路を`post resampling`に使用します。
- スムーズな経路に基づいて、velocity 逸脱量とacceleration 逸脱量を計算します。
- Velocity ControllerとAcceleration Controllerに逸脱量を通知します。

| 名称                              | タイプ  | 説明                                                                                                                                        | デフォルト値 |
| :-------------------------------- | :------ | :------------------------------------------------------------------------------------------------------------------------------------------ | :----------- |
| path_smoothingを有効にする        | boolean | パスの平滑化フラグ。リサンプリングノイズを低減するためにパスリサンプリングを使用する場合は、`True`にします。                                | `False`      |
| path_filter_moving_ave_num        | int     | path smoothingのために移動平均フィルタのデータポイントの数                                                                                  | `25`         |
| curvature_smoothing_num_traj      | int     | 軌道の曲率計算に使用される点のインデックス距離：p(i-num)、p(i)、p(i+num)。`num`が大きいほどノイズの少ない値になります。                     | `15`         |
| curvature_smoothing_num_ref_steer | int     | 参照ステアリングコマンドの曲率計算に使用される点のインデックス距離：p(i-num)、p(i)、p(i+num)。`num`が大きいほどノイズの少ない値になります。 | `15`         |

#### 軌道予測

### Planningコンポーネントの目的

Planningコンポーネントは、自車位置と目的地などの高レベルの目標に基づいて、最適な軌道生成を行い、それを経路プランに沿って実施する。

### 経路プランニング

#### 経路検索

経路検索は、始点と終点の間の経路を計算するプロセスです。経路検索アルゴリズムには、Dijkstraアルゴリズム、A\*アルゴリズム、Floyd-Warshallアルゴリズムなどがあります。

##### コスト関数

経路検索アルゴリズムは、ノード間のコストに基づいて経路を生成します。コスト関数は、距離、時間、エネルギー消費など、さまざまなファクターを考慮できます。

#### 経路フィッティング

経路フィッティングは、経路検索結果を滑らかな経路に変換するプロセスです。経路フィッティングアルゴリズムには、三次スプライン、NURBS、ベジェ曲線などがあります。

### 軌道生成

#### 軌道生成のフェーズ

軌道生成には、次の3つのフェーズがあります。

1. **計画**：経路プランと自車位置に基づいて、軌道を生成します。
2. **実行**：軌道を追従し、自車を制御します。
3. **再計画**：必要に応じて軌道プランを更新します。

#### 軌道生成アルゴリズム

軌道生成アルゴリズムには、以下のようなものがあります。

- **レファレンストラジャクトリ追従**：レファレンストラジャクトリを生成し、自車をそれに追従させます。
- **最適制御**：最適化手法を使用して、コスト関数を最小化する軌道生成します。
- **機械学習**：機械学習モデルを使用して、軌道を予測します。

### 経路プランの実行

#### 経路プラン追従

経路プラン追従は、自車を経路プランに沿って制御するプロセスです。経路プラン追従コントローラーには、以下のようなものがあります。

- **制約性モデル予測制御（MPC）**：制約を考慮して自車を制御します。
- **占有空間パス計画（OCPP）**：障害物を避けながら自車を制御します。

#### 軌道追従

軌道追従は、自車を軌道に沿って制御するプロセスです。軌道追従コントローラーには、以下のようなものがあります。

- **PIDコントローラー**：誤差を最小化するフィードバック制御を行います。
- **カルマンフィルター**：雑音を推定してより正確な制御を実現します。

### 安全性評価

#### 運動学的検証

運動学的検証は、軌道が物理的に実行可能であることを確認するプロセスです。運動学的検証では、以下の要素を考慮します。

- **速度逸脱量**
- **加速度逸脱量**
- **ジャーク逸脱量**

#### 動力学的検証

動力学的検証は、軌道が車両の動力学を考慮して実行可能であることを確認するプロセスです。動力学的検証では、以下の要素を考慮します。

- **タイヤの滑り**
- **車両の安定性**
- **サスペンションの影響**

#### 障害物検出

障害物検出は、軌道上の障害物を検出するプロセスです。障害物検出には、以下の方法があります。

- **センサーデータ（LiDAR、カメラ、レーダーなど）**
- **地図データ**
- **'post resampling'（前回のサンプルからの差分）**による検出

| 名称                                  | 種類    | 説明                                 | デフォルト値 |
| :------------------------------------ | :------ | :----------------------------------- | :----------- |
| extend_trajectory_for_end_yaw_control | boolean | end yaw 制御のための軌道の延伸フラグ | true         |

#### MPC最適化

| 名前                                                | タイプ | 説明                                                                                             | デフォルト値 |
| :-------------------------------------------------- | :----- | :----------------------------------------------------------------------------------------------- | :----------- |
| qp_solver_type                                      | 文字列 | QPソルバーオプション。以下で詳細に説明します。                                                   | "osqp"       |
| mpc_prediction_horizon                              | 整数   | MPCのステップごとの合計予測                                                                      | 50           |
| mpc_prediction_dt                                   | float  | 1ステップあたりの予測時間[単位：秒]                                                              | 0.1          |
| mpc_weight_lat_error                                | float  | 横方向の逸脱量の重み                                                                             | 1.0          |
| mpc_weight_heading_error                            | float  | ヘディングの逸脱量の重み                                                                         | 0.0          |
| mpc_weight_heading_error_squared_vel                | float  | ヘディングの逸脱量 \* 速度の重み                                                                 | 0.3          |
| mpc_weight_steering_input                           | float  | ステアリングの逸脱量の重み（ステアリングコマンド - 参照ステアリング）                            | 1.0          |
| mpc_weight_steering_input_squared_vel               | float  | ステアリングの逸脱量の重み（ステアリングコマンド - 参照ステアリング） \* 速度                    | 0.25         |
| mpc_weight_lat_jerk                                 | float  | 横方向のジャーク（steer(i) - steer(i-1)) \* 速度 の重み                                          | 0.1          |
| mpc_weight_steer_rate                               | float  | ステアリング速度の重み[単位：rad/s]                                                              | 0.0          |
| mpc_weight_steer_acc                                | float  | ステアリング速度の微分の重み[rad/ss]                                                             | 0.000001     |
| mpc_low_curvature_weight_lat_error                  | float  | [低曲率軌道で使用] 横方向の逸脱量の重み                                                          | 0.1          |
| mpc_low_curvature_weight_heading_error              | float  | [低曲率軌道で使用] ヘディングの逸脱量の重み                                                      | 0.0          |
| mpc_low_curvature_weight_heading_error_squared_vel  | float  | [低曲率軌道で使用] ヘディングの逸脱量 \* 速度の重み                                              | 0.3          |
| mpc_low_curvature_weight_steering_input             | float  | [低曲率軌道で使用] ステアリングの逸脱量の重み（ステアリングコマンド - 参照ステアリング）         | 1.0          |
| mpc_low_curvature_weight_steering_input_squared_vel | float  | [低曲率軌道で使用] ステアリングの逸脱量の重み（ステアリングコマンド - 参照ステアリング） \* 速度 | 0.25         |
| mpc_low_curvature_weight_lat_jerk                   | float  | [低曲率軌道で使用] 横方向のジャーク（steer(i) - steer(i-1)) \* 速度 の重み                       | 0.0          |
| mpc_low_curvature_weight_steer_rate                 | float  | [低曲率軌道で使用] ステアリング速度の重み[単位：rad/s]                                           | 0.0          |
| mpc_low_curvature_weight_steer_acc                  | float  | [低曲率軌道で使用] ステアリング速度の微分の重み[rad/ss]                                          | 0.000001     |
| mpc_low_curvature_thresh_curvature                  | float  | "low_curvature"パラメータを使用するための曲率のしきい値                                          | 0.0          |
| mpc_weight_terminal_lat_error                       | float  | mpcの安定性を向上させるための行列Qの最終横方向の逸脱量の重み                                     | 1.0          |
| mpc_weight_terminal_heading_error                   | float  | mpcの安定性を向上させるための行列Qの最終ヘディングの逸脱量の重み                                 | 0.1          |
| mpc_zero_ff_steer_deg                               | float  | フィードフォワード角度がゼロになるしきい値                                                       | 0.5          |
| mpc_acceleration_limit                              | float  | 車両加速度の制限                                                                                 | 2.0          |
| mpc_velocity_time_constant                          | float  | 速度スムージングに使用される時定数                                                               | 0.3          |
| mpc_min_prediction_length                           | float  | 最小予測の長さ                                                                                   | 5.0          |

#### 車両モデル

| 名称                                 | 種別       | 説明                                                                                     | デフォルト値         |
| ------------------------------------ | ---------- | ---------------------------------------------------------------------------------------- | -------------------- |
| vehicle_model_type                   | 文字列     | mpc予測のための車両モデルの種別                                                          | "kinematics"         |
| input_delay                          | 倍精度     | 遅延補正のためのステアリング入力遅延時間                                                 | 0.24                 |
| vehicle_model_steer_tau              | 倍精度     | ステアリングの動特性時間定数 (1d近似) [秒]                                               | 0.3                  |
| steer_rate_lim_dps_list_by_curvature | 倍精度配列 | 曲率に応じて制限されるステアリング角速度の制限値のリスト [deg/s]                         | [40.0, 50.0, 60.0]   |
| curvature_list_for_steer_rate_lim    | 倍精度配列 | アセンディング順の曲率のリストにより、ステアリング角速度の制限値の補間が決定される [/m]  | [0.001, 0.002, 0.01] |
| steer_rate_lim_dps_list_by_velocity  | 倍精度配列 | 速度に応じて制限されるステアリング角速度の制限値のリスト [deg/s]                         | [60.0, 50.0, 40.0]   |
| velocity_list_for_steer_rate_lim     | 倍精度配列 | アセンディング順の速度のリストにより、ステアリング角速度の制限値の補間が決定される [m/s] | [10.0, 15.0, 20.0]   |
| acceleration_limit                   | 倍精度     | 軌道速度の変更のための加速度の制限 [m/ss]                                                | 2.0                  |
| velocity_time_constant               | 倍精度     | 軌道速度の変更のための速度の動特性時間定数 [s]                                           | 0.3                  |

#### ノイズリダクションのローパスフィルタ

| 名前                      | タイプ | 説明                                                                    | デフォルト値 |
| :------------------------ | :----- | :---------------------------------------------------------------------- | :----------- |
| steering_lpf_cutoff_hz    | double | ステアリング出力コマンドに対するローパスフィルタのカットオフ周波数 [hz] | 3.0          |
| error_deriv_lpf_cutoff_hz | double | エラー微分にローパスフィルタを適用する際のカットオフ周波数 [Hz]         | 5.0          |

#### 停止状態

| 名前                                         | タイプ  | 説明                                                                 | デフォルト値 |
| :------------------------------------------- | :------ | :------------------------------------------------------------------- | :----------: |
| stop_state_entry_ego_speed <sup>\*1</sup>    | double  | 停止状態への移行条件に使用される自車速度のしきい値                   |    0.001     |
| stop_state_entry_target_speed <sup>\*1</sup> | double  | 停止状態への移行条件に使用される目標速度のしきい値                   |    0.001     |
| converged_steer_rad                          | double  | 操舵の収束に使用されるしきい値                                       |     0.1      |
| keep_steer_control_until_converged           | boolean | 操舵が収束するまで操舵制御を維持する                                 |     true     |
| new_traj_duration_time                       | double  | 新しい経路と見なされる時間のしきい値                                 |     1.0      |
| new_traj_end_dist                            | double  | 新しい経路と見なされる軌道終端間の距離のしきい値                     |     0.3      |
| mpc_converged_threshold_rps                  | double  | 最適化の出力が収束したと判断するためのしきい値、停止状態で使用される |     0.01     |

(\*1) 不要なステアリング操作を防止するため、ステアリングコマンドは停止状態では前の値に固定されます。

#### Steer Offset

`steering_offset` 名前空間で定義します。このロジックは、最小設計パラメータで可能な限りシンプルに設計されています。

| 名前                                | 種類    | 説明                                                                         | デフォルト値 |
| :---------------------------------- | :------ | :--------------------------------------------------------------------------- | :----------- |
| enable_auto_steering_offset_removal | boolean | ステアリングオフセットを推定して補正を適用する                               | true         |
| update_vel_threshold                | double  | 速度がこの値より小さい場合、データはオフセット推定に使用されない             | 5.56         |
| update_steer_threshold              | double  | ステアリング角度がこの値より大きい場合、データはオフセット推定に使用されない | 0.035        |
| average_num                         | int     | この数字の平均がステアリングオフセットとして使用される                       | 1000         |
| steering_offset_limit               | double  | オフセット補正に適用する角度制限                                             | 0.02         |

##### 力学モデル（WIP）

| 名称            | 種類   | 説明                                                                  | デフォルト値 |
| :-------------- | :----- | :-------------------------------------------------------------------- | :----------- |
| `cg_to_front_m` | double | ベースリンクからフロントアクスルまでの距離 　　　　　　　　　　　　   | 1.228        |
| `cg_to_rear_m`  | double | ベースリンクからリアアクスルまでの距離 　　　　　　　　　　　　　     | 1.5618       |
| `mass_fl`       | double | フロント左タイヤに加わる質量 　　　　　　　　　　　　　　　　　　　　 | 600          |
| `mass_fr`       | double | フロント右タイヤに加わる質量 　　　　　　　　　　　　　　　　　　　　 | 600          |
| `mass_rl`       | double | リア左タイヤに加わる質量 　　　　　　　　　　　　　　　　　　　　     | 600          |
| `mass_rr`       | double | リア右タイヤに加わる質量 　　　　　　　　　　　　　　　　　　　　     | 600          |
| `cf`            | double | フロントコーナリングパワー 　　　　　　　　　　　　　　　　　　　　   | 155494.663   |
| `cr`            | double | リアコーナリングパワー 　　　　　　　　　　　　　　　　　　　　       | 155494.663   |

#### デバッグ

| 名前                       | 種類   | 説明                                                       | デフォルト値 |
| :------------------------- | :----- | :--------------------------------------------------------- | :----------- |
| publish_debug_trajectories | ブール | デバッグ目的で予測軌跡と再サンプリングされた基準軌跡を公開 | true         |

### MPCパラメータのチューニング方法

#### 運動特性情報の設定

最初に、車両の運動特性に適したパラメータを設定することが重要です。これらには、前輪と後輪の間の距離を表す`wheelbase`や、タイヤの最大操舵角を示す`max_steering_angle`などのパラメータが含まれます。これらのパラメータは`vehicle_info.param.yaml`に設定する必要があります。

#### 動力特性情報の設定

次に、ダイナミクスモデルに適したパラメータを設定する必要があります。これらには、ステアリングダイナミクスに対する時定数`steering_tau`と時間遅れ`steering_delay`、速度ダイナミクスに対する最大加速度`mpc_acceleration_limit`と時定数`mpc_velocity_time_constant`などが含まれます。

#### 入力情報の確認

入力情報が正確であることを確認することも重要です。後輪の中心の速度[m/s]やタイヤの操舵角[rad]などの情報が必要です。入力情報の誤りによりパフォーマンスが低下するという報告が頻繁にあります。たとえば、予期しないタイヤ半径の差のために車両の速度がオフセットされる場合や、ステアリングギア比またはミッドポイントの偏差によりタイヤ角度を正確に測定できない場合があります。複数のセンサーからの情報を比較し（例：統合された車両速度とGNSS位置、ステアリング角とIMU角速度）、MPCの入力情報が適切であることを確認することをお勧めします。

#### MPC重みのチューニング

次に、MPCの重みをチューニングします。チューニングの簡単なアプローチとしては、横方向偏差の重み（`weight_lat_error`）を一定に保ち、ステアリングオシレーションと制御精度のトレードオフを観察しながら入力重み（`weight_steering_input`）を変化させることです。

ここで、`weight_lat_error`はパス追従における横方向誤差を抑えるのに対し、`weight_steering_input`はステアリング角をパスの曲率によって決まる標準値に調整するために働きます。`weight_lat_error`が大きいと、精度を高めるためにステアリングが大きく動き、オシレーションが発生する可能性があります。逆に、`weight_steering_input`が大きいと、ステアリングは追従誤差に対してあまり反応せず、安定した走行になりますが、追従精度が低下する可能性があります。

手順は次のとおりです。

1. `weight_lat_error` = 0.1、`weight_steering_input` = 1.0、その他の重み = 0に設定します。
2. 走行時に車両がオシレーションする場合は、`weight_steering_input`を大きくします。
3. 追従精度が低い場合は、`weight_steering_input`を小さくします。

高速レンジでのみ効果を調整したい場合は、`weight_steering_input_squared_vel`を使用できます。このパラメータは、高速レンジでのステアリング重みに対応します。

#### 重みについての説明

- `weight_lat_error`：横方向追従誤差を減らします。これはPIDのPゲインのように動作します。
- `weight_heading_error`：直進します。これはPIDのDゲインのように動作します。
- `weight_heading_error_squared_vel_coeff`：高速レンジで直進します。
- `weight_steering_input`：追従のオシレーションを減らします。
- `weight_steering_input_squared_vel_coeff`：高速レンジでの追従のオシレーションを減らします。
- `weight_lat_jerk`：横方向ジャークを減らします。
- `weight_terminal_lat_error`：安定性のために通常の横方向の重み`weight_lat_error`よりも高い値を設定することを推奨します。
- `weight_terminal_heading_error`：安定性のために通常のヘディングの重み`weight_heading_error`よりも高い値を設定することを推奨します。

#### その他のチューニングのヒント

他のパラメータを調整するためのヒントを以下に示します。

- 理論的には、ターミナル重み`weight_terminal_lat_error`と`weight_terminal_heading_error`を大きくすると、追従安定性が向上します。この手法は効果的な場合があります。
- `prediction_horizon`を大きくし、`prediction_sampling_time`を小さくすると、追従性能が向上します。ただし、計算コストが高くなります。
- trajectoryの曲率に応じて重みを変えたい場合（たとえば、急曲線で走行していて大きな重みが必要な場合）、`mpc_low_curvature_thresh_curvature`を使用して`mpc_low_curvature_weight_**`重みを調整します。
- 車両速度と軌跡曲率に基づいてステアリングレート制限を調整したい場合は、`steer_rate_lim_dps_list_by_curvature`、`curvature_list_for_steer_rate_lim`、`steer_rate_lim_dps_list_by_velocity`、`velocity_list_for_steer_rate_lim`の値を変更できます。これにより、高速走行中はステアリングレート制限を強制したり、曲線中は緩めたりできます。
- ターゲット曲率がギザギザに見える場合は、`curvature_smoothing`を調整することが、曲率の正確な計算に非常に重要になります。値が大きいほど曲率の計算が滑らかになり、ノイズが減りますが、フィードフォワード計算の遅延が発生し、性能が低下する可能性があります。
- `steering_lpf_cutoff_hz`の値を調整することも、計算ノイズを強制的に減らすのに効果的です。これは最後のレイヤーにインストールされた2次バターワースフィルターのカットオフ周波数を指します。カットオフ周波数が低いほどノイズの低減が大きくなりますが、動作に遅延が発生します。
- 車両が軌道から横方向に一貫して逸脱する場合は、ほとんどの場合、ステアリングセンサーのオフセットまたは自車位置推定のオフセットが原因です。これらバイアスをMPCに入力する前に排除することを推奨しますが、MPC内でこのバイアスを削除することもできます。これを使用するには、`enable_auto_steering_offset_removal`をtrueに設定してステアリングオフセット除去器を有効にします。ステアリングオフセット推定ロジックは、ステアリングが中心付近で高速走行しているときに機能し、オフセット除去を適用します。
- 曲線でのステアリングの開始が遅れる場合は、ステアリングモデルの遅延時間と時定数が正しくないことがよくあります。`input_delay`と`vehicle_model_steer_tau`の値を再確認してください。さらに、MPCはデバッグ情報の一部としてMPCモデルによって想定される現在のステアリング角を出力するため、そのステアリング角が実際のステアリング角と一致しているかどうかを確認してください。

## 参照/外部リンク

<!-- オプション -->

- [1] Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking",
  Robotics Institute, Carnegie Mellon University, February 2009.

## 自動運転ソフトウェアに関するドキュメント

### Planningコンポーネントの概要

Planningコンポーネントは、自動運転システムの主要な機能の1つであり、システムが安全で効率的な経路を計画するために必要です。

### Planningコンポーネントの動作

Planningコンポーネントは、以下のタスクを実行します。

- 自車位置の Estimation
- 環境の検出とマッピング
- 感知された障害物の追跡
- 安全で快適な経路の生成
- 各制御器への経路の伝送

### Planningコンポーネントの構成

Autoware Planningコンポーネントは、以下のようなモジュールで構成されています。

- **WaypointGenerator:** Waypointを生成し、経路を形成します。
- **TrajectoryGenerator:** Waypointに基づいて、滑らかな経路を生成します。
- **MotionPredictor:** 障害物の将来の動きを予測します。
- **CollisionChecker:** 生成された経路が環境の障害物と衝突しないことを確認します。

### Planningコンポーネントの入力

Planningコンポーネントは、以下の情報を入力として受け取ります。

- 自車位置の Estimation
- 感知された障害物のリスト
- 環境マップ
- 車両の動力学モデル

### Planningコンポーネントの出力

Planningコンポーネントは、以下の情報を出力として生成します。

- **Target Path:** 自動運転車が追従する経路
- **Target Velocity:** 経路上の各点での希望速度
- **Target Acceleration:** 経路上の各点での希望加速度

### Planningコンポーネントの性能要件

Planningコンポーネントは、以下のような性能要件を満たす必要があります。

- **安全性:** Planningコンポーネントは、車両が安全に操作されるように経路を計画する必要があります。
- **効率性:** Planningコンポーネントは、計算効率が高く、リアルタイムで動作する必要があります。
- **robustness:** Planningコンポーネントは、不完全な情報や予測不可能なイベントに対しても堅牢である必要があります。

### Planningコンポーネントの検証

Planningコンポーネントは、シミュレーションと実車テストの両方によって検証する必要があります。検証には以下が含まれます。

- **path planning**の正確性
- **Trajectory planning**の滑らかさと効率性
- **Collision avoidance**の有効性
