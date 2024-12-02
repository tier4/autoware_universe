# simple_planning_simulator

## 目的 / ユースケース

このノードは、シンプルな車両モデルを使用して、2Dの車両コマンドの車両運動をシミュレートします。

## 設計

このシミュレータの目的は、Planningモジュールと制御モジュールの統合テスト用です。これはセンシングや知覚はシミュレートせず、純粋な C++ のみで実装され、GPU なしでも動作します。

## 想定 / 制限事項

- 2D の運動のみをシミュレートします。
- 衝突やセンシングなどの物理操作は実行せず、車両ダイナミクスの積分結果のみを計算します。

## 入出力 / API

### 入力

- input/initialpose [`geometry_msgs/msg/PoseWithCovarianceStamped`] : 初期ポーズ用
- input/ackermann_control_command [`autoware_control_msgs/msg/Control`] : 車両を駆動するターゲットコマンド
- input/manual_ackermann_control_command [`autoware_control_msgs/msg/Control`] : 車両を駆動する手動ターゲットコマンド (control_mode_request = 手動の場合に使用)
- input/gear_command [`autoware_vehicle_msgs/msg/GearCommand`] : ターゲットギアコマンド
- input/manual_gear_command [`autoware_vehicle_msgs/msg/GearCommand`] : ターゲットギアコマンド (control_mode_request = 手動の場合に使用)
- input/turn_indicators_command [`autoware_vehicle_msgs/msg/TurnIndicatorsCommand`] : ターゲットターンインジケータコマンド
- input/hazard_lights_command [`autoware_vehicle_msgs/msg/HazardLightsCommand`] : ターゲットハザードライトコマンド
- input/control_mode_request [`tier4_vehicle_msgs::srv::ControlModeRequest`] : 自動/手動運転のモード変更

### 出力

- /tf [`tf2_msgs/msg/TFMessage`] : 車両のシミュレートされたポーズ (base_link)
- /output/odometry [`nav_msgs/msg/Odometry`] : シミュレートされた車両のポーズとツイスト
- /output/steering [`autoware_vehicle_msgs/msg/SteeringReport`] : 車両のシミュレートされたステアリング角
- /output/control_mode_report [`autoware_vehicle_msgs/msg/ControlModeReport`] : 現在の制御モード (自動/手動)
- /output/gear_report [`autoware_vehicle_msgs/msg/ControlModeReport`] : 車両のシミュレートされたギア
- /output/turn_indicators_report [`autoware_vehicle_msgs/msg/ControlModeReport`] : 車両のシミュレートされたターンインジケータステータス
- /output/hazard_lights_report [`autoware_vehicle_msgs/msg/ControlModeReport`] : 車両のシミュレートされたハザードライトステータス

## 動作 / アルゴリズム

### 共通パラメータ

| 名前                    | タイプ | 説明                                                                                                                                  | デフォルト値         |
| :--------------------- | :----- | :-------------------------------------------------------------------------------------------------------------------------------------- | :------------------- |
| simulated_frame_id    | 文字列 | 出力の `tf` の `child_frame_id` に設定                                                                                              | "base_link"          |
| origin_frame_id       | 文字列 | 出力の tf の `frame_id` に設定                                                                                                         | "odom"               |
| initialize_source     | 文字列 | "ORIGIN" の場合、初期姿勢は (0,0,0) に設定されます。 "INITIAL_POSE_TOPIC" の場合、ノードは `input/initialpose` トピックが発行されるまで待機します。 | "INITIAL_POSE_TOPIC" |
| add_measurement_noise | ブール | True の場合、シミュレーション結果にガウスノイズが追加されます。                                                                         | true                 |
| pos_noise_stddev      | double | 位置ノイズの標準偏差                                                                                                                      | 0.01                 |
| rpy_noise_stddev      | double | オイラー角ノイズの標準偏差                                                                                                                | 0.0001               |
| vel_noise_stddev      | double | 速度ノイズの標準偏差                                                                                                                      | 0.0                  |
| angvel_noise_stddev   | double | 角速度ノイズの標準偏差                                                                                                                      | 0.0                  |
| steer_noise_stddev    | double | ステアリング角のノイズの標準偏差                                                                                                          | 0.0001               |

### 車両モデルパラメータ

#### vehicle_model_type のオプション

- `IDEAL_STEER_VEL`
- `IDEAL_STEER_ACC`
- `IDEAL_STEER_ACC_GEARED`
- `DELAY_STEER_VEL`
- `DELAY_STEER_ACC`
- `DELAY_STEER_ACC_GEARED`
- `DELAY_STEER_ACC_GEARED_WO_FALL_GUARD`
- `DELAY_STEER_MAP_ACC_GEARED`: 1D ダイナミクスおよびタイム遅延をステアリングおよびアクセルコマンドに適用します。シミュレートされたアクセルは、提供されたアクセルマップで変換された値により決定されます。このモデルは、実際の車両のアクセルのずれによる正確なシミュレーションに役立ちます。
- `LEARNED_STEER_VEL`: 学習済み Python モデルを起動します。詳細はこちら [こちら](../learning_based_vehicle_model) を参照してください。

次のモデルは `raw_vehicle_cmd_converter` から生成された `ACTUATION_CMD` を受信します。したがって、これらのモデルが選択されると `raw_vehicle_cmd_converter` も起動されます。

- `ACTUATION_CMD`: このモデルはアクセル/ブレーキコマンドのみを変換し、ステアリングコマンドはそのまま使用します。
- `ACTUATION_CMD_STEER_MAP`: ステアリングマップを使用して、ステアリングコマンドをステアリングレートコマンドに変換します。
- `ACTUATION_CMD_VGR`: 可変ギア比を使用して、ステアリングコマンドをステアリングレートコマンドに変換します。
- `ACTUATION_CMD_MECHANICAL`: このモデルには、それらの機械的ダイナミクスとコントローラがあります。

`IDEAL` モデルはコマンドどおりに理想的に移動し、`DELAY` モデルは 1 次のタイム遅延モデルに基づいて移動します。`STEER` は、モデルがステアリングコマンドを受信することを意味します。`VEL` は、モデルがターゲット速度コマンドを受信することを意味し、`ACC` モデルはターゲットアクセルコマンドを受信することを意味します。接尾辞 `GEARED` は、モーションがギアコマンドを考慮することを意味します。車両はギアに従って 1 方向にのみ移動します。

以下の表は、どのモデルがどのパラメータに対応するかを示しています。モデル名は省略形で記載されています（例: IDEAL_STEER_VEL = I_ST_V）。

| 名 | タイプ | 説明 | I_ST_V | I_ST_A | I_ST_A_G | D_ST_V | D_ST_A | D_ST_A_G | D_ST_A_G_WO_FG | D_ST_M_ACC_G | L_S_V | A_C | デフォルト値 | 単位 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| acc_time_delay | double | 加速度入力をのデッドタイム | x | x | x | x | o | o | o | o | x | x | 0.1 | [s] |
| steer_time_delay | double | 操舵入力をのデッドタイム | x | x | x | o | o | o | o | o | x | o | 0.24 | [s] |
| vel_time_delay | double | 速度入力をのデッドタイム | x | x | x | o | x | x | x | x | x | x | 0.25 | [s] |
| acc_time_constant | double | 1次加速度のダイナミクスの時定数 | x | x | x | x | o | o | o | o | x | x | 0.1 | [s] |
| steer_time_constant | double | 1次操舵のダイナミクスの時定数 | x | x | x | o | o | o | o | o | x | o | 0.27 | [s] |
| steer_dead_band | double | 操舵角度のデッドバンド | x | x | x | o | o | o | o | x | x | o | 0.0 | [rad] |
| vel_time_constant | double | 1次速度のダイナミクスの時定数 | x | x | x | o | x | x | x | x | x | x | 0.5 | [s] |
| vel_lim | double | 速度の制限 | x | x | x | o | o | o | o | o | x | x | 50.0 | [m/s] |
| vel_rate_lim | double | 加速度の制限 | x | x | x | o | o | o | o | o | x | x | 7.0 | [m/ss] |
| steer_lim | double | 操舵角の制限 | x | x | x | o | o | o | o | o | x | o | 1.0 | [rad] |
| steer_rate_lim | double | 操舵角変化率の制限 | x | x | x | o | o | o | o | o | x | o | 5.0 | [rad/s] |
| steer_bias | double | 操舵角のバイアス | x | x | x | o | o | o | o | o | x | o | 0.0 | [rad] |
| debug_acc_scaling_factor | double | 加速度コマンドのスケーリング係数 | x | x | x | x | o | o | o | o | x | x | 1.0 | [-] |
| debug_steer_scaling_factor | double | 操舵コマンドのスケーリング係数 | x | x | x | x | o | o | o | o | x | x | 1.0 | [-] |
| acceleration_map_path | string | 速度と理想加速度を実際の加速度に変換するための、加速マップのCSVファイルへのパス | x | x | x | x | x | x | x | o | x | x | - | [-] |
| model_module_paths | string | モデルが実装されているpythonモジュールへのパス | x | x | x | x | x | x | x | x | o | x | - | [-] |
| model_param_paths | string | モデルのパラメータが格納されているファイルへのパス（パラメータファイルが必要ない場合は空文字列にすることができます） | x | x | x | x | x | x | x | x | o | x | - | [-] |
| model_class_names | string | モデルを実装するクラスの名前 | x | x | x | x | x | x | x | x | o | x | - | [-] |

_注:_ パラメータ `model_module_paths`, `model_param_paths`, `model_class_names` は同じ長さにする必要があります。

`acceleration_map` は `DELAY_STEER_MAP_ACC_GEARED` の場合のみ使用され、垂直軸にアクセルコマンド、水平軸に現在の速度を表示し、各セルがシミュレータの運動計算で実際に使用される変換されたアクセルコマンドを表しています。中間値は線形補間されます。

`acceleration_map.csv` の例:


```csv
default,  0.00,  1.39,  2.78,  4.17,  5.56,  6.94,  8.33,  9.72, 11.11, 12.50, 13.89, 15.28, 16.67
-4.0,    -4.40, -4.36, -4.38, -4.12, -4.20, -3.94, -3.98, -3.80, -3.77, -3.76, -3.59, -3.50, -3.40
-3.5,    -4.00, -3.91, -3.85, -3.64, -3.68, -3.55, -3.42, -3.24, -3.25, -3.00, -3.04, -2.93, -2.80
-3.0,    -3.40, -3.37, -3.33, -3.00, -3.00, -2.90, -2.88, -2.65, -2.43, -2.44, -2.43, -2.39, -2.30
-2.5,    -2.80, -2.72, -2.72, -2.62, -2.41, -2.43, -2.26, -2.18, -2.11, -2.03, -1.96, -1.91, -1.85
-2.0,    -2.30, -2.24, -2.12, -2.02, -1.92, -1.81, -1.67, -1.58, -1.51, -1.49, -1.40, -1.35, -1.30
-1.5,    -1.70, -1.61, -1.47, -1.46, -1.40, -1.37, -1.29, -1.24, -1.10, -0.99, -0.83, -0.80, -0.78
-1.0,    -1.30, -1.28, -1.10, -1.09, -1.04, -1.02, -0.98, -0.89, -0.82, -0.61, -0.52, -0.54, -0.56
-0.8,    -0.96, -0.90, -0.82, -0.74, -0.70, -0.65, -0.63, -0.59, -0.55, -0.44, -0.39, -0.39, -0.35
-0.6,    -0.77, -0.71, -0.67, -0.65, -0.58, -0.52, -0.51, -0.50, -0.40, -0.33, -0.30, -0.31, -0.30
-0.4,    -0.45, -0.40, -0.45, -0.44, -0.38, -0.35, -0.31, -0.30, -0.26, -0.30, -0.29, -0.31, -0.25
-0.2,    -0.24, -0.24, -0.25, -0.22, -0.23, -0.25, -0.27, -0.29, -0.24, -0.22, -0.17, -0.18, -0.12
 0.0,     0.00,  0.00, -0.05, -0.05, -0.05, -0.05, -0.08, -0.08, -0.08, -0.08, -0.10, -0.10, -0.10
 0.2,     0.16,  0.12,  0.02,  0.02,  0.00,  0.00, -0.05, -0.05, -0.05, -0.05, -0.08, -0.08, -0.08
 0.4,     0.38,  0.30,  0.22,  0.25,  0.24,  0.23,  0.20,  0.16,  0.16,  0.14,  0.10,  0.05,  0.05
 0.6,     0.52,  0.52,  0.51,  0.49,  0.43,  0.40,  0.35,  0.33,  0.33,  0.33,  0.32,  0.34,  0.34
 0.8,     0.82,  0.81,  0.78,  0.68,  0.63,  0.56,  0.53,  0.48,  0.43,  0.41,  0.37,  0.38,  0.40
 1.0,     1.00,  1.08,  1.01,  0.88,  0.76,  0.69,  0.66,  0.58,  0.54,  0.49,  0.45,  0.40,  0.40
 1.5,     1.52,  1.50,  1.38,  1.26,  1.14,  1.03,  0.91,  0.82,  0.67,  0.61,  0.51,  0.41,  0.41
 2.0,     1.80,  1.80,  1.64,  1.43,  1.25,  1.11,  0.96,  0.81,  0.70,  0.59,  0.51,  0.42,  0.42
```

![acceleration_map](./media/acceleration_map.svg)

##### ACTUATION_CMD モデル

通常 simple_planning_simulator は制御コマンドを受信することにより動作しますが、`ACTUATION_CMD*` モデルが選択されると、制御コマンドの代わりに作動コマンドを受信します。このモデルは、実際の車両に送信される車両コマンドを使用して動作をシミュレートできます。したがって、このモデルが選択されると `raw_vehicle_cmd_converter` も起動されます。詳細については、[actuation_cmd_sim.md](./docs/actuation_cmd_sim.md) を参照してください。

<!-- deadzone_delta_steer | double | dead zone for the steering dynamics                  | x      | x      | x        | o      | o      | 0.0      | [rad]         |         | -->

_注意_: ステアリング/速度/加速度のダイナミクスは、_遅延_ モデルのデッドタイムを持つ1階システムによってモデリングされます。_時定数_の定義は、ステップ応答が最終値の63％まで上昇するまでの時間です。_デッドタイム_ は制御入力に対する応答の遅れです。

### LEARNED_STEER_VEL モデルの例

`LEARNED_STEER_VEL` の仕組みを示すために、いくつかの基本モデルを作成しました。

1. 基本的な Python モデルを含む [ライブラリ](https://github.com/atomyks/control_analysis_pipeline/tree/v0.1_autoware) をインストールします (ブランチ: `v0.1_autoware`)。

2. `src/vehicle/sample_vehicle_launch/sample_vehicle_description/config/simulator_model.param.yaml` ファイルで `vehicle_model_type` を `LEARNED_STEER_VEL` に設定します。同じファイルで、次のパラメータを設定します。これらのモデルはテスト用であり、パラメータファイルは必要ありません。


```yaml
model_module_paths:
  [
    "control_analysis_pipeline.autoware_models.vehicle.kinematic",
    "control_analysis_pipeline.autoware_models.steering.steer_example",
    "control_analysis_pipeline.autoware_models.drive.drive_example",
  ]
model_param_paths: ["", "", ""]
model_class_names: ["KinematicModel", "SteerExample", "DriveExample"]
```

### デフォルト TF 設定

車両が `odom` -> `base_link` tf を出力するため、このシミュレータは同じフレーム ID 構成で tf を出力します。
simple_planning_simulator.launch.py では、通常はローカライゼーションモジュール (例: NDT) によって推定される `map` -> `odom` tf を出力するノードも起動されます。このシミュレータモジュールが返す tf は理想的な値であるため、`odom` -> `map` は常に 0 になります。

### (注意点) ピッチ計算

自車ピッチ角は次のように計算されます。

![ピッチ計算](./media/pitch-calculation.drawio.svg)

注: 画像の下段に示すように、線の向きに対して走行することはサポートされておらず、説明のためだけに示されています。

## エラー検出と処理

実行される唯一の入力検証は、有効な車両モデルタイプのテストです。

## セキュリティの考慮事項

**必須**

<!-- 考慮事項:
- スプーフィング (偽の入力をどのようにチェックして処理しますか?)
- 改ざん (改ざんされた入力をどのようにチェックして処理しますか?)
- 否認 (外部の動作者の行為の影響を受けますか?)
- 情報漏洩 (データが漏洩する可能性がありますか?)
- サービス拒否 (スパムをどのように処理しますか?)
- 権限昇格 (実行中に権限レベルを変更する必要がありますか?) -->

## 参照 / 外部リンク

これはもともと Autoware.AI で開発されました。以下のリンクを参照してください。

<https://github.com/Autoware-AI/simulation/tree/master/wf_simulator>

## 今後の拡張 / 実装されていない部分

- 車両モデルの精度の向上 (例: ステアリングのデッドゾーンとスリップ動作の追加)
- 疑似ポイント クラウドまたは疑似認識結果を出力するモジュールとの連携

