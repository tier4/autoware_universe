# Autoware Joy Controller

## 役割

`autoware_joy_controller` は、ジョイ メッセージを Autoware コマンド (例: ステアリング ホイール、シフト、ターン シグナル、エンゲージ) に変換するためのパッケージです。

## 使用方法

### ROS 2 起動


```bash
# With default config (ds4)
ros2 launch autoware_joy_controller joy_controller.launch.xml

# Default config but select from the existing parameter files
ros2 launch autoware_joy_controller joy_controller_param_selection.launch.xml joy_type:=ds4 # or g29, p65, xbox

# Override the param file
ros2 launch autoware_joy_controller joy_controller.launch.xml config_file:=/path/to/your/param.yaml
```

## 入力 / 出力

### 入力トピック

| 名称               | タイプ                    | 説明                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | ジョイスティックコントローラコマンド            |
| `~/input/odometry` | nav_msgs::msg::Odometry | 自車位置を取得するための自車オドメトリ |

### 出力トピック

| 名称                                | タイプ                                                | 説明                              |
| ----------------------------------- | --------------------------------------------------- | ---------------------------------------- |
| `~/output/control_command`          | autoware_control_msgs::msg::Control                 | 横方向および縦方向制御コマンド |
| `~/output/external_control_command` | tier4_external_api_msgs::msg::ControlCommandStamped | 横方向および縦方向制御コマンド |
| `~/output/shift`                    | tier4_external_api_msgs::msg::GearShiftStamped      | ギアシフトコマンド                     |
| `~/output/turn_signal`              | tier4_external_api_msgs::msg::TurnSignalStamped     | ターンシグナルコマンド                  |
| `~/output/gate_mode`                | tier4_control_msgs::msg::GateMode                   | ゲートモード（自動または外部）             |
| `~/output/heartbeat`                | tier4_external_api_msgs::msg::Heartbeat             | ハートビート                                |
| `~/output/vehicle_engage`           | autoware_vehicle_msgs::msg::Engage                  | 車両動作                           |

## パラメータ

| パラメータ                 | タイプ   | 説明                                                                                                       |
| ------------------------- | ------ | --------------------------------------------------------------------------------------------------------------- |
| `joy_type`                | 文字列 | ジョイコントローラのタイプ（既定値：DS4）                                                                     |
| `update_rate`             | double | 制御コマンドを発行するための更新率                                                                             |
| `accel_ratio`             | double | 加速度を計算するための比率（指示された加速度は比率 \* 操作）                                                 |
| `brake_ratio`             | double | 減速を計算するための比率（指示された加速度は -比率 \* 操作）                                                  |
| `steer_ratio`             | double | 減速を計算するための比率（指示された操舵は比率 \* 操作）                                                    |
| `steering_angle_velocity` | double | 操作のための操舵角速度                                                                                     |
| `accel_sensitivity`       | double | 外部APIの加速度を計算するための感度（指示された加速度は pow(操作, 1 / 感度)）                               |
| `brake_sensitivity`       | double | 外部APIの減速を計算するための感度（指示された加速度は pow(操作, 1 / 感度)）                               |
| `raw_control`             | ブール型 | true の場合、入力オドメトリをスキップ                                                                            |
| `velocity_gain`           | double | 加速度による速度を計算するための比率                                                                         |
| `max_forward_velocity`    | double | 前進するための絶対最大速度                                                                                   |
| `max_backward_velocity`   | double | 後退するための絶対最大速度                                                                                   |
| `backward_accel_ratio`    | double | 減速を計算するための比率（指示された加速度は -比率 \* 操作）                                                  |

## P65 ジョイスティック キーマップ

| 操作 | ボタン |
|---|---|
| 加速 | R2 |
| ブレーキ | L2 |
| ステアリング | 左スティック左右 |
| シフトアップ | カーソルアップ |
| シフトダウン | カーソルダウン |
| ドライブシフト | カーソル左 |
| リバースシフト | カーソル右 |
| 左ウィンカー | L1 |
| 右ウィンカー | R1 |
| ウィンカーキャンセル | A |
| ゲートモード | B |
| 緊急停止 | セレクト |
| 緊急停止解除 | スタート |
| Autoware エンゲージ | X |
| Autoware ディスエンゲージ | Y |
| Vehicle エンゲージ | PS |
| Vehicle ディスエンゲージ | 右トリガー |

## DS4 ジョイスティック キーマップ

| 操作               | ボタン                     |
| -------------------- | -------------------------- |
| 加速                 | R2、×、または右スティック上 |
| ブレーキ              | L2、□、または右スティック下 |
| ステアリング          | 左スティック左右          |
| シフトアップ          | カーソル上                  |
| シフトダウン          | カーソル下                  |
| ドライブシフト        | カーソル左                  |
| リバースシフト        | カーソル右                  |
| 左ウインカー          | L1                         |
| 右ウインカー          | R1                         |
| ウインカー解除         | SHARE                      |
| ゲートモード          | OPTIONS                    |
| 緊急停止             | PS                         |
| 緊急停止解除         | PS                         |
| Autowareエンゲージ     | ○                          |
| Autowareディスエンゲージ | ○                          |
| Vehicleエンゲージ     | △                          |
| Vehicleディスエンゲージ | △                          |

## XBOX ジョイスティック キー マップ

| 操作 | ボタン |
|---|---|
| 加速 | RT |
| ブレーキ | LT |
| ステアリング | 左スティック左右 |
| シフトアップ | カーソル上 |
| シフトダウン | カーソル下 |
| ドライブシフト | カーソル左 |
| リバースシフト | カーソル右 |
| 左ウィンカー | LB |
| 右ウィンカー | RB |
| ウィンカー解除 | A |
| ゲートモード | B |
| 緊急停止 | View |
| 緊急停止解除 | Menu |
| Autoware エンゲージ | X |
| Autoware ディスエンゲージ | Y |
| 車両エンゲージ | 左スティックボタン |
| 車両ディスエンゲージ | 右スティックボタン |

