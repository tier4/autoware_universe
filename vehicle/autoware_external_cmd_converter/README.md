# external_cmd_converter

`external_cmd_converter` はアクセル/ブレーキマップを使用することで目的の機械的入力を加速度と速度に変換するノードです。

## アルゴリズム

### 目的の加速度と速度の計算方法

目的の加速度と速度は外制御コマンドのアクセルとブレーキ値から導出されます。

#### 目的の加速度

目的の加速度は、desired_pedal の値と現在の速度に基づく accel_brake_map から計算されます。

$$
    pedal_d = throttle_d - brake_d,
$$

$$
    acc_{ref} = Acc(pedal_d, v_{x,current}).
$$

| パラメーター | 説明 |
|---|---|
| `$throttle_d$` | 外部制御コマンドのアクセル値 (`~/in/external_control_cmd.control.throttle`) |
| `$brake_d$` | 外部制御コマンドのブレーキ値 (`~/in/external_control_cmd.control.brake`) |
| `$v_{x,current}$` | 現在の縦速度 (`~/in/odometry.twist.twist.linear.x`) |
| Acc | acel_brake_map |

#### 参照速度

参照速度は現在の速度と参照加速度に基づいて計算されます。

$$
v_{ref} =
    v_{x,current} + k_{v_{ref}} \cdot \text{sign}_{gear} \cdot acc_{ref}.
$$

| パラメータ           | 説明                                                           |
| -------------------- | --------------------------------------------------------------------- |
| $acc_{ref}$          | 参照加速度                                                |
| $k_{v_{ref}}$        | 参照速度ゲイン                                               |
| $\text{sign}_{gear}$ | ギアコマンド (`~/in/shift_cmd`) (ドライブ/ロー: 1、リバース: -1、その他: 0) |

## 入力トピック

| 名前                          | 種類                                         | 説明                                                                                                         |
| ---------------------------- | -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `~/in/external_control_cmd` | `tier4_external_api_msgs::msg::ControlCommand` | target `throttle/brake/steering_angle/steering_angle_velocity`は目的の制御コマンドの計算に必要な制御コマンドです。 |
| `~/input/shift_cmd"`         | `autoware_vehicle_msgs::GearCommand`          | ギアの現在の指令                                                                                               |
| `~/input/emergency_stop`   | `tier4_external_api_msgs::msg::Heartbeat`     | 外部コマンド用の非常心拍数                                                                                        |
| `~/input/current_gate_mode` | `tier4_control_msgs::msg::GateMode`            | ゲートモードのトピック                                                                                              |
| `~/input/odometry`           | `navigation_msgs::Odometry`                     | オドメトリのツイストトピックを使用                                                                                  |

## 出力トピック

| 名称                | タイプ                                | 説明                                                        |
| ------------------- | ----------------------------------- | ------------------------------------------------------------------ |
| `~/out/control_cmd` | autoware_control_msgs::msg::Control | 選択された外部コマンドから変換されたアッカーマン制御コマンド |

## パラメータ

| パラメータ                 | 型   | 説明                                                     |
| ------------------------- | ------ | ----------------------------------------------------------- |
| `ref_vel_gain_`           | double | 基準速度ゲイン                                         |
| `timer_rate`              | double | タイマーの更新レート                                   |
| `wait_for_first_topic`    | double | 最初のトピックを受信した後タイムアウトチェックを行うかどうか |
| `control_command_timeout` | double | ステアリングコマンドのタイムアウトチェック                |
| `emergency_stop_timeout`  | double | 緊急停止コマンドのタイムアウトチェック                  |

## 限界

未定義

