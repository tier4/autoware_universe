# autoware_gyro_odometer

## 目的

`autoware_gyro_odometer` は IMU と車速を組み合わせて、ひねり速度を推定するためのパッケージです。

## 入力 / 出力

### 入力

- `/vehicle/odometry/twist` (geometry_msgs/Twist): ロボットのひねり速度
- `/imu/<sensor_frame_id>/data_raw` (sensor_msgs/Imu): IMU データ
- `/tf` (tf2/TransformStamped): ロボットの TF トランスフォーム
- `/vehicle/wheel_cmd` (vehicle_msgs/WheelCmd): 車輪速度

### 出力

- `/vehicle/twist` (geometry_msgs/Twist): 補償されたひねり速度

| 名称                            | 型                                               | 説明                     |
| ------------------------------- | ------------------------------------------------ | ------------------------ |
| `vehicle/twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | 車両の共分散付きツイスト |
| `imu`                           | `sensor_msgs::msg::Imu`                          | センサーの IMU           |

### アウトプット

車両のダイナミクス モデルは、エンコーダーの `post resampling` 計測値のフィードバックと、Planning コンポーネンにより生成された目的速度、加速度を組み合わせ、自車位置と姿勢を推定します。ダイナミクス モデルは、非線形車輪モデルに基づく 2 自由度システムを想定しています。

エンコーダーの `post resampling` 計測値は、ホイール アライメント、回転径、およびアライメントの不均衡性を補正するために実装された、低パス フィルター付きの differentiation により、速度および加速度に変換されます。ダイナミクス モデルは、以下の式を使用して、自車位置と姿勢を計算します。

```
x = x0 + \int vcos(θ) dt
y = y0 + \int vsin(θ) dt
θ = θ0 + \int ω dt
```

ここで、

- `x` は x 軸上の自車位置
- `y` は y 軸上の自車位置
- `θ` は自車の偏角
- `v` は車両の速度
- `ω` は車両の角速度
- `x0`、`y0`、`θ0` は初期条件

車両の速度と加速度は、Planning コンポーネンによって生成されます。Planning コンポーネンは、周囲環境に関する情報をセンサから収集し、車両を目的地まで安全かつ効率的に移動させる目的経路を生成します。

ダイナミクス モデルは、以下の基準に基づいて、自車位置と姿勢の推定値を評価します。

- **速度逸脱量：** Planning コンポーネンによって指定された速度からの自車速度の逸脱量
- **加速度逸脱量：** Planning コンポーネンによって指定された加速度からの自車加速度の逸脱量
- **軌跡逸脱量：** Planning コンポーネンによって指定された軌跡からの自車位置の逸脱量

Autoware では、こうした基準を使用して、ダイナミクス モデルの推定値の精度が検証されます。

| 名称                    | 種別                                             | 説明               |
| ----------------------- | ------------------------------------------------ | ------------------ |
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | 共分散付き推測速度 |

## パラメータ

{{ json_to_markdown("localization/autoware_gyro_odometer/schema/gyro_odometer.schema.json") }}

## 前提条件/既知の制限

- [前提条件] 入力ツイストメッセージのフレーム ID は base_link に設定する必要があります。

- [前提条件] 入力メッセージの共分散は適切に割り当てられている必要があります。

- [前提条件] 縦方向の車両速度とヨー軸まわりの角速度の両方が十分に小さい場合は、角速度をゼロに設定します。これは、IMU 角速度バイアスの抑制のためのものです。この処理がないと、停止時に車両状態を誤って推定してしまいます。

- [制限事項] 出力メッセージの周波数は、入力 IMU メッセージの周波数に依存します。

- [制限事項] 横方向および鉛直方向の速度について信頼できる値を生成できません。そのため、出力共分散行列の対応する要素には大きな値が割り当てられます。

## 診断

<img src="./media/diagnostic.png" alt="drawing" width="600"/>

| 名称                             | 説明                                                           | 警告への遷移条件   | エラーへの遷移条件                           |
| -------------------------------- | -------------------------------------------------------------- | ------------------ | -------------------------------------------- |
| `topic_time_stamp`               | サービスを呼び出すときのタイムスタンプ [ナノ秒]                | なし               | なし                                         |
| `is_arrived_first_vehicle_twist` | 車両の Twist トピックを少なくとも 1 回受信したかどうかのフラグ | まだ受信していない | なし                                         |
| `is_arrived_first_imu`           | IMU トピックを少なくとも 1 回受信したかどうかのフラグ          | まだ受信していない | なし                                         |
| `vehicle_twist_time_stamp_dt`    | 現在時刻と最新の車両の Twist トピック間の時間差 [秒]           | なし               | 時間差が `message_timeout_sec` より **長い** |
| `imu_time_stamp_dt`              | 現在時刻と最新の IMU トピック間の時間差 [秒]                   | なし               | 時間差が `message_timeout_sec` より **長い** |
| `vehicle_twist_queue_size`       | `vehicle_twist_queue` のサイズ                                 | なし               | なし                                         |
| `imu_queue_size`                 | `gyro_queue` のサイズ                                          | なし               | なし                                         |
| `is_succeed_transform_imu`       | IMU の変換の成功/失敗フラグ                                    | なし               | 変換が失敗した                               |
