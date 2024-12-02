# tier4_vehicle_rviz_plugin

このパッケージには jsk コードが含まれています。
jsk_overlay_utils.cpp と jsk_overlay_utils.hpp は BSD ライセンスであることにご注意ください。

## 目的

このプラグインは、車両速度、方向指示器、ステアリング状態、加速度を視覚的かつわかりやすく表示します。

## 入出力

### 入力

| 名前                              | タイプ                                               | 説明                        |
| --------------------------------- | -------------------------------------------------- | ---------------------------------- |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs::msg::VelocityReport`       | 車両の速度                  |
| `/control/turn_signal_cmd`        | `autoware_vehicle_msgs::msg::TurnIndicatorsReport` | ウインカーのステータス             |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs::msg::SteeringReport`       | ステアリングの状態                 |
| `/localization/acceleration`      | `geometry_msgs::msg::AccelWithCovarianceStamped`   | 加速度                        |

## パラメータ

### コアパラメータ

#### ConsoleMeter

| 名前                                | 型   | デフォルト値           | 説明                                        |
| ------------------------------------ | ------ | ----------------------- | ------------------------------------------- |
| `property_text_color_`              | QColor | QColor(25, 255, 240)    | テキストの色                                |
| `property_left_`                    | int    | 128                     | プロッターウィンドウの左 [px]               |
| `property_top_`                     | int    | 128                     | プロッターウィンドウの上 [px]               |
| `property_length_`                  | int    | 256                     | プロッターウィンドウの高さ [px]             |
| `property_value_height_offset_`     | int    | 0                       | プロッターウィンドウの高さオフセット [px]    |
| `property_value_scale_`             | float  | 1.0 / 6.667             | 値のスケール                                 |

#### SteeringAngle

| 名前                            | タイプ | デフォルト値         | 説明                                       |
| ------------------------------- | ------ | --------------------- | -------------------------------------------- |
| `property_text_color_`          | QColor | QColor(25, 255, 240) | テキストの色                               |
| `property_left_`                | int    | 128                   | プロットウィンドウの左 [px]                 |
| `property_top_`                 | int    | 128                   | プロットウィンドウの上 [px]                  |
| `property_length_`              | int    | 256                   | プロットウィンドウの高さ [px]               |
| `property_value_height_offset_` | int    | 0                     | プロットウィンドウの高さオフセット [px]       |
| `property_value_scale_`         | float  | 1.0 / 6.667           | 値のスケール                               |
| `property_handle_angle_scale_`  | float  | 3.0                   | スケールは操舵角からハンドルの角度         |

#### TurnSignal

| 名前              | タイプ | 初期値 | 説明                                       |
| ----------------- | ---- | -------- | ------------------------------------------- |
| `property_left_`    | int  | 128      | プロットウィンドウの左端 [px]             |
| `property_top_`     | int  | 128      | プロットウィンドウの上端 [px]            |
| `property_width_`   | int  | 256      | プロットウィンドウの幅 [px]               |
| `property_height_`  | int  | 256      | プロットウィンドウの高さ [px]             |

#### VelocityHistory

| Name                            | Type   | Default Value | Description                |
| ------------------------------- | ------ | ------------- | -------------------------- |
| `property_velocity_timeout_`    | float  | 10.0          | 速度のタイムアウト [s]    |
| `property_velocity_alpha_`      | float  | 1.0           | 速度のアルファ          |
| `property_velocity_scale_`      | float  | 0.3           | 速度のスケール          |
| `property_velocity_color_view_` | bool   | false         | Constant Colorかどうかの使用  |
| `property_velocity_color_`      | QColor | Qt::black     | 速度の履歴の色  |
| `property_vel_max_`             | float  | 3.0           | 色の境界速度の最大値 [m/s] |

#### 加速度計

| Name                                | Type   | Default Value        | Description                                      |
| ----------------------------------- | ------ | -------------------- | ------------------------------------------------ |
| `property_normal_text_color_`       | QColor | QColor(25, 255, 240) | 通常テキストの色                                |
| `property_emergency_text_color_`    | QColor | QColor(255, 80, 80)  | 緊急時加速度の色                                |
| `property_left_`                    | int    | 896                  | プロットウィンドウの左 [px]                  |
| `property_top_`                     | int    | 128                  | プロットウィンドウの上部 [px]                   |
| `property_length_`                  | int    | 256                  | プロットウィンドウの高さ [px]                |
| `property_value_height_offset_`     | int    | 0                    | プロットウィンドウの高さオフセット [px]         |
| `property_value_scale_`             | float  | 1 / 6.667            | 値テキストのスケール                                 |
| `property_emergency_threshold_max_` | float  | 1.0                  | 緊急時の最大加速度しきい値 [m/s^2] |
| `property_emergency_threshold_min_` | float  | -2.5                 | 緊急時の最小加速度しきい値 [m/s^2] |

## 仮定 / 制限事項

未定。

## 使用法

1. RVIZ を起動し、[Displays] パネルの下の [追加] を選択します。
   ![select_add](./images/select_add.png)
2. **tier4_vehicle_rviz_plugin** のいずれかを選択して、[OK] を押します。
   ![select_vehicle_plugin](./images/select_vehicle_plugin.png)
3. ステータスを表示するトピックの名前を入力します。
   ![select_topic_name](./images/select_topic_name.png)

