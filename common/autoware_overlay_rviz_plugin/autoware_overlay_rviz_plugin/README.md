## Autoware Overlay RVizプラグイン

RViz2の3Dシーンに2Dオーバーレイを表示するためのプラグインです。

[jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)
パッケージをベースにしています。ライセンスは3条項BSDライセンスです。

## 目的

このプラグインは、車両の速度、ウィンカー、ステアリングステータス、ギアを視覚的に分かりやすく表示します。

## 入出力

### 入力

| 名称                                                    | 型                                                    | 説明                                    |
| ------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------- |
| `/vehicle/status/velocity_status`                       | `autoware_vehicle_msgs::msg::VelocityReport`            | 車両速度のトピック                        |
| `/vehicle/status/turn_indicators_status`                | `autoware_vehicle_msgs::msg::TurnIndicatorsReport`      | ウインカーの状態のトピック                    |
| `/vehicle/status/hazard_status`                         | `autoware_vehicle_msgs::msg::HazardReport`              | ハザードの状態のトピック                        |
| `/vehicle/status/steering_status`                       | `autoware_vehicle_msgs::msg::SteeringReport`            | ステアリングの状態のトピック                    |
| `/vehicle/status/gear_status`                           | `autoware_vehicle_msgs::msg::GearReport`                | ギアのステータスに関するトピック                |
| `/planning/scenario_planning/current_max_velocity`      | `tier4_planning_msgs::msg::VelocityLimit`               | 速度制限に関するトピック                    |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 交通信号のステータスに関するトピック            |

## パラメータ

### コアパラメータ

#### SignalDisplay

| 名称 | タイプ | デフォルト値 | 説明 |
|---|---|---|---|
| `property_width_` | int | 128 | プロッターウィンドウの幅 [px] |
| `property_height_` | int | 128 | プロッターウィンドウの高さ [px] |
| `property_left_` | int | 128 | プロッターウィンドウの左 [px] |
| `property_top_` | int | 128 | プロッターウィンドウの上 [px] |
| `property_signal_color_` | QColor | QColor(25, 255, 240) | ウインカーの色 |

## 想定/既知の制限

未定

## 使用法

1. `rviz2` を起動し、`Displays` パネルの下にある `追加` ボタンをクリックする。

   ![select_add](./assets/images/select_add.png)

2. `By display type` タブで、`autoware_overlay_rviz_plugin/SignalDisplay` を選択し、OK を押す。

3. 必要に応じてトピック名を入力する。

   ![select_topic_name](./assets/images/select_topic_name.png)

