## tier4_state_rviz_plugin

## 目的

このプラグインは、Autowareの現在の状態を表示します。
このプラグインはパネルからの動作にも対応しています。

## 入出力

### 入力

| 名称                                        | タイプ                                                        | 説明                                                   |
| ----------------------------------------- | -------------------------------------------------------------- | ------------------------------------------------------------- |
| `/api/operation_mode/state`              | `autoware_adapi_v1_msgs::msg::OperationModeState`              | オペレーションモードの状態を表すトピック                 |
| `/api/routing/state`                     | `autoware_adapi_v1_msgs::msg::RouteState`                      | ルートの状態を表すトピック                              |
| `/api/localization/initialization_state` | `autoware_adapi_v1_msgs::msg::LocalizationInitializationState` | ローカリゼーション初期化の状態を表すトピック             |
| `/api/motion/state`                      | `autoware_adapi_v1_msgs::msg::MotionState`                     | モーションの状態を表すトピック                           |
| `/api/autoware/get/emergency`            | `tier4_external_api_msgs::msg::Emergency`                      | 外部緊急の状態を表すトピック                             |
| `/vehicle/status/gear_status`            | `autoware_vehicle_msgs::msg::GearReport`                       | ギアの状態を表すトピック                                 |

### 出力

[日本語訳のドキュメントのURLをここに入力してください]

| 名称                                                 | タイプ                                                   | 説明                                              |
| --------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------- |
| `/api/operation_mode/change_to_autonomous`         | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | 自律運転モードへの切り替えサービス                 |
| `/api/operation_mode/change_to_stop`               | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | 停止モードへの切り替えサービス                     |
| `/api/operation_mode/change_to_local`              | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | ローカルモードへの切り替えサービス                   |
| `/api/operation_mode/change_to_remote`             | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | リモートモードへの切り替えサービス                   |
| `/api/operation_mode/enable_autoware_control`      | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | Autoware による車両制御の有効化サービス              |
| `/api/operation_mode/disable_autoware_control`     | `autoware_adapi_v1_msgs::srv::ChangeOperationMode` | Autoware による車両制御の無効化サービス              |
| `/api/routing/clear_route`                         | `autoware_adapi_v1_msgs::srv::ClearRoute`          | ルート状態のクリアサービス                           |
| `/api/motion/accept_start`                         | `autoware_adapi_v1_msgs::srv::AcceptStart`         | 車両への走行開始の承認サービス                     |
| `/api/autoware/set/emergency`                      | `tier4_external_api_msgs::srv::SetEmergency`       | 外部緊急事態の設定サービス                         |
| `/planning/scenario_planning/max_velocity_default` | `tier4_planning_msgs::msg::VelocityLimit`          | 車両の最高速度の設定トピック                        |

## 使用方法

1. rvizを起動し、パネル/新しいパネルを追加 から選択します。

   ![select_panel](./images/select_panels.png)

2. tier4_state_rviz_plugin/AutowareStatePanelを選択し、OKを押します。

   ![select_state_plugin](./images/select_state_plugin.png)

3. 自動 ボタンがアクティブであれば、クリックしてエンゲージできます。

   ![select_auto](./images/select_auto.png)

