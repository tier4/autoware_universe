# autoware_external_cmd_selector

## 目的

`autoware_external_cmd_selector` は、現在のモード（リモートまたはローカル）に応じて `external_control_cmd`, `gear_cmd`, `hazard_lights_cmd`, `heartbeat`、`turn_indicators_cmd` をパブリッシュするパッケージです。

現在のモードは、リモート操作される `remote` と、Autoware によって計算された値を使用する `local` というサービス経由で設定されます。

## 入出力

### 入力トピック

| 名前                                              | タイプ | 説明                                                                   |
| ------------------------------------------------ | ---- | ---------------------------------------------------------------------- |
| `/api/external/set/command/local/control`         | TBD  | Local. 計算された制御値。                                           |
| `/api/external/set/command/local/heartbeat`       | TBD  | Local. ハートビート。                                                  |
| `/api/external/set/command/local/shift`           | TBD  | Local. ドライブ、リバースなどのギアシフト。                             |
| `/api/external/set/command/local/turn_signal`     | TBD  | Local. 左折、右折などのターンシグナル。                                |
| `/api/external/set/command/remote/control`        | TBD  | Remote. 計算された制御値。                                           |
| `/api/external/set/command/remote/heartbeat`      | TBD  | Remote. ハートビート。                                                 |
| `/api/external/set/command/remote/shift`          | TBD  | Remote. ドライブ、リバースなどのギアシフト。                            |
| `/api/external/set/command/remote/turn_signal`    | TBD  | Remote. 左折、右折などのターンシグナル。                              |

### 出力トピック

| 名前                                                 | 種類                                                            | 説明                                             |
| ------------------------------------------------------ | ---------------------------------------------------------------- | ------------------------------------------------ |
| `/control/external_cmd_selector/current_selector_mode` | TBD                                                           | 現在の選択モード（リモートまたはローカル）         |
| `/diagnostics`                                         | diagnostic_msgs::msg::DiagnosticArray                         | ノードのアクティビティを確認する                   |
| `/external/selected/external_control_cmd`              | TBD                                                           | 現在のモードでコントロールコマンドを渡す           |
| `/external/selected/gear_cmd`                          | autoware_vehicle_msgs::msg::GearCommand                       | 現在のモードでギヤコマンドを渡す                 |
| `/external/selected/hazard_lights_cmd`                 | autoware_vehicle_msgs::msg::HazardLightsCommand                 | 現在のモードでハザードライトを渡す               |
| `/external/selected/heartbeat`                         | TBD                                                           | 現在のモードでハートビートを渡す                 |
| `/external/selected/turn_indicators_cmd`               | autoware_vehicle_msgs::msg::TurnIndicatorsCommand               | 現在のモードでターニンジケーターを渡す           |

## パラメーター

{{json_to_markdown("control/autoware_external_cmd_selector/schema/external_cmd_selector.schema.json")}}

