# dummy_infrastructure

インフラストラクチャとの通信をデバッグするためのノードです。

## 使用方法


```sh
ros2 launch dummy_infrastructure dummy_infrastructure.launch.xml
ros2 run rqt_reconfigure rqt_reconfigure
```

## 入力 / 出力

### 入力

| 名前                      | タイプ                                               | 説明                                |
| ------------------------- | ---------------------------------------------------- | ------------------------------------ |
| `~/input/command_array`     | `tier4_v2x_msgs::msg::InfrastructureCommandArray` | インフラストラクチャのコマンド           |

### 出力

| Name                                       | Type                                                 | Description                                                                                          |
| -------------------------------------------- | ---------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `~/output/state_array`                     | `tier4_v2x_msgs::msg::VirtualTrafficLightStateArray` | 仮想信号機アレイ                                                                                       |

## パラメータ

### ノードパラメータ

| 名前                | 型   | デフォルト値 | 説明                                       |
| ------------------- | ------ | ------------- | ------------------------------------------------- |
| `update_rate`       | double | `10.0`        | タイマーコールバック周期 [Hz]                        |
| `use_first_command` | bool   | `true`        | 計器 ID を考慮                               |
| `use_command_state` | bool   | `false`       | コマンド状態を考慮                               |
| `instrument_id`     | string | ``            | コマンド ID として使用                                 |
| `approval`          | bool   | `false`       | 承認フィールドを ROS パラメータに設定                   |
| `is_finalized`      | bool   | `false`       | finalization が完了していない場合、`stop_line` で停止 |

## 前提 / 既知の限界

未定

