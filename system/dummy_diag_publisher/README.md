# dummy_diag_publisher

## 目的

このパッケージは、デバッグや開発のためにダミーの診断データを出力します。

## 入出力

### 出力

| 名称           | 型                                     | 説明         |
| -------------- | ---------------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs::msgs::DiagnosticArray` | Diagnostics出力 |

## パラメータ

### ノードパラメータ

`DIAGNOSTIC_NAME` パラメータには、パラメータ YAML ファイルに存在する名前を設定する必要があります。コマンドラインから `status` パラメータが与えられた場合、自動的に `is_active` パラメータは `true` に設定されます。

| 名                        | 型   | 初期値 | 説明                             | 設定変更可能 |
| --------------------------- | ------ | ------------- | --------------------------------------- | -------------- |
| `update_rate`               | int    | `10`          | タイマーコールバック周期 [Hz]              | false          |
| `DIAGNOSTIC_NAME.is_active` | bool   | `true`        | 強制更新するかどうか                     | true           |
| `DIAGNOSTIC_NAME.status`    | string | `"OK"`        | Dummy Diagnostics Publisherによって設定されるDiagステータス | true           |

### YAML 形式（dummy_diag_publisher）

値が `default` の場合は、既定値が設定されます。

| キー                                      | タイプ | 既定値 | 説明                                     |
| ------------------------------------------ | ------ | --------- | ---------------------------------------- |
| `required_diags.DIAGNOSTIC_NAME.is_active` | bool   | `true`    | 強制更新するかどうか                    |
| `required_diags.DIAGNOSTIC_NAME.status`    | string | `"OK"`    | ダミーの診断パブリッシャーによって設定される診断ステータス |

## 想定事項/既知の制限事項

TBD.

## 使用方法

### launch


```sh
ros2 launch dummy_diag_publisher dummy_diag_publisher.launch.xml
```

### 再設定


```sh
ros2 param set /dummy_diag_publisher velodyne_connection.status "Warn"
ros2 param set /dummy_diag_publisher velodyne_connection.is_active true
```

