# RTCインターフェース

## 目的

RTCインターフェースは、挙動計画モジュールの決定状況をパブリッシュし、外部の自律運転システムから実行コマンドを受信するためのインターフェースです。

## 内部動作/アルゴリズム

### 使用例


```c++
// Generate instance (in this example, "intersection" is selected)
autoware::rtc_interface::RTCInterface rtc_interface(node, "intersection");

// Generate UUID
const unique_identifier_msgs::msg::UUID uuid = generateUUID(getModuleId());

// Repeat while module is running
while (...) {
  // Get safety status of the module corresponding to the module id
  const bool safe = ...

  // Get distance to the object corresponding to the module id
  const double start_distance = ...
  const double finish_distance = ...

  // Get time stamp
  const rclcpp::Time stamp = ...

  // Update status
  rtc_interface.updateCooperateStatus(uuid, safe, start_distance, finish_distance, stamp);

  if (rtc_interface.isActivated(uuid)) {
    // Execute planning
  } else {
    // Stop planning
  }
  // Get time stamp
  const rclcpp::Time stamp = ...

  // Publish status topic
  rtc_interface.publishCooperateStatus(stamp);
}

// Remove the status from array
rtc_interface.removeCooperateStatus(uuid);
```

## 入出力

### RTCInterface (コンストラクター)


```c++
autoware::rtc_interface::RTCInterface(rclcpp::Node & node, const std::string & name);
```

#### 説明

`autoware::rtc_interface::RTCInterface` のコンストラクタです。

#### 入力

- `node` : このインターフェイスを呼び出すノード
- `name` : cooperative（協調走行）ステータスアレイのトピック名と cooperative コマンドサービスの名前
  - cooperative ステータスアレイのトピック名 : `~/{name}/cooperate_status`
  - cooperative コマンドサービスの名前 : `~/{name}/cooperate_commands`

#### 出力

`RTCInterface` のインスタンス

### publishCooperateStatus


```c++
autoware::rtc_interface::publishCooperateStatus(const rclcpp::Time & stamp)
```

#### 説明

登録された協力状況を公開します。

#### 入力

- `stamp`: タイムスタンプ

#### 出力

なし

### updateCooperateStatus


```c++
autoware::rtc_interface::updateCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid, const bool safe, const double start_distance, const double finish_distance, const rclcpp::Time & stamp)
```

#### 説明

`uuid`に対応する協調ステータスを更新します。
`uuid`に対応する協調ステータスがまだ登録されていない場合は、新しい協調ステータスを追加します。

#### 入力

- `uuid`：要求モジュールのUUID
- `safe`：要求モジュールの安全ステータス
- `start_distance`：自車位置から開始オブジェクトまでの距離
- `finish_distance`：自車位置から終了オブジェクトまでの距離
- `stamp`：タイムスタンプ

#### 出力

なし

### removeCooperateStatus


```c++
autoware::rtc_interface::removeCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid)
```

#### 説明

登録されたステータスから、`uuid`に対応する協調ステータスを削除します。

#### 入力

- `uuid` : 無効になったモジュールのUUID

#### 出力

なし

### clearCooperateStatus


```c++
autoware::rtc_interface::clearCooperateStatus()
```

#### 説明

すべての共同作業ステータスを削除します。

#### 入力

なし

#### 出力

なし

### isActivated


```c++
autoware::rtc_interface::isActivated(const unique_identifier_msgs::msg::UUID & uuid)
```

#### 説明

`uuid` に対応する受信コマンドステータスを返します。

#### 入力

- `uuid` : チェックするモジュールの UUID

#### 出力

自動モードが有効な場合、セーフティステータスに基づいて返します。
それ以外は、受信コマンドが `ACTIVATED` である場合は `true` を返します。
それ以外の場合は `false` を返します。

### isRegistered


```c++
autoware::rtc_interface::isRegistered(const unique_identifier_msgs::msg::UUID & uuid)
```

#### 説明

`uuid` が登録されている場合に `true` を返します。

#### 入力

- `uuid` : モジュールの確認用 UUID

#### 出力

`uuid` が登録されている場合は `true` を返します。
登録されていない場合は `false` を返します。

## デバッグツール

RTC インターフェイスには、[RTC Replayer](https://autowarefoundation.github.io/autoware_tools/main/planning/autoware_rtc_replayer/) というデバッグツールがあります。

## 仮定/既知の制限事項

## 今後の拡張/未実装の部分

