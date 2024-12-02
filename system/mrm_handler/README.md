## mrm_handler

## 目的

MRMハンドラは、OperationModeAvailabilityに格納されているシステム障害状態から適切なMRMを選択するノードです。

## 内部の仕組み/アルゴリズム

### 状態遷移

![mrm-state](image/mrm-state.svg)

## 入出力

### 入力

| 名前                                   | タイプ                                                | 説明                                                                                         |
| -------------------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `/localization/kinematic_state`        | `nav_msgs::msg::Odometry`                           | 車両が停止しているかどうかを判断するために使用されます。                           |
| `/system/operation_mode/availability`  | `tier4_system_msgs::msg::OperationModeAvailability` | operationModeAvailabilityに含まれるシステムのMRM動作から適切なMRMを選択します。 |
| `/vehicle/status/control_mode`         | `autoware_vehicle_msgs::msg::ControlModeReport`     | 車両モード（自動運転または手動運転）を確認するために使用されます。                             |
| `/system/mrm/emergency_stop/status`    | `tier4_system_msgs::msg::MrmBehaviorStatus`         | MRM緊急停止動作が利用可能かどうかを確認するために使用されます。                             |
| `/system/mrm/comfortable_stop/status`  | `tier4_system_msgs::msg::MrmBehaviorStatus`         | MRM快適停止動作が利用可能かどうかを確認するために使用されます。                             |
| `/system/mrm/pull_over_manager/status` | `tier4_system_msgs::msg::MrmBehaviorStatus`         | MRM路肩停止動作が利用可能かどうかを確認するために使用されます。                               |
| `/api/operation_mode/state`            | `autoware_adapi_v1_msgs::msg::OperationModeState`   | 現在の動作モードが自動運転または停止かを確認するために使用されます。                                 |

## 自動運転ソフトウェア

### 目次

- [プランニング](#プランニング)
- [検出](#検出)
- [軌道制御](#軌道制御)

### プランニング

プランニングは、自律走行車の経路を計画します。

* **ナビゲーション**：地図情報を使用して、自律走行車の経路を計画します。
* **ローカルパスプランニング**：周囲環境のセンサーデータを使用して、自律走行車の経路を微調整します。
* **モーションプランニング**：衝突回避や快適性を考慮して、自律走行車の経路を最適化します。

### 検出

検出は、自律走行車の周囲環境を認識します。

* **カメラ**：画像データを取得して、物体を検出します。
* **レーダー**：電磁波を使用して、物体の位置と速度を測定します。
* **LiDAR**：レーザーを使用して、周囲環境の詳細な3D地図を作成します。

### 軌道制御

軌道制御は、自律走行車の動きを制御します。

* **ステアリング**：自律走行車のステアリングを制御します。
* **ブレーキ**：自律走行車のブレーキを制御します。
* **アクセル**：自律走行車のアクセルを制御します。

### 付録

- Autowareのチュートリアル
- `post resampling`処理
- 自車位置の更新
- 検出モジュールの構成

| 名稱                                   | 型                                              | 説明                                           |
| --------------------------------------- | ------------------------------------------------- | ----------------------------------------------------- |
| `/system/emergency/gear_cmd`            | `autoware_vehicle_msgs::msg::GearCommand`         | 適切なMRMを実行するために必要（ギアコマンドを送信） |
| `/system/emergency/hazard_lights_cmd`   | `autoware_vehicle_msgs::msg::HazardLightsCommand` | 適切なMRMを実行するために必要（ターンシグナルコマンドを送信） |
| `/system/fail_safe/mrm_state`           | `autoware_adapi_v1_msgs::msg::MrmState`           | MRM実行状態と選択されたMRM動作の通知           |
| `/system/mrm/emergency_stop/operate`    | `tier4_system_msgs::srv::OperateMrm`              | MRM緊急停止の実行命令                           |
| `/system/mrm/comfortable_stop/operate`  | `tier4_system_msgs::srv::OperateMrm`              | MRM快適停止の実行命令                           |
| `/system/mrm/pull_over_manager/operate` | `tier4_system_msgs::srv::OperateMrm`              | MRM寄道の執行命令                               |

## パラメータ

{{ json_to_markdown("system/mrm_handler/schema/mrm_handler.schema.json") }}

## 前提条件 / 既知の限界

未定。

