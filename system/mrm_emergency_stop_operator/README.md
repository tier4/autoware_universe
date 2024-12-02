## mrm_emergency_stop_operator

## 目的

MRM emergency stop operatorは、MRMの緊急停止命令に従って緊急停止コマンドを生成するノードです。

## 内部機構/アルゴリズム

## 入力/出力

### 入力

| 名前 | タイプ | 説明 |
|---|---|---|
| `~/input/mrm/emergency_stop/operate` | `tier4_system_msgs::srv::OperateMrm` | MRM 実行順序 |
| `~/input/control/control_cmd` | `autoware_control_msgs::msg::Control` | 制御コンポーネントの最後のノードから出力される制御コマンド。緊急停止コマンドの初期値として使用されます。 |

### 出力

**自動運転ソフトウェア**

**概要**

本ドキュメントでは、Autowareのアーキテクチャの概要と、そのPlanningコンポーネントの設計について説明します。

**アーキテクチャ**

Autowareのアーキテクチャは、次の主要コンポーネントで構成されています。

* **Perception（知覚）コンポーネント:** センサーデータから周辺環境を認識します。
* **Planning（計画）コンポーネント:** 自車位置に基づいて安全かつ効率的な経路を計画します。
* **Control（制御）コンポーネント:** 計画された経路に従って車両を制御します。

**Planningコンポーネント**

Planningコンポーネントは、次のサブコンポーネントで構成されています。

* **Global Planning（グローバル計画）モジュール:** グローバル経路を計画します。
* **Local Planning（ローカル計画）モジュール:** ローカル経路を計画し、障害物を回避します。
* **Motion Planning（運動計画）モジュール:** 安全かつ快適な車両の運動を計画します。

**グローバル計画**

グローバル計画モジュールは、**'post resampling'**アルゴリズムを使用して、事前定義されたマップからグローバル経路を生成します。

**ローカル計画**

ローカル計画モジュールは、**'D*LITE'**アルゴリズムを使用して、リアルタイムセンサーデータに基づいてローカル経路を計画します。

**運動計画**

運動計画モジュールは、**'Covariance Matrix Adaptation Evolution Strategy (CMA-ES)'**アルゴリズムを使用して、車両の運動を計画します。これにより、車両の安定性と快適性が向上します。

| 名称                                     | 型                                         | 説明                    |
| ---------------------------------------- | -------------------------------------------- | ----------------------- |
| `~/output/mrm/emergency_stop/status`     | `tier4_system_msgs::msg::MrmBehaviorStatus` | MRM実行状態             |
| `~/output/mrm/emergency_stop/control_cmd` | `autoware_control_msgs::msg::Control`       | 緊急停止コマンド           |

## パラメータ

### ノードのパラメータ

| Name        | Type | Default value | Explanation                   |
| ----------- | ---- | ------------- | ----------------------------- |
| update_rate | int  | `30`          | Timer callback frequency [Hz] |

### パラメータのコア

---

| 名前                | タイプ   | 初期値 | 説明                                                |
| ------------------- | ------ | ------------- | -------------------------------------------------- |
| target_acceleration | double | `-2.5`        | 緊急停止時の目標加速度 [m/s^2]                   |
| target_jerk         | double | `-1.5`        | 緊急停止時の目標ジャーク [m/s^3]                  |

## 前提条件 / 既知の制限事項

未定。

