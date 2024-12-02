# mrm_comfortable_stop_operator

## 目的

MRM comfortable stop operatorは、comfortable stop MRM注文に従って快適な停止コマンドを生成するノードです。

## 仕組み / アルゴリズム

## 入出力

### 入力

| お名前                               | 型                                | 説明 |
| ------------------------------------ | --------------------------------- | ---------------------- |
| `~/input/mrm/comfortable_stop/operate` | `tier4_system_msgs::srv::OperateMrm` | MRM 実行命令 |

### 出力

Autoware 自律運転ソフトウェアの Planning モジュールは、SLAM と地図データを使用して、世界を理解し、自律走行に必要な経路計画を行います。

#### フローチャート

```
Planning
├── Local Planning
│  ├── Trajectory Generation
│  └── Path Planning
├── Motion Planning
│  └── Behavior Planning
├── Dynamic Planning
│  └── Obstacle Avoidance
└── Global Planning
   └── Route Planning
```

#### Planning モジュールの処理

1. **SLAM と地図データの融合:** Planning モジュールは、SLAM と地図データの両方を融合し、世界を理解します。このデータにより、Planning モジュールは自車位置を把握し、周囲の環境を認識することができます。
2. **経路計画:** Planning モジュールは、経路計画を行います。これには、目的地への最適な経路を見つけ、経路上の障害物を回避することが含まれます。
3. **動作計画:** Planning モジュールは、動作計画を行います。これには、自車の速度、加速度、操舵角を決定することが含まれます。
4. **障害物回避:** Planning モジュールは、障害物回避を行います。これには、自車が障害物と衝突することを回避する経路を決定することが含まれます。
5. **経路生成:** Planning モジュールは、経路生成を行います。これには、自車が目的地までたどる必要がある一連の点を作成することが含まれます。
6. **データの出力:** Planning モジュールは、自車の制御系に経路と動作の情報を提供します。

#### 追加情報

* Planning モジュールは、'post resampling` 後のSLAM データを使用します。
* Planning モジュールは、Autoware の他のモジュールと通信します。
* Planning モジュールは、ROS によって実装されています。

| 名前                                      | タイプ                                                     | 説明                                      |
| ---------------------------------------- | --------------------------------------------------------- | ----------------------------------------- |
| `~/output/mrm/comfortable_stop/status`   | `tier4_system_msgs::msg::MrmBehaviorStatus`              | MRMの実行状況                           |
| `~/output/velocity_limit`                | `tier4_planning_msgs::msg::VelocityLimit`                | 速度制限コマンド                          |
| `~/output/velocity_limit/clear`          | `tier4_planning_msgs::msg::VelocityLimitClearCommand`     | 速度制限クリアコマンド                     |

## パラメータ

### ノードパラメータ

| 名称        | 種類 | デフォルト値 | 説明                   |
| ----------- | ---- | ------------- | ----------------------------- |
| update_rate | int  | `10`          | タイマーコールバック頻度 [Hz] |

### コアパラメータ

| 名前 | タイプ | デフォルト値 | 説明 |
|---|---|---|---|
| min_acceleration | double | `-1.0` | 快適な停止のための最小加速度 [m/s^2] |
| max_jerk | double | `0.3` | 快適な停止のための最大ジャーク [m/s^3] |
| min_jerk | double | `-0.3` | 快適な停止のための最小ジャーク [m/s^3] |

## 前提条件／既知の制限

TBD.

