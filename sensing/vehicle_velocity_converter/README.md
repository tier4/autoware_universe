# vehicle_velocity_converter

## 目的

このパッケージは、autoware_vehicle_msgs::msg::VehicleReportメッセージをgyro odometerノードのgeometry_msgs::msg::TwistWithCovarianceStampedに変換します。

## 入出力

### 入力

| 名称              | 種類                                        | 説明      |
| ----------------- | ------------------------------------------- | ---------------- |
| velocity_status | `autoware_vehicle_msgs::msg::VehicleReport` | 車両速度 |

### 出力

**自動運転ソフトウェア**

**目的**

このドキュメントでは、自動運転ソフトウェアアーキテクチャの全体的な概要を示します。

**モジュール**

**Perception**

* センサーデータの取得と解析
* 周囲環境の地図作成
* 物体検出と追跡

**Planning**

* **Planning Optimizer**
    * 安全かつ効率的な経路計画
* **Path Planner**
    * 自車および周囲の車両の挙動予測に基づく経路生成
* **Collision Checker**
    * 障害物との衝突の可能性検出

**Control**

* **Lateral Controller**
    * ステアリング制御
* **Longitudinal Controller**
    * 加減速制御
* **Vehicle Interface**
    * 車両への指令送信

**Localization**

* 自車位置と姿勢の推定
* 慣性センサー、GPS、ビジョンデータの統合

**Mapping**

* 環境地図の構築と更新
* 高精度な位置決めとナビゲーション

**Post Resampling**

* センサーデータのダウンサンプル後の再サンプル処理
* リアルタイムパフォーマンスの向上

**通信**

* 他車両やインフラストラクチャとの情報共有
* V2XおよびITSのサポート

**Autoware Stack**

このソフトウェアアーキテクチャは、Autoware Foundationによって開発されたAutoware Stackに基づいています。Autoware Stackは、自動運転ソフトウェアのオープンソースプラットフォームです。

| 名称                    | タイプ                                             | 説明                                             |
| ----------------------- | ------------------------------------------------ | -------------------------------------------------- |
| `twist_with_covariance` | `geometry_msgs::msg::TwistWithCovarianceStamped` | VehicleReportから変換された共分散付きtwist |

## パラメータ

| 名称                         | タイプ   | 説明                                          |
| ---------------------------- | ------ | ---------------------------------------------- |
| `speed_scale_factor`         | double | 速度スケールファクター（推奨値は1.0）       |
| `frame_id`                   | string | 出力メッセージのフレームID                      |
| `velocity_stddev_xx`         | double | vxの標準偏差                                  |
| `angular_velocity_stddev_zz` | double | ヨーレートの標準偏差                           |

