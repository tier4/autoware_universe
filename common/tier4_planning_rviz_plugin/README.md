## tier4_planning_rviz_plugin

このパッケージは jsk コードを含みます。
jsk_overlay_utils.cpp と jsk_overlay_utils.hpp は BSD ライセンスであることに注意してください。

## 目的

このプラグインは、経路、軌跡、最大速度を表示します。

## 入出力

### 入力

| 名前                                               | タイプ                                      | 説明                                        |
| -------------------------------------------------- | ----------------------------------------- | ------------------------------------------- |
| `/input/path`                                      | `autoware_planning_msgs::msg::Path`       | パスをサブスクライブするトピック             |
| `/input/trajectory`                                | `autoware_planning_msgs::msg::Trajectory` | 軌跡をサブスクライブするトピック           |
| `/planning/scenario_planning/current_max_velocity` | `tier4_planning_msgs/msg/VelocityLimit`   | 最大速度をパブリッシュするトピック         |

### 出力

**自動運転ソフトウェアに関するドキュメント**

**概要**

このドキュメントは、Autowareの自動運転ソフトウェアのPlanning（走行計画）コンポーネントに関する内容です。Planningコンポーネントは、センサーデータから自車位置を推定し、将来の経路を計画します。

**機能**

* 自車位置の推定
* 障害物の検出と追跡
* 走行経路の計画
* 安全性の評価
* `post resampling`

**技術**

* Kalmanフィルター
* 粒子フィルター
* A*アルゴリズム
* 動的計画法

**システムアーキテクチャ**

Planningコンポーネントは、Autowareのモジュール化されたソフトウェアアーキテクチャの一部です。他のコンポーネントと通信して、センサーデータ、自車位置、走行経路などの情報を共有します。

**使用例**

Planningコンポーネントは、自動運転車の開発に使用できます。このコンポーネントは、車両を安全に走行させるために、周辺環境を理解し、適切な決定を下すのに役立ちます。

**ライセンス**

このソフトウェアはApache License 2.0でライセンスされています。

| 名前                                    | タイプ                          | 説明                                  |
| --------------------------------------- | ------------------------------- | ---------------------------------------- |
| `/planning/mission_planning/checkpoint` | `geometry_msgs/msg/PoseStamped` | チェックポイントを発行するトピック |

## パラメータ

### コアパラメータ

#### MissionCheckpoint

| 名称                   | タイプ   | デフォルト値        | 説明                                        |
| ---------------------- | ------ | -------------------- | -------------------------------------------------- |
| `pose_topic_property_` | 文字列 | `mission_checkpoint` | チェックポイントをパブリッシュするトピック           |
| `std_dev_x_`           | 浮動小数 | 0.5                  | チェックポイントのX軸標準偏差 [m]       |
| `std_dev_y_`           | 浮動小数 | 0.5                  | チェックポイントのY軸標準偏差 [m]       |
| `std_dev_theta_`       | 浮動小数 | M_PI / 12.0          | チェックポイントのTheta軸標準偏差 [rad] |
| `position_z_`          | 浮動小数 | 0.0                  | チェックポイントのZ軸位置 [m]                 |

#### Path

| 名称 | タイプ | デフォルト値 | 説明 |
|---|---|---|---|
| `property_path_view_` | bool | true | パスプロパティを使用するかどうか |
| `property_path_width_view_` | bool | false | 定幅を使用するかどうか |
| `property_path_width_` | float | 2.0 | パスプロパティの幅 [m] |
| `property_path_alpha_` | float | 1.0 | パスプロパティのアルファ |
| `property_path_color_view_` | bool | false | 定色を使用するかどうか |
| `property_path_color_` | QColor | Qt::black | パスプロパティの色 |
| `property_velocity_view_` | bool | true | ベロシティプロパティを使用するかどうか |
| `property_velocity_alpha_` | float | 1.0 | ベロシティプロパティのアルファ |
| `property_velocity_scale_` | float | 0.3 | ベロシティプロパティのスケール |
| `property_velocity_color_view_` | bool | false | 定色を使用するかどうか |
| `property_velocity_color_` | QColor | Qt::black | ベロシティプロパティの色 |
| `property_vel_max_` | float | 3.0 | 最大速度 [m/s] |

#### 車両走行可能領域



| 名前                     | タイプ  | デフォルト値 | 説明                                   |
| ------------------------ | ----- | ------------- | ---------------------------------------- |
| `color_scheme_property_` | int   | 0             | 走行可能エリアプロパティのカラーパレット |
| `alpha_property_`        | float | 0.2           | 走行可能エリアプロパティのアルファ値     |
| `draw_under_property_`   | bool  | false         | 背景として描画するかどうかの設定          |

#### PathFootprint

| 名称                             | タイプ   | 初期値 | 説明                                 |
| -------------------------------- | ------ | ------------- | --------------------------------------- |
| `property_path_footprint_view_`  | bool   | true          | パスフットプリントプロパティの使用与否 |
| `property_path_footprint_alpha_` | float  | 1.0           | パスフットプリントプロパティのアルファ値 |
| `property_path_footprint_color_` | QColor | Qt::black     | パスフットプリントプロパティの色         |
| `property_vehicle_length_`       | float  | 4.77          | 車両の長さ [m]                           |
| `property_vehicle_width_`        | float  | 1.83          | 車両の幅 [m]                            |
| `property_rear_overhang_`        | float  | 1.03          | リアオーバーハング [m]                    |

#### Trajectory

- **目的：**
    - `Planning`コンポーネントが生成した`trajectory`を作成・管理します。
    - `Planning`コンポーネントから`trajectory`を受信し、`post resampling`を実行します。
    - 他のコンポーネントに`trajectory`を提供します。

- **詳細：**
    - `trajectory`の`post resampling`、再パラメータ化、および他のコンポーネントへの提供を行います。
    - `trajectory`に沿って現在の`pose`と誤差を計算します。

| 名前                            | タイプ   | デフォルト値 | 説明                  |
| ------------------------------- | ------ | ------------- | ---------------------------- |
| `property_path_view_`           | ブール   | true          | パスプロパティを使用するかどうか     |
| `property_path_width_`          | 浮動小数 | 2.0           | パスプロパティの幅 [m]   |
| `property_path_alpha_`          | 浮動小数 | 1.0           | パスプロパティのアルファ       |
| `property_path_color_view_`     | ブール   | false         | 定数カラーを使用するかどうか    |
| `property_path_color_`          | QColor | Qt::black     | パスプロパティの色       |
| `property_velocity_view_`       | ブール   | true          | ベロシティプロパティを使用するかどうか |
| `property_velocity_alpha_`      | 浮動小数 | 1.0           | ベロシティプロパティのアルファ   |
| `property_velocity_scale_`      | 浮動小数 | 0.3           | ベロシティプロパティのスケール   |
| `property_velocity_color_view_` | ブール   | false         | 定数カラーを使用するかどうか    |
| `property_velocity_color_`      | QColor | Qt::black     | ベロシティプロパティの色   |
| `property_velocity_text_view_`  | ブール   | false         | ベロシティのテキストを表示           |
| `property_velocity_text_scale_` | 浮動小数 | 0.3           | ベロシティプロパティのスケール   |
| `property_vel_max_`             | 浮動小数 | 3.0           | 最大ベロシティ [m/s]           |

#### TrajectoryFootprint

| 名前 | タイプ | デフォルト値 | 説明 |
| ---- | ---- | -------- | -------- |
| `property_trajectory_footprint_view_` | bool | true | Trajectory Footprintプロパティの使用可否 |
| `property_trajectory_footprint_alpha_` | float | 1.0 | Trajectory Footprintプロパティのアルファ |
| `property_trajectory_footprint_color_` | QColor | QColor(230, 230, 50) | Trajectory Footprintプロパティの色 |
| `property_vehicle_length_` | float | 4.77 | 車両の長さ [m] |
| `property_vehicle_width_` | float | 1.83 | 車両の幅 [m] |
| `property_rear_overhang_` | float | 1.03 | 後方オーバーハング [m] |
| `property_trajectory_point_view_` | bool | false | Trajectory Pointプロパティの使用可否 |
| `property_trajectory_point_alpha_` | float | 1.0 | Trajectory Pointプロパティのアルファ |
| `property_trajectory_point_color_` | QColor | QColor(0, 60, 255) | Trajectory Pointプロパティの色 |
| `property_trajectory_point_radius_` | float | 0.1 | Trajectory Pointプロパティの半径 |

#### MaxVelocity

| 名称                     | 型   | デフォルト値                                       | 説明                                      |
| ----------------------- | ------ | ---------------------------------------------------- | ------------------------------------------ |
| `property_topic_name_`  | 文字列 | `/planning/scenario_planning/current_max_velocity` | 最大速度を通知するトピック                  |
| `property_text_color_`  | QColor | QColor(255, 255, 255)                              | テキストの色                                 |
| `property_left_`        | 整数  | 128                                                | プロットウィンドウの左 [px]                  |
| `property_top_`         | 整数  | 128                                                | プロットウィンドウの上 [px]                   |
| `property_length_`      | 整数  | 96                                                 | プロットウィンドウの長さ [px]                |
| `property_value_scale_` | 浮動小数点 | 1.0 / 4.0                                          | 値のスケール                                |

## 利用方法

1. rviz を起動し、「ディスプレイ」パネルで「追加」を選択します。
   ![select_add](./images/select_add.png)
2. tier4_planning_rviz_plugin のいずれかを選択し、「OK」を押します。
   ![select_planning_plugin](./images/select_planning_plugin.png)
3. パスまたは軌跡を表示するトピックの名前を入力します。
   ![select_topic_name](./images/select_topic_name.png)

## マテリアル デザイン アイコン

このプロジェクトでは、Google の [マテリアル デザイン アイコン](https://developers.google.com/fonts/docs/material_symbols) を使用しています。これらのアイコンは Apache ライセンス、バージョン 2.0 の条件に基づき使用されています。

マテリアル デザイン アイコンは、Google が提供するシンボルのコレクションで、アプリケーション、Web サイト、その他のデジタル製品のユーザーインターフェイスを向上させるために使用されています。

### ライセンス

マテリアル デザイン アイコンは、Apache ライセンス、バージョン 2.0 に基づいてライセンスされています。ライセンスのコピーは次の URL から入手できます。

<http://www.apache.org/licenses/LICENSE-2.0>

適用される法律で要求されるか、書面で合意されない限り、ライセンスに基づいて配布されるソフトウェアは、明示的または黙示的な保証または条件なしで、「現状のまま」配布されます。ライセンスにおけるライセンスに基づく許可と制限を規定する具体的な言語については、ライセンスを参照してください。

### 謝辞

これらのアイコンをコミュニティに提供し、開発者やデザイナーがプロジェクトの視覚的魅力とユーザーエクスペリエンスを向上させるのに役立てた Google に感謝の意を表します。

