# gnss_poser

## 目的

`gnss_poser` は GNSS センシングメッセージをサブスクライブし、共分散を使用した車両のポージングを計算するノードです。

このノードは NavSatFix をサブスクライブし、**base_link** のポージングをパブリッシュします。NavSatFix のデータはアンテナの位置を表します。そのため、`base_link` からアンテナ位置への tf を使用して座標変換を実行します。アンテナ位置の frame_id は NavSatFix の `header.frame_id` を参照します。(**NavSatFix の `header.frame_id` は地球や基準楕円体ではなく、アンテナの frame_id を示していることに注意してください。[NavSatFix 定義も参照してください。](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))

`base_link` からアンテナへの変換が取得できない場合は、座標変換を実行せずにアンテナ位置のポージングを出力します。

## 内部処理 / アルゴリズム

## 入出力

### 入力

| 名前                           | タイプ                                                  | 説明                                                                                                                |
| ------------------------------ | --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `/map/map_projector_info`      | `tier4_map_msgs::msg::MapProjectorInfo`                 | 地図投影情報                                                                                                          |
| `~/input/fix`                  | `sensor_msgs::msg::NavSatFix`                           | GNSSステータスメッセージ                                                                                              |
| `~/input/autoware_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | 方位 [詳細はこちら](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs) |

### 출력

### Autoware 自動運転システムアーキテクチャ

#### 自動運転ソフトウェアスタック

自動運転ソフトウェアスタックは、次の主要なレイヤーで構成されています。

- Perception
  - カメラ、レーダー、LiDARセンサからの生のデータを意味のある情報に変換する。
- Planning
  - 知覚データに基づいて、車両の経路を計画する。
- Behavior Prediction
  - 周囲の車両や歩行者の動きを予測する。
- Path Optimization
  - Planningからの経路を最適化し、周囲の環境への適応性を高める。
- Decision Making
  - 複数のPlanningとPath Optimizationからの候補経路を考慮し、最終的な経路を決定する。
- Motion Planning
  - 経路に従って車両を制御するための詳細な軌道を生成する。
- Controller
  - Motion Planningからの軌道を追従するように車両を制御する。
- Behavior Execution
  - Controllerからのコマンドを実行し、車両を制御する。

#### Perceptionレイヤー

Perceptionレイヤーは、主にセンサデータの処理を担当します。このレイヤーは次のモジュールで構成されています。

- **localization:** 車両の自車位置と姿勢を推定する。
- **object detection:** 車両、歩行者、障害物などの周囲の物体を検出する。
- **semantic segmentation:** 画像内のピクセルを、道路、歩道、建物などの意味のあるセグメントに分類する。
- **lane detection:** 道路上の車線を検出する。
- **traffic sign recognition:** 交通標識を認識する。

#### Planningレイヤー

Planningレイヤーは、知覚データに基づいて車両の経路を計画します。このレイヤーは次のモジュールで構成されています：

- **motion planner:** 車両の軌跡を生成する。
- **path planner:** Planningコンポーネントが生成した複数の候補経路を評価し、最適な経路を選択する。
- **trajectory optimizer:** 選択された経路を最適化し、周囲の環境への適応性を高める。

#### Behavior Predictionレイヤー

Behavior Predictionレイヤーは、周囲の車両や歩行者の動きを予測します。このレイヤーは次のモジュールで構成されています：

- **interactive object tracker:** 周囲の車両や歩行者を追跡する。
- **behavior predictor:** 追跡されたオブジェクトの将来の動作を予測する。

#### Path Optimizationレイヤー

Path Optimizationレイヤーは、Planningコンポーネントから生成された経路を最適化します。このレイヤーは次のモジュールで構成されています：

- **local path planner:** 'post resampling'後に経路を滑らかにし、周囲の環境への適応性を高める。
- **junction planner:** 交差点で安全かつ効率的な経路を計画する。

#### Decision Makingレイヤー

Decision Makingレイヤーは、PlanningとPath Optimizationレイヤーからの複数の候補経路を考慮し、最終的な経路を決定します。このレイヤーは次のモジュールで構成されています：

- **decision maker:** 候補経路を評価し、最も適切な経路を選択する。

#### Motion Planningレイヤー

Motion Planningレイヤーは、経路に従って車両を制御するための詳細な軌道を生成します。このレイヤーは次のモジュールで構成されています：

- **lateral controller:** 車両の横方向速度を制御する。
- **longitudinal controller:** 車両の縦方向速度を制御する。

#### Controllerレイヤー

Controllerレイヤーは、Motion Planningレイヤーからの軌道を追従するように車両を制御します。このレイヤーは次のモジュールで構成されています：

- **vehicle controller:** ブレーキ、アクセル、ステアリングを制御する。

#### Behavior Executionレイヤー

Behavior Executionレイヤーは、Controllerレイヤーからのコマンドを実行し、車両を制御します。このレイヤーは次のモジュールで構成されています：

- **actuators:** ブレーキ、アクセル、ステアリングなどのアクチュエータを制御する。

| 名前                     | タイプ                                            | 説明                                                        |
| ------------------------ | ----------------------------------------------- | ------------------------------------------------------------ |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | GNSSセンシングデータから計算された自車位置                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | GNSSセンシングデータから計算された共分散付きの自車位置   |
| `~/output/gnss_fixed`    | `tier4_debug_msgs::msg::BoolStamped`            | GNSS固定ステータス                                         |

## パラメータ

### コアパラメータ

{{ json_to_markdown("sensing/autoware_gnss_poser/schema/gnss_poser.schema.json") }}

## 想定事項と既知の制限事項

## (オプション) エラー検知と処理

## (オプション) 性能特性評価

## (オプション) 参考文献/外部リンク

## (オプション) 将来の拡張機能/実装予定外の部分

