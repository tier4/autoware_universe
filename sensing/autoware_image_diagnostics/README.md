## イメージ診断

## 目的

`image_diagnostics`は、入力された生画像のステータスを確認するノードです。

## 内部処理 / アルゴリズム

下記の図は、画像診断ノードのフローチャートを示しています。各画像は、ブロック状態評価用に小さなブロックに分割されます。

![イメージ診断フローチャート](./image/image_diagnostics_overview.svg)

各小さな画像ブロックの状態は、以下の図のように評価されます。

![ブロック状態決定木](./image/block_state_decision.svg)

すべての画像のブロックの状態が評価された後、画像全体のステータスは以下のように要約されます。

![全体の画像状態決定木](./image/image_status_decision.svg)

## 入出力

### 入力
1. `raw image topic` (sensor_msgs/Image): This topic should be subscribed to the topic that contains the raw images.
2. `camera calibration parameters` (`yaml`): This topic should be subscribed to the topic that contains the camera calibration parameters for the camera that captured the images.
3. `test image params` (`yaml`): This topic should be subscribed to the topic that contains the test image parameters.

### 出力
1. `image diagnostic results topic` (diagnostic_msgs/DiagnosticStatus): This topic should be published to the topic that contains the image diagnostic results.

| 名              | 型                      | 説明 |
| ----------------- | ------------------------- | ------- |
| `input/raw_image` | `sensor_msgs::msg::Image` | 生画像 |

### 出力

ここでは、PlanningモジュールのOutputに関するドキュメントを記載します。

**AVP Plan Output**

| ヘッダ | フィールド | データ型 | 説明 |
|---|---|---|---|
| | [submit_plan](https://gitlab.com/ros-planning/plan_proposal_evaluator/-/wikis/submit_plan) | PlanProposal | Planningモジュールからの提案 |
| | [submit_plan_feedback](https://gitlab.com/ros-planning/plan_proposal_evaluator/-/wikis/submit_plan_feedback) | PlanEvaluation | PPEからのフィードバック |
| | [replan](https://gitlab.com/ros-planning/plan_proposal_evaluator/-/wikis/replan) | Boolean | PPEからプランの再生成をリクエスト |

**Path**

| ヘッダ | フィールド | データ型 | 説明 |
|---|---|---|---|
| sensor_msgs/NavSatFix | [header](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html) | ヘッダー | タイムスタンプとフレーム |
| mavros_msgs/Altitude | [altitude](http://docs.ros.org/api/mavros_msgs/html/msg/Altitude.html) | 高度 | WGS 84座標系での自車位置 |
| geometry_msgs/PoseStamped | [current_pose](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) | 姿勢 | WGS 84座標系での自車位置 |
| geometry_msgs/PoseWithCovarianceStamped | [current_pose_covariance](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | 姿勢の共分散 | `post resampling`によるプランの再生成のための推定 |
| autoware_auto_planning_msgs/PathWithLaneId | [path](https://gitlab.com/autowarefoundation/autoware.auto/-/blob/master/ros/src/autoware_auto_planning_msgs/msg/PathWithLaneId.msg) | 軌跡 | 5次多項式スプラインを使用した軌跡 |
| autoware_auto_planning_msgs/StopPointWithLaneId | [stop_points](https://gitlab.com/autowarefoundation/autoware.auto/-/blob/master/ros/src/autoware_auto_planning_msgs/msg/StopPointWithLaneId.msg) | 停止位置 | 多角形内の停止位置 |
| autoware_auto_planning_msgs/LaneStopPoint | [lane_stop_points](https://gitlab.com/autowarefoundation/autoware.auto/-/blob/master/ros/src/autoware_auto_planning_msgs/msg/LaneStopPoint.msg) | 車線停止位置 | 車線ごとの停止位置 |
| std_msgs/Bool | [is_replan](https://docs.ros.org/api/std_msgs/html/msg/Bool.html) | 再生成フラグ | プランニングの再生成が必要かどうか |

| 名称                                | タイプ                                    | 説明                           |
| ------------------------------------ | --------------------------------------- | ------------------------------------- |
| `image_diag/debug/gray_image`       | `sensor_msgs::msg::Image`               | グレースケール画像                  |
| `image_diag/debug/dft_image`        | `sensor_msgs::msg::Image`               | 離散フーリエ変換画像             |
| `image_diag/debug/diag_block_image` | `sensor_msgs::msg::Image`               | 各ブロックの状態を示す色分け画像 |
| `image_diag/image_state_diag`       | `tier4_debug_msgs::msg::Int32Stamped`   | 画像診断ステータス値               |
| `/diagnostics`                      | `diagnostic_msgs::msg::DiagnosticArray` | 障害                             |

## パラメータ

## 仮定/既知の制限

- これは、画像診断の概念実証であり、アルゴリズムはさらなる改善が必要です。

## (オプション) エラー検出と処理

## (オプション) パフォーマンスの特性評価

## (オプション) 参考文献/外部リンク

## (オプション) 今後拡張/未実装部分

- 雨滴や埃など、さらに特定の画像の歪み/遮蔽タイプを考慮してください。

- 光学的な観点からの霧や雨の状態における視程不良を考慮してください

