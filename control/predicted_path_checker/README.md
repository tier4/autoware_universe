# 予測経路チェッカー

## 目的

Predicted Path Checker パッケージは、制御モジュールによって生成された予測経路を自動運転車がチェックするために設計されています。プランニングモジュールでは処理できない可能性がある潜在的な衝突やブレーキ距離内での衝突を処理します。ブレーキ距離内の衝突が発生した場合、パッケージは「ERROR」というラベルの付いた診断メッセージを送信してシステムに緊急事態を警告し、参照軌道外の衝突が発生した場合は、インターフェースの一時停止を要求して車両を停止させます。

![general-structure.png](images%2Fgeneral-structure.png)

## アルゴリズム

パッケージアルゴリズムは、予測軌道と環境内の予測オブジェクトに対して予測軌道を評価します。潜在的な衝突をチェックし、必要に応じてそれらを避けるための適切な応答（緊急停止または一時停止リクエスト）を生成します。

### 内部アルゴリズム

![FlowChart.png](images%2FFlowChart.png)

**cutTrajectory() ->** 入力長で予測軌道を切断します。長さは、自我車両の速度に「trajectory_check_time」パラメーターと「min_trajectory_length」を乗算して計算されます。

**filterObstacles() ->** 環境内の予測オブジェクトをフィルタリングします。車両の前方になく、予測軌道から遠く離れたオブジェクトをフィルタリングします。

**checkTrajectoryForCollision() ->** 予測軌道の予測オブジェクトとの衝突をチェックします。軌跡ポイントの多角形と予測オブジェクトの両方を計算し、両方の多角形の交差をチェックします。交差があれば、最も近い衝突ポイントを計算します。多角形の最近接衝突点と予測オブジェクトを返します。また、予期しない挙動を避けるために、「chattering_threshold」秒前にフットプリントと交差していた予測オブジェクトの履歴もチェックします。予測オブジェクトの履歴は、オブジェクトが「chattering_threshold」秒前に検出された場合にオブジェクトを格納します。

「enable_z_axis_obstacle_filtering」パラメーターが true に設定されている場合、Z 軸の予測オブジェクトは「z_axis_filtering_buffer」を使用してフィルタリングされます。オブジェクトが Z 軸と交差しない場合、オブジェクトはフィルタリングされます。

![Z_axis_filtering.png](images%2FZ_axis_filtering.png)

**calculateProjectedVelAndAcc() ->** 予測軌道の衝突点の軸上で、予測オブジェクトの投影速度と加速度を計算します。

**isInBrakeDistance() ->** 停止点がブレーキ距離内にあるかどうかをチェックします。予測オブジェクトに対する自我車両の相対速度と加速度を取得します。ブレーキ距離を計算し、点がブレーキ距離内にある場合は true を返します。

**isItDiscretePoint() ->** 予測軌道の停止点が離散点かどうかをチェックします。離散点でない場合、プランニングで停止を処理する必要があります。

**isThereStopPointOnRefTrajectory() ->** 参照軌跡に停止点があるかどうかをチェックします。停止インデックスの前に停止点がある場合、true を返します。それ以外の場合は false を返し、ノードはインターフェースの一時停止を呼び出して車両を停止します。

## 入力

| 名称                                    | タイプ                                             | 説明                                           |
| ------------------------------------- | ------------------------------------------------ | ------------------------------------------------- |
| `~/input/reference_trajectory`        | `autoware_planning_msgs::msg::Trajectory`        | 基準軌道                                        |
| `~/input/predicted_trajectory`        | `autoware_planning_msgs::msg::Trajectory`        | 予測軌道                                        |
| `~/input/objects`                     | `autoware_perception_msgs::msg::PredictedObject` | 環境内の動的物体                                 |
| `~/input/odometry`                    | `nav_msgs::msg::Odometry`                        | オドメトリメッセージ（自車位置を取得するための車両速度） |
| `~/input/current_accel`               | `geometry_msgs::msg::AccelWithCovarianceStamped` | 現在加速度                                      |
| `/control/vehicle_cmd_gate/is_paused` | `tier4_control_msgs::msg::IsPaused`              | 車両の現在のポーズ状態                            |

## 出力

| 名称                                 | 型                                         | 説明                                   |
| ------------------------------------ | ----------------------------------------- | ---------------------------------------- |
| `~/debug/marker`                      | `visualization_msgs::msg::MarkerArray`   | ビジュアライゼーション用のマーカー       |
| `~/debug/virtual_wall`                | `visualization_msgs::msg::MarkerArray`   | ビジュアライゼーション用の仮想の壁マーカー |
| `/control/vehicle_cmd_gate/set_pause` | `tier4_control_msgs::srv::SetPause`      | 車両を停止させるためのポーズサービス    |
| `/diagnostics`                        | `diagnostic_msgs::msg::DiagnosticStatus` | 車両の診断ステータス                       |

## パラメーター

### ノードパラメーター

| 名前                                      | 種類     | 説明                                                                        | デフォルト値 |
| :---------------------------------------- | :------- | :-------------------------------------------------------------------------- | :------------ |
| `update_rate`                             | `double` | 更新レート [Hz]                                                            | 10.0          |
| `delay_time`                              | `double` | 緊急対応のために考慮される遅延時間 [s]                                     | 0.17          |
| `max_deceleration`                        | `double` | 自車が停止するための最大減速度 [m/s^2]                                  | 1.5           |
| `resample_interval`                       | `double` | 軌道の再サンプリングの間隔 [m]                                           | 0.5           |
| `stop_margin`                             | `double` | 停止マージン [m]                                                           | 0.5           |
| `ego_nearest_dist_threshold`              | `double` | 自車に対する最短距離のしきい値 [m]                                        | 3.0           |
| `ego_nearest_yaw_threshold`               | `double` | 自車に対する最短ヨー角のしきい値 [rad]                                     | 1.046         |
| `min_trajectory_check_length`             | `double` | メートル単位の最小軌跡チェック長 [m]                                     | 1.5           |
| `trajectory_check_time`                   | `double` | 軌跡チェック時間 [s]                                                    | 3.0           |
| `distinct_point_distance_threshold`       | `double` | 異なる点の距離しきい値 [m]                                                | 0.3           |
| `distinct_point_yaw_threshold`            | `double` | 異なる点のヨー角しきい値 [deg]                                          | 5.0           |
| `filtering_distance_threshold`            | `double` | この距離より高い場合はオブジェクトを無視します [m]                         | 1.5           |
| `use_object_prediction`                   | `bool`   | 真の場合、ノードはデルタ時間に対するオブジェクトの現在の位置を予測します [-] | true          |

### 衝突チェッカーのパラメータ

| 名称                                 | 種類     | 説明                                                             | デフォルト値 |
| :----------------------------------- | :------- | :------------------------------------------------------------------ | :------------ |
| `width_margin`                       | `double` | 衝突確認の幅のマージン [Hz]                                  | 0.2           |
| `chattering_threshold`               | `double` | 衝突検出のチャタリングしきい値 [s]                              | 0.2           |
| `z_axis_filtering_buffer`            | `double` | Z 軸フィルタリングバッファ [m]                                     | 0.3           |
| `enable_z_axis_obstacle_filtering` | `bool`   | Z 軸障害物フィルタリングが有効かどうかを示すブールフラグ       | false         |

