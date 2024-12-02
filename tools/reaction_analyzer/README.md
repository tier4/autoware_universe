# 反応アナライザー

## 説明

Reaction Analyzerパッケージの主な目的は、ROSベースの自律走行シミュレーション環境内のさまざまなノードの反応時間を測定することです。このツールは、障害物出現などの環境内の動的な変化に対して、認識、計画、制御パイプラインがどのように反応するかを評価するのに特に役立ちます。制御出力と認識出力を測定するには、ノードを2つの実行モード（`planning_control`と`perception_planning`）に分ける必要がありました。

![ReactionAnalyzerDesign.png](media%2FReactionAnalyzerDesign.png)

### Planning Control モード

このモードでは、Reaction AnalyzerはPredictedObjectsとPointCloud2トピックのダミーパブリッシャーを作成します。テストの開始時に、エゴカーの初期位置と目標位置をパブリッシュして、テスト環境を設定します。次に、エゴカーの前に突然障害物を発生させます。障害物が発生すると、あらかじめ定義されたトピックで、計画および制御ノードの反応メッセージの検索を開始します。すべてのトピックに反応すると、各ノードの`reacted_times`を`spawn_cmd_time`と比較してノードの反応時間と統計を計算し、結果を格納するCSVファイルを作成します。

### Perception Planning モード

このモードでは、Reaction AnalyzerはAWSIMから記録されたROSバッグファイルを読み取り、ROSバッグを再生するためのトピックパブリッシャーを各トピック内に作成します。`path_bag_without_object`と`path_bag_with_object`という2つのROSバッグファイルを読み取ります。最初に、`path_bag_without_object`を再生して、エゴカーの初期位置と目標位置を設定します。`spawn_time_after_init`秒後、`path_bag_with_object`を再生して、エゴカーの前に突然障害物を発生させます。障害物が発生すると、あらかじめ定義されたトピックで、認識および計画ノードの反応メッセージの検索を開始します。すべてのトピックに反応すると、各ノードの`reacted_times`を`spawn_cmd_time`と比較してノードの反応時間と統計を計算し、結果を格納するCSVファイルを作成します。

#### Point Cloudパブリッシャーの種類

認識とセンシングパイプラインをより詳しく分析するために、Reaction Analyzerは異なる3つの方法でポイントクラウドメッセージをパブリッシュできます。`async_header_sync_publish`、`sync_header_sync_publish`、または`async_publish`。（`T`はLiDARの出力期間を表します）

![PointcloudPublisherType.png](media%2FPointcloudPublisherType.png)

- `async_header_sync_publish`: ポイントクラウドメッセージを非同期ヘッダー時刻で同期してパブリッシュします。つまり、LiDARの各出力は同時にパブリッシュされますが、ポイントクラウドメッセージのヘッダーにはフェーズ差があるために異なるタイムスタンプが含まれます。
- `sync_header_sync_publish`: ポイントクラウドメッセージを同期ヘッダー時刻で同期してパブリッシュします。つまり、LiDARの各出力は同時にパブリッシュされ、ポイントクラウドメッセージのヘッダーには同じタイムスタンプが含まれます。
- `async_publish`: ポイントクラウドメッセージを非同期にパブリッシュします。つまり、LiDARの各出力は異なる時間にパブリッシュされます。

## 使用法

両方の実行モードで定義する必要がある共通のパラメータは、`output_file_path`、`test_iteration`、および`reaction_chain`リストです。`output_file_path`は、結果と統計が格納される出力ファイルパスです。`test_iteration`は、実行されるテストの数を定義します。`reaction_chain`リストは、反応時間を測定する事前に定義されたトピックのリストです。

**重要**: `reaction_chain`リストが正しく定義されていることを確認してください:

- `perception_planning`モードでは、`Control`ノードを定義**しないで**ください。

### 準備されたテスト環境

- デモ用テストマップを [こちら](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip) のリンクからダウンロードしてください。ダウンロード後、zip ファイルを展開し、そのパスを以下で `[MAP_PATH]` として使用してください。

#### プランニング制御モード

- `reaction_chain` リストにプランニングノードと制御ノードのみ定義する必要があります。デフォルトパラメータを使用して、次のコマンドでテストを開始できます:


```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=planning_control vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=[MAP_PATH]
```

コマンド実行後、`simple_planning_simulator`と`reaction_analyzer`が起動します。テストが自動的に開始されます。テスト完了後は、定義した`output_file_path`に結果が保存されます。

#### Perception Planningモード

- Google Driveのリンク[こちら](https://drive.google.com/drive/folders/1eJMEdt4WbU-W6MPXlNTkIhZtwpof0HcO?usp=sharing)からrosbagファイルをダウンロードします。
- zipファイルを解凍し、`.db3`ファイルのパスをパラメーター`path_bag_without_object`および`path_bag_with_object`に設定します。
- 以下のコマンドでテストを開始できます：


```bash
ros2 launch reaction_analyzer reaction_analyzer.launch.xml running_mode:=perception_planning vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=[MAP_PATH]
```

- ツールを `perception_planning` モードで初回実行すると、初期化に予想より時間がかかる場合があります。プロセスが完了するまでしばらくお待ちください。

コマンド後、 `e2e_simulator` と `reaction_analyzer` が起動します。自動的にテストが開始されます。テストが完了すると、結果は定義した `output_file_path` に保存されます。

#### 準備したテスト環境

**対象物のないシーン:**
![sc1-awsim.png](media%2Fsc1-awsim.png)
![sc1-rviz.png](media%2Fsc1-rviz.png)

**対象物のあるシーン:**
![sc2-awsim.png](media%2Fsc2-awsim.png)
![sc2-rviz.png](media%2Fsc2-rviz.png)

### カスタムテスト環境

**カスタムテスト環境でリアクションアナライザーを実行するには、いくつかのパラメーターを再定義する必要があります。再定義する必要があるパラメーターは、`initialization_pose`、`entity_params`、`goal_pose`、および`topic_publisher`（`perception_planning` モードの場合）のパラメーターです。**

- `initialization_pose`、`entity_params`、`goal_pose` を設定するには：
- AWSIM 環境を実行します。AWSIM のチュートリアルは [こちら](https://autowarefoundation.github.io/AWSIM/main/GettingStarted/QuickStartDemo/) でご覧いただけます。
- 次のコマンドで `e2e_simulator` を実行します。


```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=[MAP_PATH]
```

- EGOが初期化されたら、RViz内の「2D Pose Estimate」ボタンを使用して、目的の場所に自車位置を配置できます。
- EGOが所定の位置に配置されたら、交通管制を使用して、ダミー障害物を特定します。'ESC'キーを押して交通制御セクションにアクセスできます。

**EGOとダミー車両を特定したら、`reaction_analyzer.param.yaml`内のマップフレームで、これらのエンティティの位置を記述する必要があります。これを実現するには:**

- `/awsim/ground_truth/vehicle/pose`トピックから初期化ポーズを取得します。
- `/perception/object_recognition/objects`トピックからエンティティパラメーターを取得します。
- `/planning/mission_planning/goal`トピックからゴールポーズを取得します。

**追伸: `initialization_pose`は`planning_control`モードだけに有効です。**

- パラメーターが記録されたら、テストにROSバッグを記録する必要があります。ROSバッグを記録するには、次のコマンドを使用できます。


```bash
ros2 bag record --all
```

- オブジェクトなしとオブジェクトありの2つのROSバッグを記録する必要があります。交通管制を使用して、EGO車両の前面にオブジェクトを生成したり、削除したりできます。

**注意: 同じ環境で同一のEGO車両位置でROSバッグを記録する必要があります。記録中にAutowareを実行する必要はありません。**

- ROSバッグを記録したら、`path_bag_without_object`パラメータと`path_bag_with_object`パラメータを記録されたROSバッグのパスで設定できます。

## 結果

結果は`csv`ファイル形式で保存され、定義した`output_file_path`に出力されます。メッセージのヘッダタイムスタンプを使用してAutowareの各パイプラインを示し、各ノードの`Node Latency`、`Pipeline Latency`、および`Total Latency`を報告します。

- `Node Latency`: 前のノードと現在のノードのリアクションタイムスタンプの時間差。パイプラインの最初のノードの場合は、`Pipeline Latency`と同じです。
- `Pipeline Latency`: メッセージの公開時間とPipeline Header時間の時間差。
- `Total Latency`: メッセージの公開タイムスタンプと障害物出現コマンドが送信されたタイムスタンプの時間差。

## パラメータ

| 名前                                                                        | 型  | 説明                                                                                                                                   |
| --------------------------------------------------------------------------- | ---- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `timer_period`                                                              | double | [s] メイン処理タイマーの周期。                                                                                                     |
| `test_iteration`                                                            | int   | テストの反復回数。                                                                                                            |
| `output_file_path`                                                          | 文字列 | テスト結果と統計情報が保存されるディレクトリパス。                                                                              |
| `spawn_time_after_init`                                                     | double | [s] 物体をスポーンする前の初期化後の時間遅延。`perception_planning` モードでのみ有効。                                           |
| `spawn_distance_threshold`                                                  | double | [m] 物体をスポーンするための距離閾値。`planning_control` モードでのみ有効。                                                              |
| `poses.initialization_pose`                                                 | 構造体 | 車両の初期ポーズ。`x`, `y`, `z`, `roll`, `pitch`, `yaw` フィールドを含む。`planning_control` モードでのみ有効。                 |
| `poses.entity_params`                                                       | 構造体 | エンティティ（例：障害物）のパラメータ。`x`, `y`, `z`, `roll`, `pitch`, `yaw`, `x_dimension`, `y_dimension`, `z_dimension` を含む。 |
| `poses.goal_pose`                                                           | 構造体 | 車両の目標ポーズ。`x`, `y`, `z`, `roll`, `pitch`, `yaw` フィールドを含む。                                                        |
| `topic_publisher.path_bag_without_object`                                   | 文字列 | オブジェクトを含まない ROS バッグファイルへのパス。`perception_planning` モードでのみ有効。                                                              |
| `topic_publisher.path_bag_with_object`                                      | 文字列 | オブジェクトを含む ROS バッグファイルへのパス。`perception_planning` モードでのみ有効。                                                                 |
| `topic_publisher.spawned_pointcloud_sampling_distance`                      | double | [m] スポーンされたオブジェクトの点群のサンプル距離。`planning_control` モードでのみ有効。                                                |
| `topic_publisher.dummy_perception_publisher_period`                         | double | [s] ダミー知覚データのパブリッシング周期。`planning_control` モードでのみ有効。                                                      |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_type`            | 文字列 | PointCloud2 メッセージのパブリッシュ方法を定義します。上記で説明したモードがあります。                                                        |
| `topic_publisher.pointcloud_publisher.pointcloud_publisher_period`          | double | [s] PointCloud2 メッセージのパブリッシング周期。                                                                                            |
| `topic_publisher.pointcloud_publisher.publish_only_pointcloud_with_object` | bool   | デフォルトは false。オブジェクトを含む点群メッセージのみをパブリッシュします。                                                                         |
| `reaction_params.first_brake_params.debug_control_commands`               | bool   | デバッグパブリッシュフラグ。                                                                                                                           |
| `reaction_params.first_brake_params.control_cmd_buffer_time_interval`       | double | [s] コントロールコマンドのバッファリング時間間隔。                                                                                             |
| `reaction_params.first_brake_params.min_number_descending_order_control_cmd` | int    | ブレーキ作動のトリガーとなる降順制御コマンドの最小数。                                                                  |
| `reaction_params.first_brake_params.min_jerk_for_brake_cmd`                 | double | [m/s³] ブレーキコマンドを発行するための最小ジャーク値。                                                                                        |
| `reaction_params.search_zero_vel_params.max_looking_distance`               | double | [m] 軌道上のゼロ速度の最大探索距離                                                                                  |
| `reaction_params.search_entity_params.search_radius`                        | double | [m] スポーンされたエンティティの検索半径。自己位置とエンティティのポーズとの距離。                                                           |
| `reaction_chain`                                                            | 構造体 | トピックとそのトピックのメッセージ型を含むノードのリスト。                                                                                |

## 制限事項

- Reaction analyzerには`PublisherMessageType`、 `SubscriberMessageType`、 `ReactionType`などいくつかの制限があります。現時点では、次のリストをサポートしています。

- **パブリッシャーメッセージタイプ:**

  - `sensor_msgs/msg/PointCloud2`
  - `sensor_msgs/msg/CameraInfo`
  - `sensor_msgs/msg/Image`
  - `geometry_msgs/msg/PoseWithCovarianceStamped`
  - `sensor_msgs/msg/Imu`
  - `autoware_vehicle_msgs/msg/ControlModeReport`
  - `autoware_vehicle_msgs/msg/GearReport`
  - `autoware_vehicle_msgs/msg/HazardLightsReport`
  - `autoware_vehicle_msgs/msg/SteeringReport`
  - `autoware_vehicle_msgs/msg/TurnIndicatorsReport`
  - `autoware_vehicle_msgs/msg/VelocityReport`

- **サブスクライバーメッセージタイプ:**

  - `sensor_msgs/msg/PointCloud2`
  - `autoware_perception_msgs/msg/DetectedObjects`
  - `autoware_perception_msgs/msg/TrackedObjects`
  - `autoware_perception_msgs/msg/PredictedObject`
  - `autoware_planning_msgs/msg/Trajectory`
  - `autoware_control_msgs/msg/Control`

- **リアクションタイプ:**
  - `FIRST_BRAKE`
  - `SEARCH_ZERO_VEL`
  - `SEARCH_ENTITY`

## 今後の改善

- リアクションタイプを追加することで、リアクションアナライザーを改善できます。現時点でサポートされているリアクションタイプは`FIRST_BRAKE`、 `SEARCH_ZERO_VEL`、 `SEARCH_ENTITY`のみです。各メッセージタイプに対してリアクションタイプを追加することで、拡張できます。

