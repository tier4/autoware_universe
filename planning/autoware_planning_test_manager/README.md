# Autoware Planning Test Manager

## 背景

プランニングモジュールの各ノードでは、異常な経路や自車位置が大きく逸脱するなど、想定外の入力が与えられると、ノードはそのような入力に対応できずにクラッシュする可能性があります。その結果、ノードクラッシュのデバッグには時間がかかる場合があります。たとえば、空の軌跡が入力として与えられ、実装時にそれが予想されていなかった場合、PRマージ時、シナリオテスト時、またはシステムが実際の車両で実行中に、処理されない例外入力によりノードがクラッシュする可能性があります。

## 目的

例外的な入力が受信されたときにノードが正しく動作することを保証するテストを実装するためのユーティリティを提供することです。このユーティリティを利用して例外的な入力のテストを実装することで、PRマージ前に例外的な入力に対する対策を要求することで、システムを実際に実行するときにのみ発見されるバグを削減することを目的としています。

## 機能

### 通常動作の確認

テスト対象ノードに対して、ノードが正しく動作し、後続のノードに必要なメッセージを公開していることを確認します。これを行うには、test_node が必要なメッセージを公開し、ノードの出力が公開されていることを確認します。

### 特殊入力に対する堅牢性の確認

正常動作を確認した後、テスト対象ノードが例外的な入力を与えられてもクラッシュしないことを確認します。これを行うには、test_node から例外的な入力を提供し、ノードがクラッシュしないことを確認します。

（未完了）

## 使用方法


```cpp

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  // instantiate test_manager with PlanningInterfaceTestManager type
  auto test_manager = std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  // get package directories for necessary configuration files
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto target_node_dir =
    ament_index_cpp::get_package_share_directory("target_node");

  // set arguments to get the config file
  node_options.arguments(
    {"--ros-args", "--params-file",
     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     autoware_planning_validator_dir + "/config/planning_validator.param.yaml"});

  // instantiate the TargetNode with node_options
  auto test_target_node = std::make_shared<TargetNode>(node_options);

  // publish the necessary topics from test_manager second argument is topic name
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "velocity_smoother/input/external_velocity_limit_mps");

  // set scenario_selector's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("input/parking/trajectory");

  // test with normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  // make sure target_node is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with trajectory input with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(test_target_node));

  // shutdown ROS context
  rclcpp::shutdown();
}
```

## 実施中のテスト

| ノード                        | テスト名                                                                                | 特殊入力 | 出力         | 特殊入力パターン                                                             |
| --------------------------- | ----------------------------------------------------------------------------------------- | --------- | ------------ | ------------------------------------------------------------------------------------- |
| autoware_planning_validator | NodeTestWithExceptionTrajectory                                                           | trajectory | trajectory     | 空、単一ポイント、重複ポイントを持つパス                                       |
| velocity_smoother           | NodeTestWithExceptionTrajectory                                                           | trajectory | trajectory     | 空、単一ポイント、重複ポイントを持つパス                                       |
| obstacle_cruise_planner     | NodeTestWithExceptionTrajectory                                                           | trajectory | trajectory     | 空、単一ポイント、重複ポイントを持つパス                                       |
| obstacle_stop_planner       | NodeTestWithExceptionTrajectory                                                           | trajectory | trajectory     | 空、単一ポイント、重複ポイントを持つパス                                       |
| obstacle_velocity_limiter   | NodeTestWithExceptionTrajectory                                                           | trajectory | trajectory     | 空、単一ポイント、重複ポイントを持つパス                                       |
| path_optimizer              | NodeTestWithExceptionTrajectory                                                           | trajectory | trajectory     | 空、単一ポイント、重複ポイントを持つパス                                       |
| scenario_selector           | NodeTestWithExceptionTrajectoryLaneDrivingMode NodeTestWithExceptionTrajectoryParkingMode | trajectory | scenario       | 空、単一ポイント、重複ポイントを持つパス（シナリオ:LANEDRIVINGとPARKING） |
| freespace_planner           | NodeTestWithExceptionRoute                                                                | route       | trajectory     | 空のルート                                                                            |
| behavior_path_planner       | NodeTestWithExceptionRoute NodeTestWithOffTrackEgoPose                                    | route       | route odometry | 空のルート、オフレーン自車位置                                                   |
| behavior_velocity_planner   | NodeTestWithExceptionPathWithLaneID                                                       | path_with_l | path           | 空のパス                                                                            |

## 重要な注意点

テストの実行中にノードを起動すると、パラメーターは各パッケージ内のパラメーターファイルから読み込まれます。そのため、パラメーターを追加する場合、ノード起動時にパラメーターファイルからパラメーターを取得できない場合にノードが起動できなくなることを防ぐため、対象のノードパッケージ内のパラメーターファイルに必要なパラメーターを追加する必要があります。

## 今後の拡張 / 未実装部分

（未定）

