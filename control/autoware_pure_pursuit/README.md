# Pure Pursuit Controller

Pure Pursuit Controllerモジュールは、Pure Pursuitアルゴリズムを使用して目標軌道を追従するためのステアリング角を計算します。これは`autoware_trajectory_follower_node`のラテラルコントローラプラグインとして使用されます。

## 入力

[controller_node](../autoware_trajectory_follower_node/README.md)から以下を設定します。

- `autoware_planning_msgs/Trajectory` : 追従する目標軌道
- `nav_msgs/Odometry`: 現在の自車位置と速度情報

## 出力

コントローラノードに次の項目を含むLateralOutputを返します。

- `autoware_control_msgs/Lateral`: 目標ステアリング角
- LateralSyncData
  - ステアリング角収束
- `autoware_planning_msgs/Trajectory`: 自車の予測パス

