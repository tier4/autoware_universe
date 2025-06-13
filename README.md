Gion ECU 設定については↓を参照
https://github.com/tier4/pilot-auto.xx1/tree/feature/Gion-v0.46.0

ここでは Phase1 ECU1 での動作を前提とした launch 改造を行っています。
ECU1 ではアクセラレータ移植想定として分離したノード以外のすべてのノードを実行します。
詳細は[関連confluence](https://tier4.atlassian.net/wiki/spaces/SY/pages/3485926525/Phase1+Sensing+Localization) などをして確認ください。
[Phase1 ECU2](https://github.com/tier4/autoware_universe/tree/feature-ecu2-node) とノード分割連携動作の挙動となります。

Phase1 ECU1 ノード分割を行う場合、Autoware launch 時に下記を追加してください。追加しない場合、ECU単体の通常動作となります。
[phase1_acc_trial=true]

ECU2 上で Autoweare launch を実行する際のコマンド
```bash
$ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-rosbag vehicle_model:=sample_vehicle sensor_model:=aip_xx1 phase1_acc_trial:=true
```

以下元情報から変更なし
---

# Autoware Universe

## Welcome to Autoware Universe

Autoware Universe serves as a foundational pillar within the Autoware ecosystem, playing a critical role in enhancing the core functionalities of autonomous driving technologies.
This repository is a pivotal element of the Autoware Core/Universe concept, managing a wide array of packages that significantly extend the capabilities of autonomous vehicles.

![autoware_universe_front](docs/assets/images/autoware_universe_front.png)

## Getting Started

To dive into the vast world of Autoware and understand how Autoware Universe fits into the bigger picture, we recommend starting with the [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/). This resource provides a thorough overview of the Autoware ecosystem, guiding you through its components, functionalities, and how to get started with development.

### Explore Autoware Universe documentation

For those looking to explore the specifics of Autoware Universe components, the [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/), deployed with MKDocs, offers detailed insights.
