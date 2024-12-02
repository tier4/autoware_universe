# autoware_carla_interface

## ROS 2/Autoware.universe と CARLA シミュレータのブリッジ

ROS 2 Humble での CARLA 通信に対する <https://github.com/gezp> に感謝します。
この ROS パッケージは、自律運転シミュレーションのために、Autoware と CARLA の間の通信を可能にします。

## サポート環境

| Ubuntu | ROS | CARLA | Autoware |
| :---: | :---: | :---: | :-----: |
| 22.04 | Humble | 0.9.15 | Main |

## 設定

### インストール

- [CARLAのインストール](https://carla.readthedocs.io/en/latest/start_quickstart/)
- [Carla Lanelet2マップ](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)
- [CARLA 0.9.15のROS 2 Humble通信用のPythonパッケージ](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04)

  - Pipを使用してwheelをインストールします。
  - または`PYTHONPATH`にeggファイルを追加します。

1. マップ（y軸が反転したバージョン）を任意の場所にダウンロードします。
2. 名前を変更し、`autoware_map`内にマップフォルダーを作成します（例:Town01）。（`point_cloud/Town01.pcd`->`autoware_map/Town01/pointcloud_map.pcd`, `vector_maps/lanelet2/Town01.osm`->`autoware_map/Town01/lanelet2_map.osm`）
3. フォルダ上に`map_projector_info.yaml`を作成し、最初の行に`projector_type: local`を追加します。

### ビルド


```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 実行

1. 必要に応じてカーラを実行し、マップを変更し、オブジェクトをスポーンします。<!--- cspell:ignore prefernvidia -->


   ```bash
   cd CARLA
   ./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
   ```

2. ROSノードを実行する


   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/Town01 vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit simulator_type:=carla carla_map:=Town01
   ```

### Inner-workings / アルゴリズム

`InitializeInterface` クラスは、CARLA のワールドと自動運転車の両方の設定に不可欠です。それは `autoware_carla_interface.launch.xml` を通じて構成パラメータを取得します。

メインのシミュレーションループは `carla_ros2_interface` クラス内で実行されます。このループは、`fixed_delta_seconds` 時間で CARLA シミュレータ内のシミュレーション時間を刻み、データは `self.sensor_frequencies` で定義された周波数で ROS 2 メッセージとして受信および発行されます。

Autoware からの自動運転車の指令は `autoware_raw_vehicle_cmd_converter` を通じて処理され、それらの指令は、CARLA に適するよう調整されます。その後、調整された指令が `CarlaDataProvider` 経由で直接 CARLA 制御に入力されます。

### ワールド読み込みの構成パラメータ

すべての重要なパラメータは `autoware_carla_interface.launch.xml` で構成できます。

| 名称                      | タイプ   | デフォルト値                                                                                             | 説明                                                                                                                                                                                                         |
| ------------------------- | ------ | --------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `host`                    | string | "localhost"                                                                                                | CARLA サーバーのホスト名                                                                                                                                                                                       |
| `port`                    | int    | "2000"                                                                                                  | CARLA サーバーのポート番号                                                                                                                                                                                    |
| `timeout`                 | int    | 20                                                                                                       | CARLA クライアントのタイムアウト                                                                                                                                                                                        |
| `ego_vehicle_role_name`   | string | "ego_vehicle"                                                                                            | 自動運転車のロール名                                                                                                                                                                                       |
| `vehicle_type`            | string | "vehicle.toyota.prius"                                                                                   | スポーンする車両のブループリント ID。車両のブループリント ID は [CARLA ブループリント ID](https://carla.readthedocs.io/en/latest/catalogue_vehicles/) で確認できます。                                                  |
| `spawn_point`             | string | None                                                                                                     | 自動運転車のスポーン座標 (None の場合はランダム)。フォーマット = [x, y, z, roll, pitch, yaw]                                                                                                                     |
| `carla_map`               | string | "Town01"                                                                                                 | CARLA にロードするマップ名                                                                                                                                                                                    |
| `sync_mode`               | bool   | True                                                                                                     | CARLA で同期モードを設定するブールフラグ                                                                                                                                                                       |
| `fixed_delta_seconds`     | double | 0.05                                                                                                     | シミュレーションのタイムステップ (クライアントの FPS に関連)                                                                                                                                                                |
| `objects_definition_file` | string | "$(find-pkg-share autoware_carla_interface)/objects.json"                                                 | CARLA でセンサーをスポーンするために使用するセンサーパラメータファイル                                                                                                                                                   |
| `use_traffic_manager`     | bool   | True                                                                                                     | CARLA でトラフィックマネージャーを設定するブールフラグ                                                                                                                                                                        |
| `max_real_delta_seconds`  | double | 0.05                                                                                                     | シミュレーション速度を `fixed_delta_seconds` 未満に制限するパラメーター                                                                                                                                                 |
| `config_file`             | string | "$(find-pkg-share autoware_carla_interface)/raw_vehicle_cmd_converter.param.yaml"                        | `autoware_raw_vehicle_cmd_converter` で使用される制御マッピングファイル。現在の制御は CARLA の `vehicle.toyota.prius` ブループリント ID に基づいて調整されています。車両タイプを変更する場合は、再調整が必要になる場合があります。 |

### センサーの設定可能パラメータ

以下のパラメータは `carla_ros.py` で設定できます。

| 名前                     | タイプ | デフォルト値                                                                          | 説明                                                                                                                                                                                                                           |
| ------------------------- | ---- | -------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `self.sensor_frequencies` | dict | {"top": 11, "left": 11, "right": 11, "camera": 11, "imu": 50, "status": 50, "pose": 2} | (67行目) 前回の公開以降の時間間隔を計算し、この間隔が目的の周波数を超過しないようにするために必要な最小値を満たしているかどうかをチェックします。ROSの公開周波数にのみ影響し、CARLAセンサーのチックには影響しません。 |

- CARLAセンサーパラメータは`config/objects.json`で設定できます。
  - CARLAで修正可能なパラメータの詳細については、[Carla Ref Sensor](https://carla.readthedocs.io/en/latest/ref_sensors/)で説明されています。

### Worldのロード

`carla_ros.py`はCARLAワールドを設定します。

1. **クライアント接続**:


   ```python
   client = carla.Client(self.local_host, self.port)
   client.set_timeout(self.timeout)
   ```

2. **地図の読み込み**:

   `carla_map` パラメーターに従って、CARLA ワールドに地図が読み込まれます。


   ```python
   client.load_world(self.map_name)
   self.world = client.get_world()
   ```

3. **自車配置**:

   車両は、`vehicle_type`, `spawn_point`, `agent_role_name` のパラメータに基づいて配置されます。


   ```python
   spawn_point = carla.Transform()
   point_items = self.spawn_point.split(",")
   if len(point_items) == 6:
      spawn_point.location.x = float(point_items[0])
      spawn_point.location.y = float(point_items[1])
      spawn_point.location.z = float(point_items[2]) + 2
      spawn_point.rotation.roll = float(point_items[3])
      spawn_point.rotation.pitch = float(point_items[4])
      spawn_point.rotation.yaw = float(point_items[5])
   CarlaDataProvider.request_new_actor(self.vehicle_type, spawn_point, self.agent_role_name)
   ```

## 信号認識

Carla Simulator で提供されているマップ ([Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)) には、現時点で Autoware 用の適切な信号コンポーネントがなく、点群マップと比較して緯度と経度の座標が異なります。信号認識を有効にするには、次の手順に従ってマップを変更します。

- マップを変更するためのオプション

  - A. 新しいマップを一から作成する
  - [Tier4 Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) を使用して新しいマップを作成します。

  - B. 既存の Carla Lanelet2 マップを変更する
  - [Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/) の経度と緯度を PCD (原点) に一致するように調整します。
    - この[ツール](https://github.com/mraditya01/offset_lanelet2/tree/main) を使用して座標を変更します。
    - PCD で Lanelet をスナップし、[Tier4 Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) を使用して信号を追加します。

- Tier4 Vector Map Builder を使用する場合、PCD 形式を `binary_compressed` から `ascii` に変換する必要があります。この変換には `pcl_tools` を使用できます。
- 参考までに、1 つの交差点に信号を追加した Town01 の例を[こちら](https://drive.google.com/drive/folders/1QFU0p3C8NW71sT5wwdnCKXoZFQJzXfTG?usp=sharing) からダウンロードできます。

## ヒント

- 初期化中に配置のずれが発生する可能性があります。`init by gnss` ボタンを押すと修正されます。
- `fixed_delta_seconds` を変更すると、シミュレーションのティックを増やすことができます (デフォルトは 0.05 秒)。これを変更すると、`objects.json` のセンサーのいくつかのパラメータを調整する必要があります (例: LIDAR の回転周波数は FPS と一致する必要があります)。

## 既知の問題と今後の作業

- 手続き型マップ (Adv Digital Twin) でのテスト。
  - 現在、Adv デジタルツインマップの作成に失敗するため、テストできません。
- Autoware Sensor Kit からの CARLA センサーの自動設定。
  - 現在、センサーは Autoware Sensor Kit と同じ場所に自動的に設定されていません。現在の回避策は、base_link に対して (0, 0, 0, 0, 0, 0) の座標を持つ各センサーの新しいフレームを作成し、各センサーを新しいフレーム (`autoware_carla_interface.launch.xml` の行 28) にアタッチすることです。この回避策は非常に制限的で制約があり、Sensor Kit が変更されるとセンサーの位置が誤ってアタッチされます。
- 信号認識。
  - 現在、CARLA の HD マップには、Autoware の信号認識に必要な信号に関する情報がありません。

