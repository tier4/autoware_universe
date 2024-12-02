## yabloc_pose_initializer

このパッケージには、初期姿勢推定に関するノードが含まれています。

- [camera_pose_initializer](#camera_pose_initializer)

このパッケージは、実行時に事前トレーニング済みのセマンティックセグメンテーションモデルを必要とします。このモデルは通常、[インストール](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)の環境準備フェーズ中に`ansible`によってダウンロードされます。手動でダウンロードすることも可能です。モデルがダウンロードされていなくても初期化は完了しますが、精度は低下する可能性があります。

モデルを手動でダウンロードして解凍するには、次の手順に従います。


```bash
$ mkdir -p ~/autoware_data/yabloc_pose_initializer/
$ wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
$ tar xzf ~/autoware_data/yabloc_pose_initializer/resources.tar.gz -C ~/autoware_data/yabloc_pose_initializer/
```

## 注記

このパッケージは外部コードを使用します。トレーニング済みファイルはアポロによって提供されます。トレーニング済みファイルは環境準備中に自動的にダウンロードされます。

元モデルのURL

<https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/road-segmentation-adas-0001>

> Open Model ZooはApache Licenseバージョン2.0のライセンスに基づいています。

変換されたモデルのURL

<https://github.com/PINTO0309/PINTO_model_zoo/tree/main/136_road-segmentation-adas-0001>

> モデル変換スクリプトはMITライセンスに基づいてリリースされています。

## 特別な感謝

- [openvinotoolkit/open_model_zoo](https://github.com/openvinotoolkit/open_model_zoo)
- [PINTO0309](https://github.com/PINTO0309)

## camera_pose_initializer

### 目的

- このノードは、ADAPIの要求に応じてカメラを使用して初期位置を推定します。

#### 入力

| 名                               | タイプ                                    | 説明              |
| --------------------------------- | --------------------------------------- | ------------------------ |
| `input/camera_info`                 | `sensor_msgs::msg::CameraInfo`          | 変形解除カメラ情報  |
| `input/image_raw`                   | `sensor_msgs::msg::Image`               | 変形解除カメラ画像 |
| `input/vector_map`                  | `autoware_map_msgs::msg::LaneletMapBin` | vector map               |

## 自動運転ソフトウェア

### Plan.ioアプリケーション

#### Planningモジュール

- PlannerはHolomapから検出した障害物を考慮のうえ走行経路を計画します。

#### 経路生成

- **Precision**
    - Holomapのデータに基づき高精度経路を生成します。
- **Adaptive**
    - センサーデータに基づき低遅延の経路を生成します。
- **Safety**
    - 安全を確保する経路を生成します。

#### Planning Framework

- Planning Frameworkは経路生成を制御し、以下のような機能を提供します。
    - Plannerの選択
    - Planning/制御間の同期
    - `post resampling`
    - 車線維持

#### Planningコンポーネント

- **PlannerManager:** プランナーの管理
- **TrajectoryManager:** 経路の作成と管理
- **AdaptivePlanner:** Holomapの更新を考慮した経路の再生成
- **LanePlanner:** 車線維持
- **SafetyPlanner:** 安全性確保

#### システムアーキテクチャ

![Autoware Plan.ioアーキテクチャ](architecture.png)

### Claudeアプリケーション

#### システムアーキテクチャ

![Autoware Claudeアーキテクチャ](architecture.png)

#### Mappingモジュール

- **Mapper:** Waypointマップを生成・管理します。
- **Lidar Mapping:** 3D点群データから高精度マップを生成します。

#### 自車位置推定モジュール

- **Localization:** ClaudeはHolomapやセンサーデータを活用して自車位置を推定します。

#### オブジェクト認識モジュール

- **Detector:** Claudeは車両や歩行者などのダイナミックオブジェクトを検出します。
- **Tracker:** ダイナミックオブジェクトを追跡します。

#### センシングモジュール

- **Sensor Manager:** センサーの管理とデータの統合
- **Lidar:** 高精度3D点群データを取得します。
- **Camera:** 画像データを取得します。
- **Radar:** ドップラー情報を取得します。

| 名前                | タイプ                                   | 説明                    |
| ------------------- | -------------------------------------- | ----------------------- |
| `output/candidates` | `visualization_msgs::msg::MarkerArray` | 初期姿勢候補            |

### パラメータ

{{ json_to_markdown("localization/yabloc/yabloc_pose_initializer/schema/camera_pose_initializer.schema.json") }}

### Services

| 名前               | 型                                                      | 説明                     |
| ------------------ | --------------------------------------------------------- | ------------------------------- |
| `yabloc_align_srv` | `tier4_localization_msgs::srv::PoseWithCovarianceStamped` | 初期位置推定リクエスト |

