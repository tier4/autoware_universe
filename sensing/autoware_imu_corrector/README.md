# autoware_imu_corrector

## imu_corrector

`imu_corrector_node`はIMUデータを補正するノードです。

1. パラメーターを読み取って、ヨーレートオフセット$b$を補正します。
2. パラメーターを読み取って、ヨーレートの標準偏差$\sigma$を補正します。

数学的には、以下の方程式を使用します。

$$
\tilde{\omega}(t) = \omega(t) + b(t) + n(t)
$$

ここで、$\tilde{\omega}$は観測された角速度、$\omega$は真の角速度、$b$はオフセット、$n$はガウス雑音です。また、$n\sim\mathcal{N}(0, \sigma^2)$と想定します。

<!-- TODO(TIER IV): Make this repository public or change the link. -->
<!-- このノードのパラメーターとして、[deviation_estimator](https://github.com/tier4/calibration_tools/tree/main/localization/deviation_estimation_tools)で推定された値を使用してください。 -->

### 入力

| 名前     | 種別                    | 説明  |
| -------- | ----------------------- | ------------ |
| `~input` | `sensor_msgs::msg::Imu` | 生のIMUデータ |

### 出力

自動運転ソフトウェアに関するドキュメントのURL: [https://gitlab.com/autowarefoundation/autoware.auto/autoware/planning](https://gitlab.com/autowarefoundation/autoware.auto/autoware/planning)

**Planningモジュール**

Planningモジュールは、安全で効率的な経路計画と意思決定を実行します。 Planningモジュールは、現在の自車位置と周囲環境の情報を処理し、最適な軌道を生成します。

**機能**

* 障害物回避
* 交通法規遵守
* 経路計画
* 軌跡生成
* `post resampling`
* 障壁回避

**アーキテクチャ**

Planningモジュールは、以下の主要コンポーネントで構成されています。

* **パスプランナー:** 安全で効率的な経路を生成します。
* **モーションプランナー:** 障害物回避と交通法規遵守を処理します。
* **Trajectory Optimizer:** スムーズで実現可能な軌跡を生成します。

**インターフェイス**

Planningモジュールは、以下のインターフェイスを介して他のコンポーネントと通信します。

* **Vehicle Control:** 軌跡を送信し、自車位置を受信します。
* **Perception:** 周囲環境に関する情報を提供します。
* **Localization:** 自車位置と姿勢を推定します。

| 名称      | タイプ                    | 説明        |
| --------- | ----------------------- | ------------------ |
| `~output` | `sensor_msgs::msg::Imu` | 補正済imuデータ |

### パラメータ

| 項目名                      | 型   | 説明                                                  |
| ---------------------------- | ------ | -------------------------------------------------------- |
| `angular_velocity_offset_x`  | double | `imu_link`のロールレートオフセット [rad/s]             |
| `angular_velocity_offset_y`  | double | `imu_link`のピッチレートオフセット [rad/s]               |
| `angular_velocity_offset_z`  | double | `imu_link`のヨーレートオフセット [rad/s]                 |
| `angular_velocity_stddev_xx` | double | `imu_link`のロールレート標準偏差 [rad/s]                |
| `angular_velocity_stddev_yy` | double | `imu_link`のピッチレート標準偏差 [rad/s]                   |
| `angular_velocity_stddev_zz` | double | `imu_link`のヨーレート標準偏差 [rad/s]                   |
| `acceleration_stddev`        | double | `imu_link`の加速度標準偏差 [m/s^2]                    |

## gyro_bias_validator

`gyro_bias_validator`はジャイロスコープのバイアスを確認するノードです。`sensor_msgs::msg::Imu`トピックをサブスクライブして，ジャイロスコープのバイアスが一定の範囲内にあるかどうかを確認します。

ノードは，車両が停止しているときのみジャイロスコープデータの平均値からバイアスを計算することに注意してください。

### 入力

| 名前                      | タイプ                                            | 説明                                  |
| -------------------------- | ----------------------------------------------- | ------------------------------------- |
| `~/input/imu_raw`          | `sensor_msgs::msg::Imu`                         | **生**IMUデータ                              |
| `~/input/pose`             | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt pose                                    |

入力された位姿は十分に正確であると仮定されます。例えばNDTを使用する場合、NDTが適切な収束をしていると仮定します。

現在、Autowareの `pose_source`としてNDT以外のメソッドを使用することは可能ですが、精度が低いメソッドはIMUバイアス推定には適していません。

将来的には、位姿誤差に対する注意深い実装により、NDTによって推定されたIMUバイアスは検証だけでなくオンラインキャリブレーションにも使用できる可能性があります。

### 出力

| 名称                       | タイプ                                 | 説明                           |
| ------------------------ | ------------------------------------ | ------------------------------ |
| `~/output/gyro_bias`     | `geometry_msgs::msg::Vector3Stamped` | ジャイロスコープのバイアス [rad/s] |

### パラメータ

このノードは、`imu_corrector.param.yaml` の `angular_velocity_offset_x`, `angular_velocity_offset_y`, `angular_velocity_offset_z` パラメータも使用することに注意してください。

| 名前                                      | 型     | 説明                                                                                                    |
| ----------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------- |
| `gyro_bias_threshold`                     | double | ジャイロスコープのバイアスのしきい値 [rad/s]                                                          |
| `timer_callback_interval_sec`               | double | タイマークールバック関数の秒数 [sec]                                                                     |
| `diagnostics_updater_interval_sec`          | double | 診断アップデーターの期間 [sec]                                                                          |
| `straight_motion_ang_vel_upper_limit`       | double | 直線モーションでないと思われるヨー角速度の上限 [rad/s]                                                 |

