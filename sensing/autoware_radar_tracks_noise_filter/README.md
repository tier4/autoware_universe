# autoware_radar_tracks_noise_filter

このパッケージには、`radar_msgs/msg/RadarTrack`用レーダーオブジェクトフィルタモジュールが含まれています。
このパッケージは、RadarTracks内のノイズオブジェクトをフィルタリングできます。

## アルゴリズム

このパッケージのコアアルゴリズムは`RadarTrackCrossingNoiseFilterNode::isNoise()`関数です。
詳細は関数とそのパラメータを参照してください。

- Y軸のしきい値

レーダーはドップラー速度としてx軸速度を検出できますが、y軸速度を検出することはできません。
一部のレーダーはデバイス内にy軸速度を推定できますが、精度が劣る場合があります。
y軸しきい値フィルタでは、RadarTrackのy軸速度が`velocity_y_threshold`を超える場合、ノイズオブジェクトとして扱います。

## 入力

| 名前            | タイプ                           | 説明                  |
| ---------------- | ------------------------------- | --------------------- |
| `~/input/tracks` | radar_msgs/msg/RadarTracks.msg   | 3D検出された軌跡       |

## アウトプット

このモジュールは、自動運転システム向けのPlanningコンポーネントを提供します。Planningモジュールは、一次元経路生成（``1D Path Generation``）、二次元経路生成（``2D Path Generation``）、経路追従（``Path Following``）などの機能を提供します。

### Planningモジュールの機能

* 安全性の高い一次元経路の生成
* 交通状況や交通規制を考慮した二次元経路の生成
* 車両の自車位置をリアルタイムで考慮した経路の追従
* 経路追従中の障害物検知と回避
* 制御コマンドの出力

### Planningモジュールのインターフェース

Planningモジュールは、次のインターフェースを介して他のAutowareコンポーネントと通信します。

* `Input:`
    * センサーデータ
    * 自車位置
    * 交通状況
* `Output:`
    * 制御コマンド
    * 経路情報

### Planningモジュールの構成

Planningモジュールは、次のサブモジュールで構成されています。

* `1D Path Generation:` 一次元経路生成モジュール
* `2D Path Generation:` 二次元経路生成モジュール
* `Path Following:` 経路追従モジュール
* `Obstacle Detection and Avoidance:` 障害物検知と回避モジュール

### Planningモジュールの使用方法

Planningモジュールを使用するには、次の手順に従います。

1. PlanningモジュールをAutowareシステムに統合します。
2. Planningモジュールに必要な設定を行います。
3. Planningモジュールを実行します。

### Planningモジュールの設定

Planningモジュールの設定は、`config.yaml`ファイルで行います。設定ファイルには、次のパラメータが含まれます。

* `1D Path Generation:` 一次元経路生成モジュールのパラメータ
* `2D Path Generation:` 二次元経路生成モジュールのパラメータ
* `Path Following:` 経路追従モジュールの設定
* `Obstacle Detection and Avoidance:` 障害物検知と回避モジュールの設定

### Planningモジュールの実行

Planningモジュールを実行するには、次のコマンドを使用します。

```
$ roslaunch autoware_planning planning.launch
```

| 名称                       | 種類                           | 説明      |
| -------------------------- | ------------------------------ | --------- |
| `~/output/noise_tracks`    | radar_msgs/msg/RadarTracks.msg | ノイズオブジェクト    |
| `~/output/filtered_tracks` | radar_msgs/msg/RadarTracks.msg | フィルタリングされたオブジェクト |

## パラメータ

| 名前                   | 型   | 説明                                                                                                                              | デフォルト値 |
| :--------------------- | :----- | :------------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| `velocity_y_threshold` | double | Y軸速度しきい値 [m/s]。RadarTrack の Y 軸速度が `velocity_y_threshold` を超える場合、ノイズオブジェクトとして処理されます。 | 7.0           |

