## 交差点信号推定器

## 目的

`crosswalk_traffic_light_estimator` は HDMap と検出された車両信号から歩行者用信号を推定するモジュールです。

## 入出力

### 入力
- `HDMap`
- `DetectedVehicleTrafficLights`

### 出力
- `crosswalk_traffic_light`

| 名前 | タイプ | 説明 |
| --- | --- | --- |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | ベクターマップ |
| `~/input/route` | `autoware_planning_msgs::msg::LaneletRoute` | ルート |
| `~/input/classified/traffic_signals` | `tier4_perception_msgs::msg::TrafficSignalArray` | 分類信号 |

### 出力

**自動運転ソフトウェアドキュメント**

**Planningコンポーネント**

**概要**

Planningコンポーネントは、自車位置や障害物などのセンサデータから、車両の経路計画と制御を行います。

**機能**

* **経路計画:** 安全かつ快適な経路を生成します。
* **衝突回避:** 障害物と衝突することを回避する回避行動を生成します。
* **速度制御:** 速度を制限と障害物に応じて調整します。
* **ステアリング制御:** 目標経路に従ってステアリング角を計算します。

**アーキテクチャ**

Planningコンポーネントは、以下のモジュールで構成されます。

* **経路生成モジュール**
* **衝突回避モジュール**
* **速度制御モジュール**
* **ステアリング制御モジュール**

**依存関係**

Planningコンポーネントは、以下のデータに依存します。

* **センサデータ:** 自車位置、障害物、レーンマーカーなど
* **地図データ:** 道路ネットワーク、規制情報など

**パラメータ**

Planningコンポーネントには、以下のパラメータを設定できます。

* **安全マージン:** 障害物との安全距離
* **加速限界:** 加速度の上限
* **旋回半径:** 旋回可能な最小半径

**推奨使用法**

Planningコンポーネントは、Autoware FCWやAutoware FDWなどの自动運転システムに組み込んで使用できます。

**リファレンス**

* [Autoware GitHubリポジトリ](https://github.com/AutowareFoundation/autoware.ai)
* [Autoware ドキュメンテーション](https://autoware.docs.openrobotics.org/)

| 名前                        | タイプ                                                   | 説明                                               |
| -------------------------- | --------------------------------------------------------- | --------------------------------------------------------- |
| `~/output/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 歩行者の交通信号の推定値を出力                        |

## パラメータ

| 名称                        | タイプ     | 説明                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | デフォルト値 |
| :---------------------------- | :------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `use_last_detect_color`       | `bool`   | このパラメータが `true` の場合、このモジュールは、車両の信号が GREEN/AMBER として検出された場合だけでなく、検出結果が GREEN/AMBER から UNKNOWN に変化した場合も、歩行者の交通信号を RED と推定します。 (検出結果が RED または AMBER から UNKNOWN に変化した場合、このモジュールは歩行者の交通信号を UNKNOWN と推定します。) このパラメータが `false` の場合、このモジュールは推定に最新の検出結果のみを使用します。 (検出結果が GREEN/AMBER の場合のみ、このモジュールは歩行者の交通信号を RED と推定します。) | `true`        |
| `last_detect_color_hold_time` | `double` | 最後に検出された色の保持時間しきい値。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | `2.0`         |

## 内部機構 / アルゴリズム


```plantuml

start
:subscribe detected traffic signals & HDMap;
:extract crosswalk lanelets from HDMap;
:extract road lanelets that conflicts crosswalk;
:initialize non_red_lanelets(lanelet::ConstLanelets);
if (Latest detection result is **GREEN** or **AMBER**?) then (yes)
  :push back non_red_lanelets;
else (no)
  if (use_last_detect_color is **true**?) then (yes)
    if (Latest detection result is **UNKNOWN** and last detection result is **GREEN** or **AMBER**?) then (yes)
     :push back non_red_lanelets;
    endif
  endif
endif
if (Is there **STRAIGHT-NON-RED** road lanelet in non_red_lanelets?) then (yes)
  :estimate related pedestrian's traffic signal as **RED**;
else if (Is there both **LEFT-NON-RED** and **RIGHT-NON-RED** road lanelet in non_red_lanelets?) then (yes)
  :estimate related pedestrian's traffic signal as **RED**;
else (no)
  :estimate related pedestrian's traffic signal as **UNKNOWN**;
endif
end

```

## 歩行者と車両の通行が信号機で管理されている場合、歩行者が横断しないように、歩行者信号機を **赤** にすることがあります。その条件は次のとおりです。

### ケース 1

- 歩道が **直進** レーンレットと交差する
- レーンレットが **緑** または **黄** の信号機を参照する (次の図は **緑** の場合のみを示します)

<div align="center">
  <img src="images/straight.drawio.svg" width=80%>
</div>
<div align="center">
  <img src="images/intersection1.svg" width=80%>
</div>

### ケース 2

- 歩道が別の曲がり角のレーンレット (直進と左折、左折と右折、右折と直進) と交差する
- レーンレットが **緑** または **黄** の信号機を参照する (次の図は **緑** の場合のみを示します)

<div align="center">
  <img src="images/intersection2.svg" width=80%>
</div>

## 想定事項/既知の制約事項

## 将来の拡張機能/未実装の部分

