# 可動物体の回避モジュール

このモジュールは開発中です。

## 目的 / 役割

このモジュールは、Autowareの[autoware_path_optimizer](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_path_optimizer/)と連携して、自車の経路周辺の車両、歩行者、障害物を回避する機能を提供します。
それぞれのモジュールは次の役割を果たします。
動的回避モジュールは、回避対象物の位置と速度に応じて走行可能領域を切り取ります。
障害物回避モジュールは、受信した走行可能領域に収まるように従うべき経路を変更します。

静的障害物の回避機能は[Static Avoidance module](https://autowarefoundation.github.io/autoware.universe/main/planning/autoware_behavior_path_static_obstacle_avoidance_module/)でも提供されますが、これらのモジュールには役割の違いがあります。
スタティック障害回避モジュールは独自の車線を外部から回避しますが、可動物体は回避できません。
一方、このモジュールは可動物体を回避できます。
このため、モジュール名に「動的」という単語が使用されています。
次の表は、各状況に対処できる回避モジュールを示しています。

|                          |                   車線内回避                   |          車線外通過による回避          |
| :----------------------- | :-------------------------------------------------------------------------: | :---------------------------------------: |
| 静止物回避              | Avoidance Module <br> Dynamic Avoidance Module + Obstacle Avoidance Module |            Avoidance Module            |
| 動的物回避              |            Dynamic Avoidance Module + Obstacle Avoidance Module            |     No Module (Under Development)      |

## アルゴリズムのポリシー

ここでは、内部アルゴリズムのポリシーについて説明します。
内部アルゴリズムは2つの部分に分けられます。最初の部分は障害物を回避するかを決定し、2番目の部分は回避する障害物に対する走行可能領域を切り取ります。

### 回避する障害物を選択

オブジェクトを回避するかどうかを決定するには、予測パスと各オブジェクトの状態（ポーズとツイスト）の両方が使用されます。
ユーザーがこのモジュールに回避してほしいタイプのオブジェクトも必要です。
この情報を使用して、モジュールは「自車位置の通過を妨げる」かつ「回避可能な」オブジェクトを_回避_するように決定します。

「自車位置の通過を妨げる」の定義は、数秒以内に衝突するオブジェクトとして実装されています。
もう一方の「回避可能な」は、乗客や他の車両を危険にさらすことなく回避できるかどうかを示します。
このため、モジュールは横加速度と横ジャークの制約を満たして障害物を回避できるかどうかを判断します。
たとえば、モジュールは横方向に近すぎるか速度が速すぎるオブジェクトを回避しないことを決定します。

### 選択した車両に対して走行可能領域を切り取り

回避するべき選択された障害物に対して、モジュールは走行可能領域を切り取ります。
カットオフ多角形の形状を決定する入力として、主に予測パスではなく、障害物のポーズが使用され、障害物が自車位置のパスに平行に移動すると仮定します。
この設計は、オブジェクトの予測パスがパス修正を使用するには十分に正確ではない（少なくとも現時点では）という点に起因しています。
さらに、出力の走行可能領域の形状は、平面ではなく自車位置のパスに沿った長方形の切り抜きとして設計されており、計算を平面でなくスカラーで行います。

#### 横方向の寸法の決定

多角形の横方向の寸法は次のように計算されます。
走行可能領域から抽出する多角形の幅は障害物の幅と`drivable_area_generation.lat_offset_from_obstacle`です。
横方向のシフト長は`drivable_area_generation.max_lat_offset_to_avoid`で制限できます。

![drivable_area_extraction_width](./image/drivable_area_extraction_width.drawio.svg)

#### 縦方向の寸法の決定

次に、走行可能領域から同じ方向および逆方向の障害物を抽出することは、TTC（衝突時間）を考慮して次のように機能します。

同じ方向の障害物に関して、TTCが負の障害物は無視されます（例：障害物が自車位置の前にあり、障害物の速度が自車位置の速度より大きい）。

同じ方向の障害物（パラメータ名は実装によって異なる場合があります）
![same_directional_object](./image/same_directional_object.svg)

逆方向の障害物（パラメータ名は実装によって異なる場合があります）
![opposite_directional_object](./image/opposite_directional_object.svg)

### 選択した歩行者に対して走行可能領域を切り取り

次に、回避するべき歩行者に対して走行可能領域を生成するロジックについて説明します。
このタイプのオブジェクトは、自車位置の車両よりも優先権を持ち、自車位置の車両の最低限の安全性を確保していると考えられます。
言い換えれば、モジュールは障害物に次の図のように特定の時間に特定の信頼度で予測されたパスに基づいて特定のマージンのある走行可能領域を割り当てます。

<figure>
    <img src="./image/2024-04-18_15-13-01.png" width="600">
    <figcaption>各歩行者の予測パスから制限領域が生成されます</figcaption>
</figure>

オブジェクトの多角形とは別に、モジュールは自車位置の安全性を確保するため、つまり急激な操舵やパスからの大幅な変更を避けるために別の多角形も生成します。
これは車両に対する回避と似ており、回避するオブジェクトからの安全距離を確保することよりも優先されます。
その結果、下の図に示すように、自車位置の安全な多角形によって減少したオブジェクト周辺の多角形が自車位置の走行可能領域から差し引かれます。

<figure>

## 例

<figure>
    <img src="./image/image-20230807-151945.png" width="800">
    <figcaption>自動運転車両側の最小限の条件を対象物とのマージンに対して優先</figcaption>
</figure>

## 将来の作業

現在、回避経路の移動距離は `drivable_area_generation.max_lat_offset_to_avoid` によって 0.5 メートル以下に制限されています。
これは、他のモジュールで機能するためには十分な機能がなく、Planning コンポーネントの構造が原因です。
この問題により、このモジュールは回避幅が小さい場合にのみ対応できます。
この問題は、このモジュールにとって最も重要です。
さらに、必要に応じて走行可能領域を拡張する機能も必要です。

## パラメーター

開発中

| 名称                                                                   | 単位  | 型   | 説明                                                          | デフォルト値 |
| :--------------------------------------------------------------------- | :---- | :----- | :--------------------------------------------------------- | :------------ |
| target_object.car                                                     | [-]   | bool   | 避けるべき対象物に車が含まれるかどうかのフラグ                       | true          |
| target_object.truck                                                   | [-]   | bool   | 避けるべき対象物にトラックが含まれるかどうかのフラグ                     | true          |
| ...                                                                   | [-]   | bool   | ...                                                           | ...           |
| target_object.min_obstacle_vel                                        | [m/s] | double | 回避すべき障害物の最小速度                                 | 1.0           |
| drivable_area_generation.lat_offset_from_obstacle                     | [m]   | double | 障害物から回避するための横方向オフセット                       | 0.8           |
| drivable_area_generation.max_lat_offset_to_avoid                      | [m]   | double | 避けるための最大横方向オフセット                             | 0.5           |
| drivable_area_generation.overtaking_object.max_time_to_collision      | [s]   | double | 衝突時間計算時の最大値                                     | 3.0           |
| drivable_area_generation.overtaking_object.start_duration_to_avoid    | [s]   | double | 障害物を追い越す前に回避を考慮する時間                         | 4.0           |
| drivable_area_generation.overtaking_object.end_duration_to_avoid      | [s]   | double | 障害物を追い抜いた後に回避を考慮する時間                        | 5.0           |
| drivable_area_generation.overtaking_object.duration_to_hold_avoidance | [s]   | double | 障害物を追い抜いた後に回避を保持する時間                       | 3.0           |
| drivable_area_generation.oncoming_object.max_time_to_collision        | [s]   | double | 衝突時間計算時の最大値                                     | 3.0           |
| drivable_area_generation.oncoming_object.start_duration_to_avoid      | [s]   | double | 障害物に接近する前に回避を考慮する時間                         | 9.0           |
| drivable_area_generation.oncoming_object.end_duration_to_avoid        | [s]   | double | 障害物を回避した後に回避を考慮する時間                        | 0.0           |

