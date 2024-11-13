## スピードバンプ

### 役割

このモジュールは、速度バンプ規制要素が参照している場合、パスの関連部分の速度を計画します。

![speed_bump_design.svg](docs/speed_bump_design.svg)

### 起動タイミング

マネージャーは、基準パスを参照する速度バンプ規制要素がある場合に、速度バンプシーンモジュールを起動します。

### モジュールパラメーター

| パラメーター        | 型     | 説明                                            |
| ------------------- | ------ | ----------------------------------------------- |
| `slow_start_margin` | double | 車両が減速帯手前で減速するための [m] マージン   |
| `slow_end_margin`   | double | 車両が減速帯通過後に加速するための [m] マージン |
| `print_debug_info`  | bool   | デバッグ情報を印刷するか否か                    |

#### 速度計算

- 段差の高さおよび減速速度の限界を使用して一次方程式を作成する

| パラメータ   | 型     | 説明                                 |
| ------------ | ------ | ------------------------------------ |
| `min_height` | double | [m] スピードバンプの低さの最低想定値 |
| `max_height` | double | [m] スピードバンプの高さの最高想定値 |
| `min_speed`  | double | [m/s] 減速速度の最低想定値           |
| `max_speed`  | double | [m/s] 減速速度の最高想定値           |

### 内部動作 / アルゴリズム

- lanelet2 マップから経路上にある減速帯規制要素を取得する
- 規制要素で指定された `speed_bump_height` に対する `slow_down_speed` を計算する、または減速帯アノテーションから `slow_down_speed` タグを読み取る（存在する場合）

![speed_bump_vel_calc](docs/speed_bump_vel_calc.png)

**注:** 減速帯アノテーションで `slow_down_speed` タグが使用されている場合、減速帯の高さに対する速度の計算は無視されます。その場合、**[kph]** 単位で指定された `slow_down_speed` 値が使用されます。

- 経路と減速帯ポリゴンの交差点を取得する
- 交差点を基に `slow_start_point` と `slow_end_point` を計算し、それらを経路に挿入する
- `slow_start_point` または `slow_end_point` が与えられた（または計算された）オフセット値で挿入できない場合、任意の経路点が仮想的に `slow_start_point` または `slow_end_point` に割り当てられるかどうかを確認する

![speed_bump_scenarios.svg](docs/speed_bump_scenarios.svg)

- `slow_start_point` または `slow_end_point` 間の経路点に `slow_down_speed` を割り当てる

### 今後の作業

- [こちら](https://journals.sagepub.com/doi/10.1155/2014/736576) の記事では、バンプのモデリング手法が提案されています。これは、バンプを円に当てはめて、半径を計算するという簡単なものです。最近のインプリメンテーションでは、速度の計算はバンプの高さだけに基づいていますが、将来はより現実的な結果を得るために、この手法を適用する予定です。
