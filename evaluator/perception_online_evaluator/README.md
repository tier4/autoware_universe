# 知覚評価器

知覚システムの出力の評価のためのノード。

## 目的

このモジュールは、注釈なしで知覚結果がいかに正確に生成されているかを評価することを可能にします。性能を確認することができ、数秒前から結果を評価して、オンライン実行を可能にします。

## 内部動作 / アルゴリズム

評価される指標は以下の通りです。

- predicted_path_deviation（予測経路偏差）
- predicted_path_deviation_variance（予測経路偏差分散）
- lateral_deviation（横方向偏差）
- yaw_deviation（ヨー逸脱）
- yaw_rate（ヨーレート）
- total_objects_count（全オブジェクト数）
- average_objects_count（平均オブジェクト数）
- interval_objects_count（インターバルオブジェクト数）

### 予測経路偏差 / 予測経路偏差分散

**移動オブジェクト**の実際の移動経路と過去のオブジェクトの予測経路を比較して、偏差を特定します。オブジェクトごとに、予測経路ポイントと実際の経路上の対応するポイント間の平均距離を、指定されたタイムステップまで計算します。言い換えれば、平均変位誤差（ADE）を計算します。評価対象オブジェクトは $T_N$ 秒前のオブジェクトで、$T_N$ は予測時間範囲 $[T_1, T_2, ..., T_N]$ の最大値です。

> [!NOTE]
> $T_N$ 秒前のオブジェクトは、すべての指標で対象オブジェクトです。これは、指標間で対象オブジェクトの時間を統一するためです。

![path_deviation_each_object](./images/path_deviation_each_object.drawio.svg)

$$
\begin{align}
n_{points} = T / dt \\
ADE = \Sigma_{i=1}^{n_{points}} d_i / n_{points}　\\
Var = \Sigma_{i=1}^{n_{points}} (d_i - ADE)^2 / n_{points}
\end{align}
$$

- $n_{points}$ : 予測経路のポイント数
- $T$ : 予測評価の時間範囲。
- $dt$ : 予測経路の時間間隔
- $d_i$ : 経路ポイント $i$ での予測経路と実際の移動経路間の距離
- $ADE$ : 対象オブジェクトの予測経路の平均偏差。
- $Var$ : 対象オブジェクトの予測経路偏差の分散。

最終的な予測経路偏差指標は、同じクラスのすべてのオブジェクトの予測経路の平均偏差の平均を計算し、次に平均偏差の平均、最大、最小を計算することによって計算されます。

![path_deviation](./images/path_deviation.drawio.svg)

$$
\begin{align}
ADE_{mean} = \Sigma_{j=1}^{n_{objects}} ADE_j / n_{objects} \\
ADE_{max} = max(ADE_j) \\
ADE_{min} = min(ADE_j)
\end{align}
$$

`Var_{max}` = `max(Var_j)` \\
`Var_{min}` = `min(Var_j)`
\end{align}
$$

- `n_{objects}`: オブジェクト数
- `ADE_{mean}`: 全オブジェクトに対する予測経路の平均偏差
- `ADE_{max}`: 全オブジェクトに対する予測経路の最大偏差
- `ADE_{min}`: 全オブジェクトに対する予測経路の最小偏差
- `Var_{mean}`: 全オブジェクトに対する予測経路偏差の平均分散
- `Var_{max}`: 全オブジェクトに対する予測経路偏差の最大分散
- `Var_{min}`: 全オブジェクトに対する予測経路偏差の最小分散

実際のメトリック名は、オブジェクトクラスと時間帯によって決定されます。例: `predicted_path_deviation_variance_CAR_5.00`

### 側方偏差

**移動オブジェクト**の側方位置認識の安定性を評価するために、平滑化された走行経路と認識された位置との間の側方偏差を計算します。平滑化された走行経路は、パラメーター `smoothing_window_size` で指定されたウィンドウサイズを持つ中心移動平均フィルターを適用することで計算されます。側方偏差は、平滑化された走行経路とタイムスタンプが `T=T_n` 秒前の過去のオブジェクトの認識された位置を比較することで計算されます。停止しているオブジェクトの場合、平滑化された走行経路は不安定になるため、このメトリックは計算されません。

![lateral_deviation](./images/lateral_deviation.drawio.svg)

### ヨー偏差

**移動オブジェクト**について、過去のオブジェクトの認識されたヨー角と平滑化された走行経路のヨー方位角との間の偏差を計算します。平滑化された走行経路は、パラメーター `smoothing_window_size` で指定されたウィンドウサイズを持つ中心移動平均フィルターを適用することで計算されます。ヨー偏差は、平滑化された走行経路のヨー方位角とタイムスタンプが `T=T_n` 秒前の過去のオブジェクトの認識された向きを比較することで計算されます。
停止しているオブジェクトの場合、平滑化された走行経路は不安定になるため、このメトリックは計算されません。

![yaw_deviation](./images/yaw_deviation.drawio.svg)

### ヨーレイト

ヨー角の前回タイムステップからの変化に基づいてオブジェクトのヨーレイトを計算します。この値は**静止オブジェクト**に適用され、ヨーレイト認識の安定性を評価します。ヨーレイトは、過去のオブジェクトのヨー角と、前のサイクルで受信したオブジェクトのヨー角を比較することで計算されます。このとき、t2は `T_n` 秒前のタイムスタンプとなります。

![yaw_rate](./images/yaw_rate.drawio.svg)

### オブジェクト数

指定された検出範囲内で各オブジェクトクラスの検出数をカウントします。これらのメトリックは、過去のオブジェクトではなく、最新のオブジェクトに対して測定されます。

![detection_counts](./images/detection_counts.drawio.svg)

提供された図では、範囲 `R` は半径のリスト（例: `r_1, r_2, ...`）と高さのリスト（例: `h_1, h_2, ...`）の組み合わせによって決定されます。
たとえば、

- 範囲 `R = (r_1, h_1)` の CAR の数は 1 です
- 範囲 `R = (r_1, h_2)` の CAR の数は 2 です
- 範囲 `R = (r_2, h_1)` の CAR の数は 3 です
- 範囲 `R = (r_2, h_2)` の CAR の数は 4 です

#### オブジェクトの総数

指定された検出範囲内で各クラスの一意のオブジェクトの数をカウントします。オブジェクトの総数は以下のように計算されます。

$$
\begin{align}
\text{オブジェクトの総数 (クラス、範囲)} = \left| \bigcup_{t=0}^{T_{\text{now}}} \{ \text{uuid} \mid \text{class}(t, \text{uuid}) = C \wedge \text{position}(t, \text{uuid}) \in R \} \right|
\end{align}
$$

- $\bigcup$ は`t = 0` から `T_\text{now}` までのすべてのフレームに対する集合を表し、各 uuid のカウントが1回のみ行われるようにする。
- $\text{class}(t, \text{uuid}) = C$ は時刻 `t` で uuid を持つオブジェクトがクラス `C` に属することを示す。
- $\text{position}(t, \text{uuid}) \in R$ は時刻 `t` で uuid を持つオブジェクトが指定範囲 `R` 内にあることを示す。
- $\left| \{ \ldots \} \right|$ は集合の濃度を表し、すべての対象時間においてクラスと範囲の基準を満たす固有の uuid の数をカウントする。

#### 平均オブジェクト数

指定された検出範囲内の各クラスのオブジェクトの平均数をカウントする。このメトリックは、uuid を考慮せずに1フレームで検出されたオブジェクト数を測定する。平均オブジェクト数は次のように計算される。

$$
\begin{align}
\text{平均オブジェクト数 (クラス, 範囲)} = \frac{1}{N} \sum_{t=0}^{T_\text{now}} \left| \{ \text{object} \mid \text{class}(t, \text{object}) = C \wedge \text{position}(t, \text{object}) \in R \} \right|
\end{align}
$$

ただし:

- `N` はタイム期間 `t` から `T_\text{now}` 内のフレームの総数を示す (正確に `detection_count_purge_seconds`)。
- `\text{object}` は時刻 `t` でクラスと範囲の基準を満たすオブジェクトの数を示す。

#### 間隔オブジェクト数

過去 `objects_count_window_seconds` 間で指定された検出範囲内の各クラスのオブジェクトの平均数をカウントする。このメトリックは、uuid を考慮せずに1フレームで検出されたオブジェクト数を測定する。間隔オブジェクト数は次のように計算される。

$$
\begin{align}
\text{間隔オブジェクト数 (クラス, 範囲)} = \frac{1}{W} \sum_{t=T_\text{now} - T_W}^{T_\text{now}} \left| \{ \text{object} \mid \text{class}(t, \text{object}) = C \wedge \text{position}(t, \text{object}) \in R \} \right|
\end{align}
$$

ただし:

- `W` は `objects_count_window_seconds` 秒内のフレームの総数を示す。
- `T_W` はタイムウィンドウ `objects_count_window_seconds` を示す。

## 入力 / 出力

| Name              | Type                                              | Description                                       |
| ----------------- | ------------------------------------------------- | ------------------------------------------------- |
| `~/input/objects` | `autoware_perception_msgs::msg::PredictedObjects` | 評価する予測オブジェクト。                          |
| `~/metrics`       | `diagnostic_msgs::msg::DiagnosticArray`           | 知覚精度の診断情報。                              |
| `~/markers`       | `visualization_msgs::msg::MarkerArray`            | デバッグと可視化用のビジュアルマーカー。          |

## パラメータ設定

| 名称                                               | タイプ         | 説明                                                                                                                                               |
| -------------------------------------------------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `selected_metrics`                                | リスト         | 横ずれ、ヨーずれ、予測パスずれなどの評価する測定基準。                                                                                         |
| `smoothing_window_size`                             | 整数          | パスの平滑化ウィンドウサイズを決定する。奇数にする必要がある。                                                                                 |
| `prediction_time_horizons`                          | リスト[double] | 秒単位の予測評価の時間軸。                                                                                                               |
| `stopped_velocity_threshold`                       | double       | 車両が停止しているかどうかをチェックする速度しきい値。                                                                                       |
| `detection_radius_list`                            | リスト[double] | 評価するオブジェクトの検出半径（オブジェクトカウントのみに使用）。                                                                             |
| `detection_height_list`                             | リスト[double] | 評価するオブジェクトの検出高さ（オブジェクトカウントのみに使用）。                                                                             |
| `detection_count_purge_seconds`                     | double       | オブジェクト検出カウントを消去する時間ウィンドウ。                                                                                             |
| `objects_count_window_seconds`                      | double       | オブジェクト検出カウントを保持する時間ウィンドウ。この時間ウィンドウ内のオブジェクト検出数が `detection_count_vector_` に格納される。 |
| `target_object.*.check_lateral_deviation`           | ブール         | 特定のオブジェクト種類（車、トラックなど）の横ずれをチェックするかどうか。                                                                   |
| `target_object.*.check_yaw_deviation`              | ブール         | 特定のオブジェクト種類（車、トラックなど）のヨーずれをチェックするかどうか。                                                                   |
| `target_object.*.check_predicted_path_deviation`    | ブール         | 特定のオブジェクト種類（車、トラックなど）の予測パスずれをチェックするかどうか。                                                           |
| `target_object.*.check_yaw_rate`                   | ブール         | 特定のオブジェクト種類（車、トラックなど）のヨーレートをチェックするかどうか。                                                                |
| `target_object.*.check_total_objects_count`         | ブール         | 特定のオブジェクト種類（車、トラックなど）の総オブジェクトカウントをチェックするかどうか。                                                     |
| `target_object.*.check_average_objects_count`       | ブール         | 特定のオブジェクト種類（車、トラックなど）の平均オブジェクトカウントをチェックするかどうか。                                                   |
| `target_object.*.check_interval_average_objects_count` | ブール         | 特定のオブジェクト種類（車、トラックなど）のインターバル平均オブジェクトカウントをチェックするかどうか。                                       |
| `debug_marker.*`                                   | ブール         | マーカーの可視化（履歴パス、予測パスなど）のデバッグパラメータ。                                                                                     |

## 想定 / 已知の制限

PredictedObjectの現在の位置はおおよそ正確であると想定されます。

## 将来の拡張 / 未実装部分

- クラスごとの認識率の向上
- 異常な物理的挙動（例：フェンスをすり抜ける）を示す物体に関するメトリクス
- 物体を分割するためのメトリクス
- 通常は静止しているが移動する物体の問題に関するメトリクス
- 消滅する物体のメトリクス

