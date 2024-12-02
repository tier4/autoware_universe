# radar_threshold_filter

## radar_threshold_filter_node

レーダーリターンのノイズを閾値によって除去します。

- 振幅フィルタ: 低振幅はノイズと見なされます
- FOVフィルタ: レーダーの視野の端からの点群は擾乱が発生します
- 距離フィルタ: 近すぎる点群はノイズが発生することがよくあります

計算コストはO(n)です。`n`はレーダーリターンの数です。

### 入力トピック

| 名 | 型 | 説明 |
|---|---|---|
| input/radar | radar_msgs/msg/RadarScan.msg | レーダポイントクラウドデータ |

### 出力トピック

| 名前         | タイプ                         | 説明               |
| ------------ | ---------------------------- | ------------------------- |
| output/radar | radar_msgs/msg/RadarScan.msg | フィルタリングされたレーダー点群 |

### パラメータ

- ノードパラメータの場合

| 名前                | 型   | 説明                                                                                 |
| ------------------- | ------ | ------------------------------------------------------------------------------------------- |
| is_amplitude_filter | bool   | このパラメータが真の場合、振幅フィルタを適用（振幅_最小 < 振幅 < 振幅_最大をパブリッシュ） |
| amplitude_min       | double | [dBm^2]                                                                                   |
| amplitude_max       | double | [dBm^2]                                                                                   |
| is_range_filter     | bool   | このパラメータが真の場合、レンジフィルタを適用（レンジ_最小 < レンジ < レンジ_最大をパブリッシュ） |
| range_min           | double | [m]                                                                                       |
| range_max           | double | [m]                                                                                       |
| is_azimuth_filter   | bool   | このパラメータが真の場合、角度フィルタを適用（方位角_最小 < 範囲 < 方位角_最大をパブリッシュ） |
| azimuth_min         | double | [rad]                                                                                     |
| azimuth_max         | double | [rad]                                                                                     |
| is_z_filter         | bool   | このパラメータが真の場合、Z位置フィルタを適用（z_最小 < z < z_最大をパブリッシュ） |
| z_min               | double | [m]                                                                                       |
| z_max               | double | [m]                                                                                       |

### 起動方法


```sh
ros2 launch autoware_radar_threshold_filter radar_threshold_filter.launch.xml
```

