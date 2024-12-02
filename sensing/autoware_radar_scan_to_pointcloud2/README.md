# radar_scan_to_pointcloud2

## radar_scan_to_pointcloud2_node

- `radar_msgs::msg::RadarScan`から`sensor_msgs::msg::PointCloud2`に変換します
- 計算コスト O(n)
  - n: レーダーリターンの数

### 入力トピック

| 名前        | 型                       | 説明 |
| ----------- | -------------------------- | ----------- |
| input/radar | radar_msgs::msg::RadarScan | RadarScan   |

### 出力トピック

| 名称 | 型 | 説明 |
|---|---|---|
| output/amplitude_pointcloud | sensor_msgs::msg::PointCloud2 | 強度が振幅のPointCloud2レーダー点群 |
| output/doppler_pointcloud | sensor_msgs::msg::PointCloud2 | 強度がドップラー速度のPointCloud2レーダー点群 |

### パラメーター

| 名称                         | タイプ | 説明                                                                                        |
| ---------------------------- | ---- | ------------------------------------------------------------------------------------------ |
| publish_amplitude_pointcloud | bool | 強度が振幅のレーダーポイントクラウドを公開するかどうか。既定値は「true」です。          |
| publish_doppler_pointcloud   | bool | 強度がドップラー速度のレーダーポイントクラウドを公開するかどうか。既定値は「false」です。 |

### 実行方法


```sh
ros2 launch autoware_radar_scan_to_pointcloud2 radar_scan_to_pointcloud2.launch.xml
```

