# autoware_geo_pose_projector

## 概要

このノードは、地理参照された姿勢トピックをサブスクライブして、マップフレーム内の姿勢をパブリッシュするシンプルなノードです。

## サブスクライブされるトピック

| 名前                      | タイプ                                                     | 説明         |
| ------------------------- | --------------------------------------------------------- | ---------------- |
| `input_geo_pose`          | `geographic_msgs::msg::GeoPoseWithCovarianceStamped` | 地理参照された位置 |
| `/map/map_projector_info` | `tier4_map_msgs::msg::MapProjectedObjectInfo`        | マップ投影情報  |

## パブリッシュされるトピック

| 名称          | 型                                            | 説明                           |
| ------------- | ----------------------------------------------- | ------------------------------------- |
| `output_pose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | マップ座標系における自車位置                     |
| `/tf`         | `tf2_msgs::msg::TFMessage`                      | 親リンクから子リンクへの変換                       |

## パラメータ

{{ json_to_markdown("localization/autoware_geo_pose_projector/schema/geo_pose_projector_ja.schema.json") }}

## 制約事項

使用する投影の種類によっては、共分散の変換が正しくない場合があります。入力トピックの共分散は、(緯度、経度、高度)で対角行列として表されます。
現在、x軸を東方向で、y軸を北方向と仮定しています。そのため、この仮定が破綻すると、特に緯度と経度の共分散が異なる場合、変換が正しくない可能性があります。

