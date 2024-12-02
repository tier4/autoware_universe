# detected_object_validation

## 目的

このパッケージの目的は、検出されたオブジェクトの明白な誤検知を除去することです。

## リファレンス/外部リンク

- [障害物点群ベースバリデータ](obstacle-pointcloud-based-validator.md)
- [占有グリッドベースバリデータ](occupancy-grid-based-validator.md)
- [オブジェクトレーンレットフィルタ](object-lanelet-filter.md)
- [オブジェクト位置フィルタ](object-position-filter.md)

### ノードパラメータ

#### object_lanelet_filter

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/object_lanelet_filter.schema.json") }}

#### object_position_filter

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/object_position_filter.schema.json") }}

#### obstacle_pointcloud_based_validator

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/obstacle_pointcloud_based_validator.schema.json") }}

#### occupancy_grid_based_validator

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/occupancy_grid_based_validator.schema.json") }}

