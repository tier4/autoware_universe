## YabLoc

**YabLoc** は、ベクターマップを用いたビジョンベースのローカリゼーションです。 [https://youtu.be/Eaf6r_BNFfk](https://youtu.be/Eaf6r_BNFfk)

[![サムネイル](docs/yabloc_thumbnail.jpg)](https://youtu.be/Eaf6r_BNFfk)

画像から抽出した路面マーキングをベクターマップと照合して自車位置を推定します。
点群マップやLiDARは不要です。
YabLocは、LiDARを搭載していない車両や、点群マップが利用できない環境でも車両のローカリゼーションを可能にします。

## パッケージ

- [yabloc_common](yabloc_common/README.md)
- [yabloc_image_processing](yabloc_image_processing/README.md)
- [yabloc_particle_filter](yabloc_particle_filter/README.md)
- [yabloc_pose_initializer](yabloc_pose_initializer/README.md)

## YabLocをNDTの代わりに起動する方法

autowareを起動する際、引数に`pose_source:=yabloc`を設定すると、YabLocがNDTの代わりに起動されます。
デフォルトでは、`pose_source`は`ndt`です。

YabLocを実行するためのサンプルコマンドを以下に示します。


```shell
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag\
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  pose_source:=yabloc
```

## アーキテクチャ

![node_diagram](docs/yabloc_architecture.drawio.svg)

## 原理

以下の図はYabLocの基本原理を示しています。
グラフベースセグメンテーションから得られた道路領域を使用して線分を抽出し、道路表面の標識を抽出します。
図の中央上部にある赤い線は、道路表面の標識として識別された線分を表します。
YabLocは各パーティクルに対してこれらのセグメントを変換し、Lanelet2から生成されたコストマップと比較することでパーティクルの重みを決めます。

![principle](docs/yabloc_principle.png)

### コアビジュアライゼーショントピック

これらのトピックはデフォルトでは表示されません。

<img src="docs/yabloc_rviz_description.png" width=800>

| インデックス | トピック名                                             | 説明                                                                                                                                                                |
| ----------- | ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1           | `/localization/yabloc/pf/predicted_particle_marker`            | パティクルフィルタのパーティクル分布。赤いパーティクルは候補として予測。                                                                                            |
| 2           | `/localization/yabloc/pf/scored_cloud`                         | 3D に投影された線分。色はマップとの一致度を示す。                                                                                                                        |
| 3           | `/localization/yabloc/image_processing/lanelet2_overlay_image` | 推定された自車位置に基づき、Lanelet2（黄色の線）を画像にオーバーレイ。実際の道路標示とよく一致していれば、自車位置の算出が適切に行われていることを示す。 |

### デバッグ用画像トピック

これらのトピックは、デフォルトでは可視化されません。

<img src="docs/yabloc_image_description.png" width=800>

| インデックス | トピック名                                                                | 説明                                                                             |
|---|---|---|
| 1     | `/localization/yabloc/pf/cost_map_image`                                  | レーンレット 2 で作成されたコストマップ                                             |
| 2     | `/localization/yabloc/pf/match_image`                                     | 投影線分                                                                          |
| 3     | `/localization/yabloc/image_processing/image_with_colored_line_segment` | 分類された線分。緑色の線分はパーティクル修正で使用されます。                        |
| 4     | `/localization/yabloc/image_processing/lanelet2_overlay_image`            | レーンレット 2 のオーバーレイ                                                     |
| 5     | `/localization/yabloc/image_processing/segmented_image`                   | グラフベースセグメンテーションの結果                                               |

## 制限事項

- YabLocとNDTを同時実行することはできません。
  - これらは同時に実行すると計算負荷が高くなりすぎる可能性があります。
  - また、ほとんどの場合NDTはYabLocよりも優れているため、同時実行する利点はあまりありません。
- ロールとピッチを推定しないため、一部の認識ノードが適切に機能しない可能性があります。
- 現在、複数のカメラには対応していません。ただし、将来的には対応する予定です。
- 交差点など道路標示の少ない場所では、推定はGNSS、IMU、および車両の車輪オドメトリに大きく依存します。
- 道路境界線または道路標示がLanelet2に含まれていない場合、推定が失敗する可能性があります。
- autowareチュートリアルで提供されるサンプルrosbagには画像が含まれていないため、YabLocは実行できません。
  - YabLocの機能をテストしたい場合は、[PR](https://github.com/autowarefoundation/autoware.universe/pull/3946)で提供されているサンプルテストデータを使用すると便利です。

