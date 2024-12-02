# tier4_map_launch

## 構造

![tier4_map_launch](./map_launch.drawio.svg)

## パッケージ依存

`<package.xml>` の`<exec_depend>` を参照してください。

## 使用法

`map.launch.py` を使用するには、次のように `*.launch.xml` に含めることができます。

`PACKAGE_param_path` としてパラメータのパスを指定する必要があることに注意してください。指定する必要のあるパラメータパスのリストは、`map.launch.xml` の先頭に記載されています。


```xml
<arg name="map_path" description="point cloud and lanelet2 map directory path"/>
<arg name="lanelet2_map_file" default="lanelet2_map.osm" description="lanelet2 map file name"/>
<arg name="pointcloud_map_file" default="pointcloud_map.pcd" description="pointcloud map file name"/>

<include file="$(find-pkg-share tier4_map_launch)/launch/map.launch.py">
  <arg name="lanelet2_map_path" value="$(var map_path)/$(var lanelet2_map_file)" />
  <arg name="pointcloud_map_path" value="$(var map_path)/$(var pointcloud_map_file)"/>

  <!-- Parameter files -->
  <arg name="FOO_param_path" value="..."/>
  <arg name="BAR_param_path" value="..."/>
  ...
</include>
```

## 注釈

処理負荷を軽減するため、[コンポーネント](https://docs.ros.org/en/galactic/Concepts/About-Composition.html)機能をROS 2で使用します（ROS 1のNodeletに近い）

