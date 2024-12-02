# tier4_perception_launch

## 構造

![tier4_perception_launch](./perception_launch.drawio.svg)

## パッケージの依存関係

`<exec_depend>` を `package.xml` で参照してください。

## 使用方法

`*.launch.xml` に次のように含めることで `perception.launch.xml` を使用できます。

パラメータパスを `PACKAGE_param_path` として提供する必要があることに注意してください。提供すべきパラメータパスの一覧は `perception.launch.xml` の先頭に記載されています。


```xml
  <include file="$(find-pkg-share tier4_perception_launch)/launch/perception.launch.xml">
    <!-- options for mode: camera_lidar_fusion, lidar, camera -->
    <arg name="mode" value="lidar" />

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```

