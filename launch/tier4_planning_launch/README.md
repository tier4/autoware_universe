# tier4_planning_launch

## 構造

![tier4_planning_launch](./planning_launch.drawio.svg)

## パッケージ依存関係

`package.xml` 内の `<exec_depend>` を参照してください。

## 使用方法

パラメータパスを `PACKAGE_param_path` として提供する必要があることに注意してください。提供する必要があるパラメータパスのリストは `planning.launch.xml` の最上部に記載されています。


```xml
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <!-- Parameter files -->
  <arg name="FOO_NODE_param_path" value="..."/>
  <arg name="BAR_NODE_param_path" value="..."/>
  ...
</include>
```

