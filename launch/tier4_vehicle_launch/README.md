# tier4_vehicle_launch

## 構造

![tier4_vehicle_launch](./vehicle_launch.drawio.svg)

## パッケージ依存関係

`<exec_depend>` を `package.xml` で参照してください。

## 使用方法

`vehicle.launch.xml` を使用する場合は、`*.launch.xml` に以下のように含めることができます。


```xml
  <arg name="vehicle_model" default="sample_vehicle" description="vehicle model name"/>
  <arg name="sensor_model" default="sample_sensor_kit" description="sensor model name"/>

  <include file="$(find-pkg-share tier4_vehicle_launch)/launch/vehicle.launch.xml">
    <arg name="vehicle_model" value="$(var vehicle_model)"/>
    <arg name="sensor_model" value="$(var sensor_model)"/>
  </include>
```

## メモ

このパッケージは、変数とパッケージ名を使用して外部パッケージと設定をいくつか検索します。

例）


```xml
<let name="vehicle_model_pkg" value="$(find-pkg-share $(var vehicle_model)_description)"/>
```


```xml
<arg name="config_dir" default="$(find-pkg-share individual_params)/config/$(var vehicle_id)/$(var sensor_model)"/>
```

## vehicle.xacro

### 引数

| 名称          | タイプ   | 説明        | デフォルト |
| ------------- | ------ | ----------- | ------- |
| sensor_model  | String | センサーモデル名  | ""      |
| vehicle_model | String | 車両モデル名 | ""      |

### 使用方法

`*.launch.xml`に次のように記載できます。


```xml
  <arg name="vehicle_model" default="sample_vehicle" description="vehicle model name"/>
  <arg name="sensor_model" default="sample_sensor_kit" description="sensor model name"/>
  <arg name="model" default="$(find-pkg-share tier4_vehicle_launch)/urdf/vehicle.xacro"/>

  <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher">
    <param name="robot_description" value="$(command 'xacro $(var model) vehicle_model:=$(var vehicle_model) sensor_model:=$(var sensor_model)')"/>
  </node>

```

