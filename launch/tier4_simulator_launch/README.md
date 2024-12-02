# tier4_simulator_launch

## 構造

![tier4_simulator_launch](./simulator_launch.drawio.svg)

## パッケージの依存関係

`<pack>package.xml</pack>` の `<exec_depend>` を参照してください。

## 使用法


```xml
  <include file="$(find-pkg-share tier4_simulator_launch)/launch/simulator.launch.xml">
    <arg name="vehicle_info_param_file" value="VEHICLE_INFO_PARAM_FILE" />
    <arg name="vehicle_model" value="VEHICLE_MODEL"/>
  </include>
```

`VEHICLE_MODEL`\_descriptionパッケージの"config/simulator_model.param.yaml"から、simple_planning_simulatorで使用されるシミュレーターモデルがロードされます。

