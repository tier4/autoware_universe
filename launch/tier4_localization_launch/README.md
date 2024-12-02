# tier4_localization_launch

## 構造

![tier4_localization_launch](./localization_launch.drawio.svg)

## パッケージの依存関係

`package.xml`の`<exec_depend>`を参照してください。

## 使用

他のlaunchファイルに`localization.launch.xml`を以下のようにインクルードします。

`pose_estimator`または`twist_estimator`として起動するローカライゼーション内のどのメソッドを選択するかを`pose_source`と`twist_source`を指定して選択できます。

さらに、パラメーターパスを`PACKAGE_param_path`として指定する必要があります。指定する必要のあるパラメーターパスのリストは`localization.launch.xml`の先頭部分に記載されています。


```xml
  <include file="$(find-pkg-share tier4_localization_launch)/launch/localization.launch.xml">
    <!-- Localization methods -->
    <arg name="pose_source" value="..."/>
    <arg name="twist_source" value="..."/>

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```

