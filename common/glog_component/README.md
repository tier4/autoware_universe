# glog_component

本パッケージは、ros2コンポーネントライブラリとしてglog（googleロギングライブラリ）の機能を提供します。これを使用してコンテナでglog機能を動的ロードします。

機能の詳細については、[glog github](https://github.com/google/glog)を参照してください。

## 使用例

コンテナで`glog_component`をロードする場合、以下のようにlaunchファイルを設定できます。


```py
glog_component = ComposableNode(
    package="glog_component",
    plugin="GlogComponent",
    name="glog_component",
)

container = ComposableNodeContainer(
    name="my_container",
    namespace="",
    package="rclcpp_components",
    executable=LaunchConfiguration("container_executable"),
    composable_node_descriptions=[
        component1,
        component2,
        glog_component,
    ],
)
```

