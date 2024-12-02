# Planning Topic Converter

## 目的

このパッケージは、<https://github.com/autowarefoundation/autoware_msgs>で定義されている型間でのトピックの型の変換を提供します。

## 機能 / アルゴリズム

### 使用例

このパッケージのツールは構成可能な ROS 2 コンポーネントノードとして提供されるため、既存のプロセスにスポーンしたり、起動ファイルから開始したり、コマンドラインから呼び出したりできます。


```xml
<load_composable_node target="container_name">
  <composable_node pkg="planning_topic_converter" plugin="autoware::planning_topic_converter::PathToTrajectory" name="path_to_trajectory_converter" namespace="">
  <!-- params -->
  <param name="input_topic" value="foo"/>
  <param name="output_topic" value="bar"/>
  <!-- composable node config -->
  <extra_arg name="use_intra_process_comms" value="false"/>
  </composable_node>
</load_composable_node>
```

## パラメータ

| 名称           | タイプ   | 説明        |
| :------------- | :----- | :----------------- |
| `input_topic`  | 文字列 | 入力トピック名。  |
| `output_topic` | 文字列 | 出力トピック名。 |

## 仮定/既知の制限事項

## 今後の拡張/未実装部分

