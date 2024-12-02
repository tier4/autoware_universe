# diagnostic_graph_aggregator

## 概要

診断グラフ集約ノードは診断配列をサブスクライブし、集約診断グラフをパブリッシュします。
下の図に示すように、このノードは中間の機能単位に対して追加の診断ステータスを導入します。

![overview](./doc/overview.drawio.svg)

## 診断グラフ構造

診断グラフは実際には Autoware の各運用モードに対するフォールトツリー解析 (FTA) のセットです。
同じノードのステータスは複数のノードから参照される可能性があるため、全体的な構造は有向非巡回グラフ (DAG) です。
診断グラフの各ノードは、入力診断を含め、特定の機能単位の診断ステータスを表します。
そのためこれを「ユニット」と定義し、入力診断に対応するユニットを「diag ユニット」、その他を「ノード ユニット」と呼びます。

すべてのユニットには、DiagnosticStatus と同じエラーレベル、ユニットタイプがあり、オプションでユニットパスがあります。
さらに、すべての diag ユニットには、DiagnosticStatus と同じメッセージ、hardware_id、および値があります。
ユニットタイプは、AND や OR などのユニットステータスの計算方法を表します。
ユニットパスは、ユニットの機能を表す任意の一意の文字列です。

注記: この機能は現在開発中です。
診断グラフは「リンク」もサポートしますが、これはユニット間の接続にステータスが追加される場合があるためです。
たとえば、多くの機能ユニットが初期化が完了するまでエラーステータスになるのは当然です。

## 運用モードの可用性

MRM の場合、このノードは専用メッセージでトップレベルの機能ユニットのステータスをパブリッシュします。
したがって、診断グラフには次の名前の機能ユニットが含まれている必要があります。
この機能はグラフの普遍性を損ない、将来はプラグインまたは別のノードに変更される可能性があります。

- /autoware/operation/stop
- /autoware/operation/autonomous
- /autoware/operation/local
- /autoware/operation/remote
- /autoware/operation/emergency-stop
- /autoware/operation/comfortable-stop
- /autoware/operation/pull-over

## インターフェイス

|- インターフェースのタイプ | インターフェース名 | データタイプ | 説明 |
| ------------------------- | ------------------- | -------------------------------------- | --------------------------------- |
| `subscription` | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |Diagnostics 入力 |
| `publisher` | `/diagnostics_graph/unknowns` | `diagnostic_msgs/msg/DiagnosticArray` | グラフに含まれない Diagnostics |
| `publisher` | `/diagnostics_graph/struct` | `tier4_system_msgs/msg/DiagGraphStruct` | Diagnostics グラフ (静的な部分) |
| `publisher` | `/diagnostics_graph/status` | `tier4_system_msgs/msg/DiagGraphStatus` | Diagnostics グラフ (動的な部分) |
| `publisher` | `/system/operation_mode/availability` | `tier4_system_msgs/msg/OperationModeAvailability` | 操作モードの可用性 |

## パラメータ

| パラメータ名 | データ型 | 説明 |
|---|---|---|
| `graph_file` | `string` | 設定ファイルのパス |
| `rate` | `double` | 集約とトピックの公開率 |
| `input_qos_depth` | `uint` | 入力配列トピックのQoS深度 |
| `graph_qos_depth` | `uint` | 出力グラフのQoS深度 |
| `use_operation_mode_availability` | `bool` | 操作モードの可用性パブリッシャーを使用 |

## 例

これは診断用グラフ設定の例です。設定は複数のファイルに分けることができます。

- [main.yaml](./example/graph/main.yaml)
- [module1.yaml](./example/graph/module1.yaml)
- [module2.yaml](./example/graph/module2.yaml)


```bash
ros2 launch diagnostic_graph_aggregator example-main.launch.xml
```

シミュレーションの一部変更を加えることでグラフを再利用できます。たとえば、シミュレーション用のハードウェアチェックを無効にします。

- [edit.yaml](./example/graph/edit.yaml)


```bash
ros2 launch diagnostic_graph_aggregator example-edit.launch.xml
```

## デバッグツール

- [tree](./doc/tool/tree.md)
- [diagnostic_graph_utils](../diagnostic_graph_utils/README.md)

## グラフファイル形式

- [graph](./doc/format/graph.md)
- [path](./doc/format/path.md)
- [unit](./doc/format/unit.md)
- [edit](./doc/format/edit.md)

