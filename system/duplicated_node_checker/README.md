# 重複ノードチェッカー

## 目的

このノードはROS 2環境を監視し、環境内のノード名が重複しているのを検出します。
結果は診断メッセージとしてパブリッシュされます。

### スタンドアロンでの起動


```bash
ros2 launch duplicated_node_checker duplicated_node_checker.launch.xml
```

## 内部動作／アルゴリズム

トピックステータスの種類とそれに対応する診断ステータスを次に示します。

| 複製状況 | 診断状況 | 説明 |
|---|---|---|
| `OK` | OK | 複製は検出されません |
| `複製が検出されました` | ERROR | 複製が検出されました |

## 入出力

### 出力

| 名称           | 型                                    | 説明             |
| -------------- | ------------------------------------- | ---------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 診断結果出力 |

## パラメータ

{{ json_to_markdown("system/duplicated_node_checker/schema/duplicated_node_checker.schema.json") }}

## 想定事項 / 既知の限界事項

TBD.

