# ad_api_adaptors

## initial_pose_adaptor

このノードを使用すると、RViz からローカリゼーション AD API を簡単に使用できます。
初期姿勢のトピックを受信すると、ローカリゼーション初期化 API を呼び出します。
このノードはマップの高さの調整ライブラリに依存しています。
[詳細はこちらをご覧ください。](../../../map/autoware_map_height_fitter/README.md)

| Interface | Local Name | Global Name | Description |
|---|---|---|---|
| Subscription | initialpose | /initialpose | 局所化初期化のためのポーズ |
| Client | - | /api/localization/initialize | 局所化初期化 API |

## routing_adaptor

このノードを使用すると、RVizからRouting AD APIを簡単に利用できます。
ゴールのポーズに関するトピックを受信すると、ウェイポイントをリセットしてAPIを呼び出します。
ウェイポイントのポーズに関するトピックを受信すると、APIを呼び出すためにウェイポイントの最後に追加します。
ルートを設定する前に、clear APIが自動的に呼び出されます。

| インターフェース | ローカル名 | グローバル名 | 説明 |
|---|---|---|---|
| サブスクリプション | - | /api/routing/state | ルーティング API の状態 |
| サブスクリプション | ~/input/fixed_goal | /planning/mission_planning/goal | ルートの目標姿勢。ゴールの変更を無効にする |
| サブスクリプション | ~/input/rough_goal | /rviz/routing/rough_goal | ルートの目標姿勢。ゴールの変更を有効にする |
| サブスクリプション | ~/input/reroute | /rviz/routing/reroute | 再ルーティングの目標姿勢 |
| サブスクリプション | ~/input/waypoint | /planning/mission_planning/checkpoint | ルートのウェイポイント姿勢 |
| クライアント | - | /api/routing/clear_route | ルートクリア API |
| クライアント | - | /api/routing/set_route_points | ルートポイントセット API |
| クライアント | - | /api/routing/change_route_points | ルートポイント変更 API |

## パラメーター

{{ json_to_markdown("/system/default_ad_api_helpers/ad_api_adaptors/schema/ad_api_adaptors.schema.json") }}

