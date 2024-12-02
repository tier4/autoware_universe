# automatic_pose_initializer

## automatic_pose_initializer

このノードは、ローカリゼーションの初期状態が「uninitialized（初期化されていない）」の場合にローカリゼーションの初期化APIを呼び出します。
APIは、基準点が指定されていない場合はGNSSの基準点を使用するため、GNSSを使用した初期化を自動的に実行できます。

| インターフェース | ローカル名 | グローバル名 | 説明 |
|---|---|---|---|
| サブスクリプション | - | /api/localization/initialization_state | ローカリゼーション初期化状態 API |
| クライアント | - | /api/localization/initialize | ローカリゼーション初期化 API |

