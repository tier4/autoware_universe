# autoware_default_adapi

## 注意

サービスをリレーするコンポーネントは、Multi-Threaded Executor によって実行される必要があります。

## 特徴

このパッケージはデフォルト実装の AD API です。

- [autoware のステータス（後方互換性）](document/autoware-state.md)
- [フェイルセーフ](document/fail-safe.md)
- [インターフェイス](document/interface.md)
- [ローカリゼーション](document/localization.md)
- [モーション](document/motion.md)
- [オペレーションモード](document/operation-mode.md)
- [ルーティング](document/routing.md)

## Web サーバースクリプト

これは、HTTP を使用して API を呼び出すサンプルです。

## ガイドメッセージスクリプト

これは、自律運転モードへの移行条件をチェックするためのデバッグスクリプトです。


```bash
$ ros2 run autoware_default_adapi guide.py

The vehicle pose is not estimated. Please set an initial pose or check GNSS.
The route is not set. Please set a goal pose.
The topic rate error is detected. Please check [control,planning] components.
The vehicle is ready. Please change the operation mode to autonomous.
The vehicle is driving autonomously.
The vehicle has reached the goal of the route. Please reset a route.
```

