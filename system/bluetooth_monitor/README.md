# bluetooth_monitor

## 説明

このノードは、L2pingを使用してワイヤレスデバイスへのBluetooth接続を監視します。<br>
L2pingは、Bluetooth L2CAPレイヤーでPINGエコーコマンドを生成します。そして、ワイヤレスデバイスからエコーレスポンスを受信してチェックできます。

## ブロックダイアグラム

L2pingはデフォルトではrootにのみ許可されているため、このパッケージはセキュリティリスクを可能な限り最小限に抑えるために、次のようなアプローチを提供します。

- `l2ping_service`という、L2pingを実行し、ソケットプログラミングを使用してワイヤレスデバイス情報を`bluetooth_monitor`に提供する小さなプログラムを提供します。
- `bluetooth_monitor`は、これらの情報がソケット通信によって送信されるため、非特権ユーザーとしてワイヤレスデバイスの情報とL2pingの状態を知ることができます。

![block_diagram](docs/block_diagram.drawio.svg)

## 出力

### <u>bluetooth_monitor: bluetooth_connection</u>

<b>[概要]</b>

## 自動運転ソフトウェア

**翻訳**

**メッセージレベル**

| レベル | メッセージ |
|---|---|
| OK    | OK |
| WARN  | RTT警告 |
| ERROR | ロスト |
|       | 関数エラー |

<b>[値]</b>

| キー                        | 値（例）                                                       |
| -------------------------- | --------------------------------------------------------------------|
| デバイス [0-9]: ステータス | OK / RTT 警告 / 検証エラー / 損失 / Ping 拒否 / 機能エラー |
| デバイス [0-9]: 名前         | ワイヤレスコントローラー                                                 |
| デバイス [0-9]: 製造者 | MediaTek, Inc.                                                      |
| デバイス [0-9]: アドレス      | AA:BB:CC:DD:EE:FF                                                   |
| デバイス [0-9]: RTT          | 0.00ms                                                              |

- `bluetooth_monitor`が`機能エラー`を報告すると、次のキーが追加されます。<br>
  例）`connect`システムコールが失敗しました。

| キー（例）         | 値（例）           |
| --------------------- | ------------------------- |
| Device [0-9]: connect | そのようなファイルまたはディレクトリはありません |

## パラメーター

{{ json_to_markdown("system/bluetooth_monitor/schema/bluetooth_monitor.schema.json") }}

- `rtt_warn`

  - **0.00(ゼロ)**: RTT チェックを無効にする
  - **それ以外**: 指定した秒数で RTT をチェックする

- `addresses`
  - **\***: すべての接続デバイス
  - **AA:BB:CC:DD:EE:FF**: Bluetooth アドレスを設定することで監視対象のデバイスを指定できる

## 開始前の注意事項

- `l2ping_service` を root ユーザーとして実行すれば、この手順は省略できます。

1. L2ping には `cap_net_raw+eip` ケーパビリティが必要なため、`l2ping_service` にケーパビリティを割り当てます。


   ```sh
   sudo setcap 'cap_net_raw+eip' ./build/bluetooth_monitor/l2ping_service
   ```

2. `l2ping_service` と `bluetooth_monitor` を実行してください。


   ```sh
   ./build/bluetooth_monitor/l2ping_service
   ros2 launch bluetooth_monitor bluetooth_monitor.launch.xml
   ```

## 制限事項および問題点

なし。

