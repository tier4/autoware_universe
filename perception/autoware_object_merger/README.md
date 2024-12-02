# object_merger

## 目的

object_mergerは、データ関連付けにより2つの方式から検出されたオブジェクトをマージするためのパッケージです。

## 内部動作／アルゴリズム

データ関連付け問題（最小コストフロー問題）を解くために、相次ぐ最短経路アルゴリズムが使用されます。コストは2つのオブジェクト間の距離によって計算され、ゲート関数がコストをリセットするために適用され、最大距離、最大領域、最小領域が考慮されます。

## 入出力

### 入力

| 名称            | タイプ                                             | 説明       |
| --------------- | ------------------------------------------------ | ----------- |
| `input/object0` | `autoware_perception_msgs::msg::DetectedObjects` | 検出対象物 |
| `input/object1` | `autoware_perception_msgs::msg::DetectedObjects` | 検出対象物 |

### 出力

Autowareで自動運転ソフトウェアの開発とデバッグを行うためのドキュメントです。次のモジュールを含んでいます。

**Perceptionモジュール:**
- カメラとLiDARのデータから周囲の環境を認識します。

**Planningモジュール:**
- 車両の経路計画と制御を行います。

**Controlモジュール:**
- 加減速とステアリングを制御します。

各モジュールの使用方法とデバッグ方法について説明します。

**デバッグガイド:**
- ログファイルを使用してデバッグします。
- 可視化ツールを使用してデータを確認します。

**Autowareのアーキテクチャ:**
- Autowareのアーキテクチャの概要を説明します。

**用語集:**
- 技術に関する用語を定義します。

**主な情報源:**
- Autowareのドキュメンテーションとリポジトリへのリンクを示します。

**[Autowareのドキュメンテーション](https://www.autoware.org/documentation/)**
**[AutowareのGitHubリポジトリ](https://github.com/autowarefoundation/autoware.ai)**

| 名前            | 型                                                | 説明      |
| --------------- | ------------------------------------------------- | ---------- |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | オブジェクトを修正する |

## パラメータ

{{ json_to_markdown("perception/autoware_object_merger/schema/object_association_merger.schema.json") }}
{{ json_to_markdown("perception/autoware_object_merger/schema/data_association_matrix.schema.json") }}
{{ json_to_markdown("perception/autoware_object_merger/schema/overlapped_judge.schema.json") }}

## ヒント

- クラスタリング手法によって検出された誤検出の未知物体は、急停止のリスクを高め、Planningモジュールに影響を与えることがあります。MLベースの検出器がオブジェクトを見逃すことが稀にある場合、object_mergerのパラメータを調整して認識モジュールに未知のオブジェクトを無視させることができます。
  - 大型車両の近くにある未知のオブジェクトを削除する場合、
    - `distance_threshold_list`をHIGHにする
      - ただし、これにより計算負荷が高くなります
    - `precision_threshold_to_judge_overlapped`をLOWにする
    - `generalized_iou_threshold`をLOWにする
      - ただし、これら2つのパラメータは、既知のオブジェクトの近くにあるオブジェクトを見落とすリスクを高めます。

## 仮定/既知の制限

<!-- 実装の想定と制限を記述します。

例:
  このアルゴリズムでは障害物が動かないと仮定しているため、障害物を回避し始めた後に障害物が急速に動くと、障害物と衝突する可能性があります。
  また、このアルゴリズムでは死角を考慮しません。一般的に、近すぎる障害物はセンシング性能の限界により見えないため、障害物に対して十分な余裕を取ってください。
-->

## (任意) エラー検出と処理

<!-- エラーを検出する方法と回復する方法を記述します。

例:
  このパッケージは最大20個の障害物を処理できます。それ以上の障害物が検出された場合、このノードは諦めて診断エラーを発生させます。
-->

## (任意) パフォーマンス特性

<!-- 複雑性などのパフォーマンス情報を記述します。ボトルネックにならない場合は必須ではありません。

例:
  ### 複雑性

  このアルゴリズムはO(N)です。

  ### 処理時間

  ...
-->

## (任意) 参照/外部リンク

<!-- 実装時に参照したリンクを記述します。

例:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (任意) 今後の拡張/未実装部分

データアソシエーションアルゴリズムはmulti_object_trackerと同じでしたが、multi_object_trackerのアルゴリズムはすでに更新されています。

