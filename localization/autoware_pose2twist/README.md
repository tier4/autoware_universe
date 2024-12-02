## autoware_pose2twist

## 目的

この`autoware_pose2twist`は、入力された姿勢の履歴から速度を計算します。計算されたツイストに加え、このノードは、デバッグを簡素化するために、線形x成分と角加速度z成分をfloatメッセージとして出力します。

`twist.linear.x`は、`sqrt(dx * dx + dy * dy + dz * dz) / dt`として計算され、`y`と`z`フィールドの値は0です。
`twist.angular`は、各フィールドに対して`relative_rotation_vector / dt`として計算されます。

## 入出力

### 入力

| 名 | 型 | 説明 |
| -- | -- | -- |
| pose | geometry_msgs::msg::PoseStamped | 速度の計算に使用されるポーズのソース |

## 自動運転ソフトウェアのアーキテクチャ

### 概要

このドキュメントでは、Autoware's自動運転ソフトウェアのアーキテクチャについて説明します。このソフトウェアは、Perception（認識）、Planning（計画）、Control（制御）の3つの主要コンポーネントで構成されています。

### Perceptionコンポーネント

Perceptionコンポーネントは、センサデータから周囲環境を認識します。次のタスクを行います。

- オブジェクト検出
- 車線検出
- 交通標識検出

### Planningコンポーネント

Planningコンポーネントは、Perceptionモジュールから提供された情報に基づいて、車両の経路と速度を計画します。次のタスクを行います。

- パス計画
- 速度計画

### Controlコンポーネント

Controlコンポーネントは、Planningモジュールから提供された計画に基づいて、車両のステアリング、加速、ブレーキを制御します。次のタスクを行います。

- ステアリング制御
- 加速制御
- ブレーキ制御

### データフロー

Perception、Planning、Controlコンポーネント間のデータフローを以下に示します。

1. Perceptionコンポーネントは、周囲環境の認識結果をPlanningコンポーネントに送信します。
2. Planningコンポーネントは、Perceptionコンポーネントのデータに基づいて計画を作成し、Controlコンポーネントに送信します。
3. Controlコンポーネントは、Planningコンポーネントの計画に基づいて車両を制御します。

### その他のコンポーネント

自動運転ソフトウェアには、上記3つの主要コンポーネントに加えて、以下のコンポーネントも含まれます。

- **Map Managementコンポーネント:** HDマップを管理します。
- **Localizationコンポーネント:** 自車位置を推定します。
- **Vehicle Interfaceコンポーネント:** 車載コンピュータと車両の通信を処理します。

| 名前 | タイプ | 説明 |
|---|---|---|
| twist | geometry_msgs::msg::TwistStamped | 入力ポーズ履歴から計算されたツイスト |
| linear_x | tier4_debug_msgs::msg::Float32Stamped | 出力ツイストの linear-x フィールド |
| angular_z | tier4_debug_msgs::msg::Float32Stamped | 出力ツイストの angular-z フィールド |

## パラメータ

なし。

## 想定 / 留意事項

なし。

