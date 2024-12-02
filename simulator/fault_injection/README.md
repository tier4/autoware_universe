# fault_injection

## 目的

このパッケージは、PSim から偽のシステム障害を診断に変換し、Autoware に通知するために使用されます。
コンポーネント構成図を以下に示します:

![fault_injection パッケージのコンポーネント構成図](img/component.drawio.svg)

## テスト


```bash
source install/setup.bash
cd fault_injection
launch_test test/test_fault_injection_node.test.py
```

## 内部の仕組み/アルゴリズム

## 入出力

### 入力

| 名前                        | タイプ                                           | 説明       |
| --------------------------- | ---------------------------------------------- | ----------------- |
| `~/input/simulation_events` | `tier4_simulation_msgs::msg::SimulationEvents` | シミュレーションイベント |

### 出力

Autowareのアーキテクチャは、**Perception**、**Planning**、**Behavior Control**、**Vehicle Control**の4つの主要コンポーネントで構成されています。

**Planning**コンポーネントは、Perceptionコンポーネントから提供された周辺環境の情報を処理し、安全で効率的な走行軌跡を生成します。

**Behavior Control**コンポーネントは、Planningコンポーネントによって生成された軌跡を、現在の状況に適応して変更し、加減速とステアリングコマンドを生成します。

**Vehicle Control**コンポーネントは、Behavior Controlコンポーネントから生成されたコマンドを受け取り、車両ハードウェアに送信します。

**Perception**コンポーネントは、カメラ、レーダー、LiDARなどのセンサからのデータを処理して、周囲環境の正確な表現である地図を作成します。

このアーキテクチャにより、Autowareは以下を実現できます。

- **リアルタイム性能**：Perception、Planning、Behavior Control、Vehicle Controlの各コンポーネントは、リアルタイムで動作します。
- **モジュール性**：各コンポーネントは独立しており、他のコンポーネントに依存せずに開発およびテストできます。
- **拡張性**：Autowareは、新しいセンサやアルゴリズムを簡単に統合できるように設計されています。

### Planningモジュール

Planningモジュールは、以下を含む複数のサブモジュールで構成されています。

- **Motion Planning**：障害物を回避し、交通規則に従う安全な走行軌跡を生成します。
- **Trajectory Generation**：Motion Planningモジュールによって生成された軌跡に沿って、車体を滑らかに移動させるために、速度と加速度のプロファイルを作成します。
- **'Post Resampling'**：Planningモジュールは、軌跡と制御入力を一定の周波数で生成しますが、Vehicle Controlコンポーネントがより高い周波数で車両ハードウェアにコマンドを送信する場合があります。Post Resamplingモジュールは、元の軌跡と制御入力を補間して、Vehicle Controlコンポーネントが必要とする詳細なコマンドを生成します。
- **Decision Making**：現在の状況に基づいて、Planningモジュールは、異なる軌跡や制御戦略の候補から最適なものを選択します。

### Behavior Controlモジュール

Behavior Controlモジュールは、以下を含む複数のサブモジュールで構成されています。

- **Longitudinal Controller**：車両の速度を制御し、衝突を回避するために加減速コマンドを生成します。
- **Lateral Controller**：車両のステアリングを制御し、軌跡を追従します。
- **Motion Control**：車両の運動を制御し、ジャークや振動を抑えます。

### Vehicle Controlモジュール

Vehicle Controlモジュールは、以下を含む複数のサブモジュールで構成されています。

- **Actuator Control**：車両のアクセル、ブレーキ、ステアリングを制御するためのコマンドを生成します。
- **State Estimation**：Vehicle Controlコンポーネントは、センサからのデータを処理して、自車位置、速度、加速度などの車両の状態を推定します。
- **Error Correction**：自車位置の推定値と目標軌跡との間の誤差に基づいて、制御コマンドを補正します。

| 名前            | 型                                     | 説明               |
| --------------- | --------------------------------------- | ------------------ |
| `/diagnostics` | `diagnostic_msgs::msg::DiagnosticArray` | ダミー診断値       |

## パラメータ

なし

### ノードパラメータ

なし

### コアパラメータ

なし

## 仮定 / 既知の制限

TBD.

