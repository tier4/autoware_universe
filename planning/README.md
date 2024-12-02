## Planningコンポーネント

## はじめに

Autoware.Universe Planningモジュールは、広範なオープンソース自動運転ソフトウェアスタックの最先端コンポーネントです。これらのモジュールは、自律走行のナビゲーションにおいて重要な役割を果たし、ルート計画、動的障害物回避、さまざまな交通状況へのリアルタイム適応を巧みに処理します。

- Planningコンポーネントのハイレベルな概念については、[Planning Component Design Document](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/)を参照してください。
- Planningコンポーネントが他のコンポーネントとどのように相互作用するかを理解するには、[Planning Component Interface Document](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/planning/)を参照してください。
- [Node Diagram](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/)は、Planningモジュールを含め、Autoware.Universe内のすべてのモジュールの相互作用、入力、出力を示しています。

## Planningモジュール

Planningコンポーネントの**モジュール**は、ソフトウェアの計画システムを共同で形成するさまざまなコンポーネントを表します。これらのモジュールは、自律走行計画に必要な機能の範囲をカバーしています。AutowareのPlanningモジュールはモジュール化されており、ユーザーは構成を変更することで有効にする機能をカスタマイズできます。このモジュール設計により、自律走行におけるさまざまなシナリオや要件に柔軟に対応できます。

### Planningモジュールの有効化と無効化

モジュールの有効化と無効化には、主要な構成ファイルと起動ファイルの設定を管理することが含まれます。

### 構成用のキーファイル

`default_preset.yaml`ファイルはプライマリ構成ファイルとして機能し、計画モジュールを無効化または有効化できます。さらに、ユーザーはさまざまなMotion PlannerでMotion Plannerのタイプを設定することもできます。たとえば:

- `launch_avoidance_module`: `true`に設定して回避モジュールを有効にするか、`false`に設定して無効にします。
- `motion_stop_planner_type`: `default`を`obstacle_stop_planner`または`obstacle_cruise_planner`に設定します。

!!! note

    `default_preset.yaml`を表示するには[ここ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/preset/default_preset.yaml)をクリックしてください。

[起動ファイル](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/launch/scenario_planning/lane_driving)は、動作パスプランナーのノードが実行されているときに構成を適用するために、`default_preset.yaml`で定義された設定を参照します。たとえば、`tier4_planning_launch/launch/scenario_planning/lane_driving/config/avoidance/conf/default_conf.yaml`の`avoidance.enable_module`パラメーターは、以下のように設定されます:

```
avoidance:
  enable_module: ${launch_avoidance_module}
```

これにより、`default_preset.yaml`の`launch_avoidance_module`設定が、回避モジュールが有効化または無効化されるかどうかを決定するようになります。


```xml
<param name="avoidance.enable_module" value="$(var launch_avoidance_module)"/>
```

 соответствует `default_preset.yaml` の `launch_avoidance_module` に対応します。

### パラメーター設定

構成可能なパラメーターは複数あり、ユーザーは[こちら](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning)で変更するオプションがあります。ただし、パラメーターの設定がすべて `rqt_reconfigure` から調整できるわけではありません。変更内容が有効になるようにするには、パラメーターを変更してから Autoware を再起動してください。さらに、各パラメーターの詳細情報は、Planning タブにある対応するドキュメントに記載されています。

### Autoware にカスタムモジュールを統合するためのステップバイステップガイド

このガイドでは、カスタムモジュールを Autoware に統合するための手順を説明します。

- モジュールを `default_preset.yaml` ファイルに追加します。例:


```yaml
- arg:
  name: launch_intersection_module
  default: "true"
```

- [ランチャー](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/launch/scenario_planning)にモジュールを組み込みます。例えば、[behavior_planning.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml)で：


```xml
<arg name="launch_intersection_module" default="true"/>

<let
  name="behavior_velocity_planner_launch_modules"
  value="$(eval &quot;'$(var behavior_velocity_planner_launch_modules)' + 'behavior_velocity_planner::IntersectionModulePlugin, '&quot;)"
  if="$(var launch_intersection_module)"
/>
```

- 該当する場合、パラメータフォルダを適切な既存のパラメータフォルダ内に配置します。たとえば、[インターセクションモジュールのパラメータ](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/intersection.param.yaml)は[behavior_velocity_planner](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner)内にあります。
- [tier4_planning_component.launch.xml](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/components/tier4_planning_component.launch.xml)にパラメータのパスを挿入します。たとえば、`behavior_velocity_planner_intersection_module_param_path`が使用されます。


```xml
<arg name="behavior_velocity_planner_intersection_module_param_path" value="$(var behavior_velocity_config_path)/intersection.param.yaml"/>
```

- 対応するランチャー内でパラメータパスの変数を定義します。たとえば、[behavior_planning.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/04aa54bf5fb0c88e70198ca74b9ac343cc3457bf/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml#L191)


```xml
<param from="$(var behavior_velocity_planner_intersection_module_param_path)"/>
```

!!! note

    追加したい特定のモジュールに応じて、関連ファイルと手順が異なる場合があります。このガイドでは、一般的な概要と出発点を提供しています。これらの手順をモジュールの特異性に適応することが重要です。

## コミュニティ主導の取り組みへの参加

Autowareはコミュニティの協力を重視しています。大小を問わず、すべての貢献は私たちにとって貴重です。バグの報告、改善の提案、新しいアイデアの提供など、他に考え付くものが何でもあります - 私たちはすべてを歓迎します。

### 貢献する方法

貢献する準備はできましたか？素晴らしい！始めに、[貢献ガイドライン](https://autowarefoundation.github.io/autoware-documentation/main/contributing/) にアクセスしてください。そこには、飛び込むために必要な情報がすべて記載されています。これには、バグレポートの送信、機能強化の提案、さらにはコードベースへの貢献に関する指示が含まれます。

### Planning & Controlワーキンググループミーティングへの参加

Planning & Controlワーキンググループは、私たちのコミュニティの不可欠な部分です。私たちは2週間ごとに会合して、現在の進捗状況、今後の課題について話し合い、新しいアイデアをブレインストーミングしています。これらのミーティングは、私たちの議論と意思決定プロセスに直接貢献する素晴らしい機会です。

ミーティングの詳細：

- **頻度**：2週間ごと
- **曜日**：木曜日
- **時間**：08:00 AM UTC (05:00 PM JST)
- **議題**：現在の進捗を話し合い、今後の開発を計画する。過去のミーティングの議事録はこちら[こちら](https://github.com/orgs/autowarefoundation/discussions?discussions_q=is%3Aopen+label%3Ameeting%3Aplanning-control-wg+)で確認およびコメントできます。

私たちの会議に参加することに興味がありますか？大歓迎です！参加方法の詳細については、次のリンクをご覧ください：[ワーキンググループに参加する方法](https://github.com/autowarefoundation/autoware-projects/wiki/Autoware-Planning-Control-Working-Group#how-to-participate-in-the-working-group)。

### 引用

場合によっては、AutowareのPlanningコンポーネントに固有の論文を発行します。これらの出版物を調べて、あなたの仕事に貴重な洞察を得ていただくことをお勧めします。それらが有用であり、プロジェクトに私たちの方法論やアルゴリズムのうちのいずれかを組み込む場合は、私たちの論文を引用していただけると非常に役立ちます。このサポートにより、私たちはより多くの聴衆にリーチし、この分野への貢献を続けることができます。

**Planning**コンポーネントの[**Motion Velocity Smoother**](./autoware_velocity_smoother/README.md)モジュールでJerk Constrained Velocity Planningアルゴリズムを使用する場合は、関連する論文を引用するようお願いいたします。

<!-- cspell:ignore Shimizu, Horibe, Watanabe, Kato -->

**Y. Shimizu, T. Horibe, F. Watanabe, S. Kato, "[Jerk Constrained Velocity Planning for an Autonomous Vehicle: Linear Programming Approach](https://arxiv.org/abs/2202.10029)", 2022 International Conference on Robotics and Automation (ICRA)**


```tex
@inproceedings{shimizu2022,
  author={Shimizu, Yutaka and Horibe, Takamasa and Watanabe, Fumiya and Kato, Shinpei},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  title={Jerk Constrained Velocity Planning for an Autonomous Vehicle: Linear Programming Approach},
  year={2022},
  pages={5814-5820},
  doi={10.1109/ICRA46639.2022.9812155}}
```

