# Autoware Universe

## Autoware Universe へようこそ

Autoware Universe は自動運転技術の中核機能を強化において重要な役割を果たし、Autoware エコシステムの基盤を担っています。
このリポジトリは Autoware Core/Universe のコンセプトの中心的な要素であり、自動運転車両の機能を大幅に拡張する幅広いパッケージを管理しています。

![autoware_universe_front](docs/assets/images/autoware_universe_front.png)

## はじめに

Autoware の広大な世界に飛び込み、Autoware Universe がどのように全体像に当てはまるかを理解するには、[Autoware ドキュメント](https://autowarefoundation.github.io/autoware-documentation/) から始めることをお勧めします。このリソースは、Autoware エコシステムの包括的な概要を提供し、そのコンポーネント、機能、開発の開始方法について案内します。

### Autoware Universe ドキュメントの探査

Autoware Universe コンポーネントの詳細を探求したい場合は、MKDocs で展開された [Autoware Universe ドキュメント](https://autowarefoundation.github.io/autoware.universe/) で詳細な洞察が得られます。

## コードカバレッジの測定値

以下の表は、Autoware Universe 全体とサブコンポーネントのそれぞれのカバレッジ率を示しています。

### プロジェクト全体のカバー率

[![codecov](https://codecov.io/github/autowarefoundation/autoware.universe/graph/badge.svg?token=KQP68YQ65D)](https://codecov.io/github/autowarefoundation/autoware.universe)

### コンポーネントごとのカバー率

バッジをクリックして codecov Web サイトにアクセスすることで、詳細を確認できます。

| コンポーネント | カバレッジ                                                                                                                                                                                                                                                                                                      |
|---|---|
| Common | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Common%20Packages&query=$.[0].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Common%20Packages) |
| Control | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Control%20Packages&query=$.[1].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Control%20Packages) |
| Evaluator | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Evaluator%20Packages&query=$.[2].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Evaluator%20Packages) |
| Launch | 未定                                                                                                                                                                                                                                                                                                      |
| Localization | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Localization%20Packages&query=$.[4].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Localization%20Packages) |
| Map | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Map%20Packages&query=$.[5].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Map%20Packages) |
| Perception | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Perception%20Packages&query=$.[6].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Perception%20Packages) |
| Planning | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Planning%20Packages&query=$.[7].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Planning%20Packages) |
| Sensing | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Sensing%20Packages&query=$.[8].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Sensing%20Packages) |
| Simulator | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Simulator%20Packages&query=$.[9].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Simulator%20Packages) |
| System | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=System%20Packages&query=$.[10].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=System%20Packages) |
| Vehicle | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware.universe/components&label=Vehicle%20Packages&query=$.[11].coverage)](https://app.codecov.io/gh/autowarefoundation/autoware.universe?components%5B0%5D=Vehicle%20Packages) |

## 自動運転ソフトウェア

### 目次

- [概要](#概要)
- [コンポーネント](#コンポーネント)
  - [Perception](#Perception)
  - [Planning](#Planning)
  - [Control](#Control)
- [Autowareについて](#Autowareについて)
  - [使用方法](#使用方法)
  - [利用上の注意](#利用上の注意)

### 概要

自動運転ソフトウェアは、車両が人間介入なしで安全かつ効率的に道路を走行できるようにするソフトウェアシステムです。

### コンポーネント

自動運転ソフトウェアは、次の主要コンポーネントで構成されています。

- **Perception:** センサーからのデータを使用して、周囲の環境を認識します。
- **Planning:** 障害物を回避し、目的地に到達するための経路を計画します。
- **Control:** ステアリング、アクセル、ブレーキを制御し、計画された経路に従って車両を走行させます。

### Autowareについて

Autowareは、オープンソースの自動運転ソフトウェアプラットフォームです。次のような機能を提供します。

- Perception、Planning、Control用のモジュール
- シミュレーションとテスト用のツール
- リアルタイムオペレーティングシステム（ROS）との統合

### 使用方法

Autowareを使用するには、次の手順に従います。

1. Autowareをインストールします。
2. 必要に応じてモジュールをカスタマイズします。
3. システムをテストします。
4. 車両に展開します。

### 利用上の注意

Autowareを使用する際には、次の点に注意してください。

- システムは安全ではありません。実験的な目的でのみ使用してください。
- システムは常に人間の監視下で使用してください。
- システムは、悪天候やその他の困難な条件下では正しく動作しない可能性があります。

