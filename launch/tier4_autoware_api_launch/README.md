# tier4_autoware_api_launch

## 説明

このパッケージには、Autowareの内部トピックを外部ソフトウェア（フリート管理システム、シミュレーターなど）が使用する一貫したAPIに変換するノードを実行するための起動ファイルを格納しています。

## パッケージの依存関係

`package.xml` の `<exec_depend>` を参照してください。

## 使用方法

以下のように `*.launch.xml` に含めて `autoware_api.launch.xml` を利用できます。


```xml
  <include file="$(find-pkg-share tier4_autoware_api_launch)/launch/autoware_api.launch.xml"/>
```

## 注釈

処理負荷を低減するために、ROS 2 では [コンポーネント](https://docs.ros.org/en/galactic/Concepts/About-Composition.html) 機能（ROS 1 の Nodelet に類似）を使用します。

