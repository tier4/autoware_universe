## tier4_traffic_light_rviz_plugin

## 目的

このプラグインパネルはダミーの信号機信号をパブリッシュします。

## 入出力

### 出力

| 名称                                          | タイプ                                             | 説明                     |
| -------------------------------------------- | ------------------------------------------------ | -------------------------- |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 交通信号の公開             |

## 使い方

<div align="center">
  <img src="images/select_panels.png" width=50%>
</div>
<div align="center">
  <img src="images/select_traffic_light_publish_panel.png" width=50%>
</div>
<div align="center">
  <img src="images/select_traffic_light_id.png" width=50%>
</div>

1. rvizを起動して、パネル/新しいパネルを追加を選択します。
2. TrafficLightPublishPanelを選択してOKを押します。
3. `信号ID`と`信号状態`を設定し、`設定`ボタンを押します。
4. `発行`ボタンを押すと、信号が発行されます。

<div align="center">
  <img src="images/traffic_light_publish_panel.gif">
</div>

