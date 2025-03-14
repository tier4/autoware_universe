# autoware_command_mode_switcher

## Overview

This package activates the selected command mode from the decider node.
The activation process for each command mode is implementation dependent, so extensions using plugins are supported.
By providing a specified interface, each plugin can operate under the appropriate state transitions.

## Operation Mode Transition Conditions

Command Mode 系の条件については対象モードのコマンドを Autoware が提供できるかを表している。

- Command Mode Available
- Command Mode Continuable

Transition 系の条件についてはモード同士の遷移における責任の所在を明確にするために設けられている。

- Transition Prepared
- Transition Completed

## Mode Transition

![operation-mode](./doc/operation-mode.drawio.svg)

特定のモードに注目した時、そのモードに関する遷移は以下の５つに分類される。

| 遷移                               | 説明                                                                      |
| ---------------------------------- | ------------------------------------------------------------------------- |
| autoware control enable            | Autoware 制御を有効にする。走行中エンゲージ。                             |
| autoware control disable           | Autoware 制御を無効にする。オーバーライド。                               |
| operation mode enable (foreground) | Autoware 制御を有効にしたまま Operation Mode を変更する。                 |
| operation mode enable (background) | Autoware 制御を無効にしたまま Operation Mode を変更する。                 |
| operation mode disable             | Autoware 制御の状態に関わらず他の Operation Mode の enable が実行された。 |

## State Transition

![switcher-state](./doc/switcher-state.drawio.svg)

Mode Transition の分類に応じて以下の遷移を行う。

| 遷移                               | 対象        | 条件                                            |
| ---------------------------------- | ----------- | ----------------------------------------------- |
| autoware control enable            | C-EN        | Operation Mode Available && Transition Prepared |
| autoware control disable           | C-DIS       | Always                                          |
| operation mode enable (foreground) | M-EN, C-RST | Operation Mode Available && Transition Prepared |
| operation mode enable (background) | M-EN        | Operation Mode Available                        |
| operation mode disable             | M-DIS       | Always                                          |

| 遷移 | 条件                                                                                          |
| ---- | --------------------------------------------------------------------------------------------- |
| C-A1 | 対象モードがENABLE、かつ、他のモードが全てDISABLE (例えばVelocityLimit解除などを保証するため) |
| C-A2 | ControlMode=True, Autoware Control Completed が継続                                           |
| C-A3 | ControlMode=False                                                                             |
| C-OR | ControlMode=False (オーバーライド)                                                            |
