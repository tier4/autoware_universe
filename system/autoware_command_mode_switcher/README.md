# autoware_command_mode_switcher

## Overview

This package activates the selected command mode from the decider node.
The activation process for each command mode is implementation dependent, so extensions using plugins are supported.
By providing a specified interface, each plugin can operate under the appropriate state transitions.

## Operation Mode Transition Conditions

| 条件                        | 説明                                                    |
| --------------------------- | ------------------------------------------------------- |
| Command Mode Available      | 対象モードのコマンドへの切り替えを実施してよいか。      |
| Command Mode Continuable    | 対象モードのコマンドが選択済みの場合に継続できるか。    |
| Command Mode Completed      | 対象モードへの切り替えが完了して責任を責任を負えるか。  |
| Command Mode Transition     | モード間遷移の条件。現状サポートしてないので不要。      |
| Autoware Control Transition | Autoware Control を有効にできる車両状態になっているか。 |

## Operation Mode Transition Sequence

![switcher-state](./doc/switcher-sequence.drawio.svg)
