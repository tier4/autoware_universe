### 禁止駐車エリア

#### 役割

このモジュールは、「禁止駐車エリア」での停止を回避するために計画を立てます。

![brief](./docs/no-stopping-area.svg)

- 通過ケース
  - 自動運転車が通過判断点を通過した場合、自動運転車は最大のジャークと加速度で停止できないため、このモジュールは停止速度を挿入しません。この場合、オーバーライドまたは外部操作が必要です。
- 停止ケース
  - 「禁止駐車エリア」の周囲に停止車両または停止速度がある場合、車両は「禁止駐車エリア」内で停止するため、このモジュールは「禁止駐車エリア」の手前に停止速度を作成します。
- 走行ケース
  - その他

### 制限

このモジュールにより、開発者は特定のルールを使用して「禁止駐車エリア」モジュールにおける車両速度を設計できます。自動運転車が通過点を通過すると、自動運転車は停止速度を挿入せず、走行するという決定を変更しません。また、このモジュールは不要な停止を避けるために動的物体のみを考慮します。

#### ModelParameter

| パラメータ名                   | 型     | 説明                                                               |
| ---------------------------- | ------ | ------------------------------------------------------------------- |
| `state_clear_time`           | double | [秒] 停止状態を解除する時間                                        |
| `stuck_vehicle_vel_thr`      | double | [m/秒] この速度以下の車両は停止車両とみなされます。               |
| `stop_margin`                | double | [m] 停止線がないエリアでの停止線に対する余裕                     |
| `dead_line_margin`           | double | [m] 自動車がこの位置を通過したらGO                                |
| `stop_line_margin`           | double | [m] 停止線がないエリアでの自動生成された停止線に対する余裕       |
| `detection_area_length`      | double | [m] ポリゴンを検索する長さ                                         |
| `stuck_vehicle_front_margin` | double | [m] 停止車両との最大距離                                           |

#### フローチャート


```plantuml
@startuml
title modifyPathVelocity
start

if (ego path has "no stopping area" ?) then (yes)
else (no)
  stop
endif

partition pass_through_condition {
if (ego vehicle is not after dead line?) then (yes)
else (no)
  stop
endif
if (ego vehicle is stoppable before stop line consider jerk limit?) then (yes)
else (no)
  stop
endif
}
note right
  - ego vehicle is already over dead line(1.0[m] forward stop line) Do Not Stop.
  - "pass through or not" considering jerk limit is judged only once to avoid chattering.
end note

:generate ego "stuck_vehicle_detect_area" polygon;
note right
"stuck_vehicle_detect_area" polygon includes space of
 vehicle_length + obstacle_stop_max_distance
 after "no stopping area"
end note

:generate ego "stop_line_detect_area" polygon;
note right
"stop_line_detect_area" polygon includes space of
 vehicle_length + margin
 after "no stopping area"
end note

:set current judgement as GO;
if (Is stuck vehicle inside "stuck_vehicle_detect_area" polygon?) then (yes)
note right
only consider stuck vehicle following condition.
- below velocity 3.0 [m/s]
- semantic type of car bus truck or motorbike
only consider stop line as following condition.
- low velocity that is in path with lane id is considered.
end note
if (Is stop line inside "stop_line_detect_area" polygon?) then (yes)
  :set current judgement as STOP;
endif
endif

partition set_state_with_margin_time {

if (current judgement is same as previous state) then (yes)
  :reset timer;
else if (state is GO->STOP) then (yes)
  :set state as STOP;
  :reset timer;
else if (state is STOP -> GO) then (yes)
  if (start time is not set) then (yes)
    :set start time;
  else(no)
   :calculate duration;
   if(duration is more than margin time)then (yes)
    :set state GO;
    :reset timer;
  else(no)
   endif
  endif
else(no)
endif

}

note right
  - it takes 2 seconds to change state from STOP -> GO
  - it takes 0 seconds to change state from GO -> STOP
  - reset timer if no state change
end note

if (state is STOP) then (yes)
  :set stop velocity;
  :set stop reason and factor;
  else(no)
endif
stop


@enduml
```

