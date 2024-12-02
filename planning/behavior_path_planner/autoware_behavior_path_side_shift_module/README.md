# 横ずれ設計

（リモート制御用）外部からの指示に従って経路を左または右にシフトします。

## Side Shiftモジュールプロセスの概要

1. 必要な横ずれオフセット入力を受信する。
2. 以下の条件を満たす場合に`requested_lateral_offset_`を更新する。
    a. 最後の更新時刻が経過しているかどうかを確認する。
    b. 必要な横ずれオフセット値が前の値と異なっていることを確認する。
3. Side ShiftモジュールのステータスがSHIFTINGステータスでなければ、シフトポイントをパスに挿入する。

`requested_lateral_offset_`は最新の値で継続的に更新され、キューに入れられないことに注意してください。

## Side Shiftのステータス

Side Shiftには3つの異なるステータスがあります。SHIFTINGステータス中はパスを更新できないことに注意してください。

1. BEFORE_SHIFT：シフトの準備。
2. SHIFTING：現在シフト中です。
3. AFTER_SHIFT：シフト完了。

<figure markdown>
  ![case1](images/side_shift_status.drawio.svg){width=1000}
  <figcaption>side shiftステータス</figcaption>
</figure>

## フローチャート


```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title callback function of lateral offset input
start

partition onLateralOffset {
:**INPUT** double new_lateral_offset;

if (abs(inserted_lateral_offset_ - new_lateral_offset) < 1e-4) then ( true)
  stop
else ( false)
  if (interval from last request is too short) then ( no)
  else ( yes)
  :requested_lateral_offset_ = new_lateral_offset \n lateral_offset_change_request_ = true;
  endif
stop
@enduml
```


```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title path generation

start
partition plan {
if (lateral_offset_change_request_ == true \n && \n (shifting_status_ == BEFORE_SHIFT \n || \n shifting_status_ == AFTER_SHIFT)) then ( true)
  partition replace-shift-line {
    if ( shift line is inserted in the path ) then ( yes)
      :erase left shift line;
    else ( no)
    endif
    :calcShiftLines;
    :add new shift lines;
    :inserted_lateral_offset_ = requested_lateral_offset_ \n inserted_shift_lines_ = new_shift_line;
  }
else( false)
endif
stop
@enduml
```


```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title update state

start
partition updateState {
  :last_sp = path_shifter_.getLastShiftLine();
  note left
  get furthest shift lines
  end note
  :calculate max_planned_shift_length;
  note left
  calculate furthest shift length of previous shifted path
  end note
  if (abs(inserted_lateral_offset_ - inserted_shift_line_.end_shift_length) < 1e-4 \n && \n abs(max_planned_shift_length) < 1e-4 \n && \n abs(requested_lateral_offset_) < 1e-4) then ( true)
    :current_state_ = BT::NodeStatus::SUCCESS;
  else (false)
    if (ego's position is behind of shift line's start point) then( yes)
      :shifting_status_ = BEFORE_SHIFT;
    else ( no)
      if ( ego's position is between shift line's start point and end point) then (yes)
        :shifting_status_ = SHIFTING;
      else( no)
        :shifting_status_ = AFTER_SHIFT;
      endif
    endif
    :current_state_ = BT::NodeStatus::RUNNING;
  endif
  stop
}

@enduml
```

