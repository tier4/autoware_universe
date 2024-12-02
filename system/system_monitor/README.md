## Autoware向けシステムモニター

**Autowareのシステムモニター機能のさらなる向上**

## 説明

このパッケージは、システムのモニタリング用として以下のノードを提供します。

- CPUモニター
- HDDモニター
- メモリーモニター
- ネットワークモニター
- NTPモニター
- プロセスマニター
- GPUモニター
- 電圧モニター

### サポートされるアーキテクチャ

- x86_64
- arm64v8/aarch64

### 動作確認済みプラットフォーム

- PCシステムintel core i7
- NVIDIA Jetson AGX Xavier
- Raspberry Pi4 Model B

## 使用方法

他のパッケージと同様に、colconを使用してビルドと起動を行います。


```sh
colcon build
source install/setup.bash
ros2 launch system_monitor system_monitor.launch.xml
```

CPUおよびGPUのモニタリング方法は、プラットフォームによって異なります。<br>
CMakeは自動的に構築環境に基づいて構築するソースを選択します。<br>
このパッケージをIntelプラットフォームで構築する場合、Intelプラットフォーム上で実行されるCPUモニターとGPUモニターが構築されます。

## システムモニタが公開するROSトピック

各トピックは1分間隔で公開されます。

- [CPU Monitor](docs/topics_cpu_monitor.md)
- [HDD Monitor](docs/topics_hdd_monitor.md)
- [Mem Monitor](docs/topics_mem_monitor.md)
- [Net Monitor](docs/topics_net_monitor.md)
- [NTP Monitor](docs/topics_ntp_monitor.md)
- [Process Monitor](docs/topics_process_monitor.md)
- [GPU Monitor](docs/topics_gpu_monitor.md)
- [Voltage Monitor](docs/topics_voltage_monitor.md)

[使用] ✓:サポートされ、-:サポートされていない

| ノード | メッセージ | Intel | arm64(tegra) | arm64(raspi) | 備考 |
|---|---|---|---|---|---|
| CPU Monitor | CPU Temperature |   ✓   |       ✓      |       ✓      | |
| | CPU Usage |   ✓   |       ✓      |       ✓      | |
| | CPU Load Average |   ✓   |       ✓      |       ✓      | |
| | CPU Thermal Throttling |   ✓   |       -      |       ✓      | |
| | CPU Frequency |   ✓   |       ✓      |       ✓      | 周波数の通知のみで、通常エラーは生成されません。 |
| HDD Monitor | HDD Temperature |   ✓   |       ✓      |       ✓      | |
| | HDD PowerOnHours |   ✓   |       ✓      |       ✓      | |
| | HDD TotalDataWritten |   ✓   |       ✓      |       ✓      | |
| | HDD RecoveredError |   ✓   |       ✓      |       ✓      | |
| | HDD Usage |   ✓   |       ✓      |       ✓      | |
| | HDD ReadDataRate |   ✓   |       ✓      |       ✓      | |
| | HDD WriteDataRate |   ✓   |       ✓      |       ✓      | |
| | HDD ReadIOPS |   ✓   |       ✓      |       ✓      | |
| | HDD WriteIOPS |   ✓   |       ✓      |       ✓      | |
| | HDD Connection |   ✓   |       ✓      |       ✓      | |
| Memory Monitor | Memory Usage |   ✓   |       ✓      |       ✓      | |
| Net Monitor | Network Connection |   ✓   |       ✓      |       ✓      | |
| | Network Usage |   ✓   |       ✓      |       ✓      | 使用量の通知のみで、通常エラーは生成されません。 |
| | Network CRC Error |   ✓   |       ✓      |       ✓      | この期間の CRC エラーの数がしきい値に達すると、警告が発生します。発生する CRC エラーの数は、ip コマンドで確認できる値と同じです。 |
| | IP Packet Reassembles Failed |   ✓   |       ✓      |       ✓      | |
| NTP Monitor | NTP Offset |   ✓   |       ✓      |       ✓      | |
| Process Monitor | Tasks Summary |   ✓   |       ✓      |       ✓      | |
| | High-load Proc[0-9] |   ✓   |       ✓      |       ✓      | |
| | High-mem Proc[0-9] |   ✓   |       ✓      |       ✓      | |
| GPU Monitor | GPU Temperature |   ✓   |       ✓      |       -      | |
| | GPU Usage |   ✓   |       ✓      |       -      | |
| | GPU Memory Usage |   ✓   |       -      |       -      | |
| | GPU Thermal Throttling |   ✓   |       -      |       -      | |
| | GPU Frequency |   ✓   |       ✓      |       -      | インテルプラットフォームの場合、現在の GPU クロックが GPU によってサポートされているかどうかを監視します。 |
| Voltage Monitor | CMOS Battery Status |   ✓   |       -      |       -      | RTC および BIOS のバッテリの状態

## ROSパラメータ

[ROSパラメータ](docs/ros_parameters.md)を参照してください。

## メモ

### <u>Intelプラットフォーム用CPUモニタ</u>

サーマルスロットリングイベントはMSR(モデル固有レジスタ)の内容を読み取ることで監視できますが、MSRにアクセスするのはデフォルトではrootのみが許可されています。そこでこのパッケージでは、セキュリティリスクを最小限に抑えるための以下のアプローチを提供します。

- MSRにアクセスし、ソケットプログラミングを使用してサーマルスロットリングステータスをCPUモニタに送信する「msr_reader」という小規模なプログラムを提供します。
- rootではなく特定のユーザとして「msr_reader」を実行します。
- サーマルスロットリングステータスはソケット通信で送信されるため、CPUモニタは権限のないユーザとしてそのステータスを知ることができます。

### 開始前の注意事項

1. 「msr_reader」を実行するユーザを作成します。


   ```sh
   sudo adduser <username>
   ```

2. ターゲットシステムにカーネルモジュール「msr」を読み込みます。
   「/dev/cpu/CPUNUM/msr」パスが表示されます。


   ```sh
   sudo modprobe msr
   ```

3. アクセス制御リスト (ACL) を使用して、ユーザーが読み取り専用権限で MSR にアクセスできるようにする。


   ```sh
   sudo setfacl -m u:<username>:r /dev/cpu/*/msr
   ```

4. MSRカーネルモジュールにはrawio機能が必要なため、'msr_reader'に機能を割り当てます。


   ```sh
   sudo setcap cap_sys_rawio=ep install/system_monitor/lib/system_monitor/msr_reader
   ```

5. 作成したユーザーとして `msr_reader` を実行し、汎用ユーザーとして `system_monitor` を実行します。


   ```sh
   su <username>
   install/system_monitor/lib/system_monitor/msr_reader
   ```

### 参照

[msr_reader](docs/msr_reader.md)

## <u>HDD モニター</u>

一般的に、SMART 情報は HDD 温度と HDD の使用寿命を監視するために使用され、通常は root ユーザーまたはディスクグループがディスクデバイスノードにアクセスできます。<br>
CPU モニターと同様に、このパッケージは可能な限りセキュリティリスクを最小化するアプローチを提供します。<br>

- ソケットプログラミングを使用して SMART 情報にアクセスし、その一部を HDD モニターに送信する 'hdd_reader' という名前の小さなプログラムを提供します。
- 特定のユーザーとして 'hdd_reader' を実行します。
- HDD モニターはソケット通信によって送信されるため、権限のないユーザーとしても SMART 情報の一部を知ることができます。

### 開始前の手順

1. 'hdd_reader' を実行するユーザーを作成します。


   ```sh
   sudo adduser <username>
   ```

2. ユーザーをディスクグループに追加する。


   ```sh
   sudo usermod -a -G disk <username>
   ```

3. SCSIカーネルモジュールがATA PASS-THROUGH (12) コマンドを送信するために`rawio`ケイパビリティを必要とし、NVMeカーネルモジュールが管理コマンドを送信するために`admin`ケイパビリティを必要とするため、`hdd_reader`にケイパビリティを割り当てます。


   ```sh
   sudo setcap 'cap_sys_rawio=ep cap_sys_admin=ep' install/system_monitor/lib/system_monitor/hdd_reader
   ```

4. 作成したユーザーで「hdd_reader」を実行し、汎用ユーザーとしてsystem_monitorを実行します。


   ```sh
   su <username>
   install/system_monitor/lib/system_monitor/hdd_reader
   ```

### 関連情報

[hdd_reader](docs/hdd_reader.md)

## <u>Intel プラットフォームの GPU モニター</u>

現在、Intel プラットフォームの GPU モニターは、NVML API で情報にアクセスできる NVIDIA GPU にのみ対応しています。

また、CUDA ライブラリをインストールする必要があります。CUDA 10.0 のインストール手順については、[NVIDIA CUDA Installation Guide for Linux](https://docs.nvidia.com/cuda/archive/10.0/cuda-installation-guide-linux/index.html) を参照してください。

## <u>CMOS バッテリーの電圧モニター</u>

一部のプラットフォームは RTC と CMOS 用の内蔵バッテリーを備えています。このノードは、cat /proc/driver/rtc の実行結果からバッテリーの状態を判断します。
また、lm-sensors がインストールされている場合は、結果を使用できます。
ただし、sensors の戻り値はチップセットによって異なるため、対応する電圧を抽出するための文字列を設定する必要があります。
警告電圧とエラー電圧を設定する必要もあります。
例えば、電圧が 2.9V 未満のときに警告し、2.7V 未満のときにエラーが発生するように設定する場合は、次のようになります。
nct6106 チップセット上のセンサーの実行結果は次のとおりで、「in7:」は CMOS バッテリーの電圧です。


```txt
$ sensors
pch_cannonlake-virtual-0
Adapter: Virtual device
temp1:        +42.0°C

nct6106-isa-0a10
Adapter: ISA adapter
in0:           728.00 mV (min =  +0.00 V, max =  +1.74 V)
in1:             1.01 V  (min =  +0.00 V, max =  +2.04 V)
in2:             3.34 V  (min =  +0.00 V, max =  +4.08 V)
in3:             3.34 V  (min =  +0.00 V, max =  +4.08 V)
in4:             1.07 V  (min =  +0.00 V, max =  +2.04 V)
in5:             1.05 V  (min =  +0.00 V, max =  +2.04 V)
in6:             1.67 V  (min =  +0.00 V, max =  +2.04 V)
in7:             3.06 V  (min =  +0.00 V, max =  +4.08 V)
in8:             2.10 V  (min =  +0.00 V, max =  +4.08 V)
fan1:          2789 RPM  (min =    0 RPM)
fan2:             0 RPM  (min =    0 RPM)
```

`voltage_monitor.param.yaml` の設定値は以下の通りです。


```yaml
/**:
  ros__parameters:
    cmos_battery_warn: 2.90
    cmos_battery_error: 2.70
    cmos_battery_label: "in7:"
```

上で示した 2.7 V と 2.90 V の値は仮定上のものです。マザーボードとチップセットによっては、この値は異なる場合があります。ただし、リチウム電池の電圧が 2.7 V を下回る場合は、交換することを推奨します。
上記の例では、トピック /diagnostics に出力されるメッセージは次のとおりです。
電圧が 2.9 V 未満の場合:


```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: Warning
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: Low Battery
```

電圧が2.7V未満の場合は次のとおりです。


```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: Warning
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: Battery Died
```

どちらでもない場合：


```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: OK
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: OK
```

CMOSバッテリー電圧がvoltage_errorやvoltage_warnを下回ると警告が発生します。
バッテリーがなくなると、電源が切れたときにRTCの動作が停止します。ただし、車両が走行できるため、エラーではありません。車両はエラーが発生すると停止しますが、すぐに停止する必要はありません。
「ローバッテリー」または「バッテリー切れ」の値によって判断できます。

## UMLダイアグラム

[クラス図](docs/class_diagrams.md)を参照してください。
[シーケンス図](docs/seq_diagrams.md)を参照してください。

