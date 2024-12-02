## autoware_component_monitor

`autoware_component_monitor`パッケージではコンポーネントコンテナのシステム使用量をモニターできます。
このパッケージ内のコンポーザブルノードはコンポーネントコンテナにアタッチされ、コンテナのCPUおよびメモリ使用率をパブリッシュします。

## 入力 / 出力

### 入力

なし。

### 出力

| 名称                       | タイプ                                               | 説明            |
| -------------------------- | -------------------------------------------------- | ---------------------- |
| `~/component_system_usage` | `autoware_internal_msgs::msg::ResourceUsageReport` | CPU、メモリ使用率など |

## パラメータ

### コアパラメータ

{{ json_to_markdown("system/autoware_component_monitor/schema/component_monitor.schema.json") }}

## 使用方法

起動ファイルに構成可能なノードとして追加します:


```xml

<launch>
  <group>
    <push-ros-namespace namespace="your_namespace"/>
    ...

    <load_composable_node target="$(var container_name)">
      <composable_node pkg="autoware_component_monitor"
                       plugin="autoware::component_monitor::ComponentMonitor"
                       name="component_monitor">
        <param from="$(find-pkg-share autoware_component_monitor)/config/component_monitor.param.yaml"/>
      </composable_node>
    </load_composable_node>

    ...
  </group>
</launch>
```

## クイックテスト 

以下のコマンドを実行することでパッケージをテストできます。


```bash
ros2 component load <container_name> autoware_component_monitor autoware::component_monitor::ComponentMonitor -p publish_rate:=10.0 --node-namespace <namespace>

# Example usage
ros2 component load /pointcloud_container autoware_component_monitor autoware::component_monitor::ComponentMonitor -p publish_rate:=10.0 --node-namespace /pointcloud_container
```

## 動作原理

このパッケージは内部で `top` コマンドを使用します。
`top -b -n 1 -E k -p PID` コマンドは 10 Hz で実行され、そのプロセスに対するシステムの使用率を取得します。

- `-b` はバッチモードをアクティブにします。デフォルトでは `top` は終了せずに定期的に stdout に出力します。バッチモードではプログラムの終了が許可されます。
- `-n` は `top` がバッチモードでシステムの使用率を出力する回数です。
- `-p` はモニタリングするプロセスの PID を指定します。
- `-E k` はサマリーセクションのメモリ単位を KiB に変更します。

出力例を以下に示します。


```text
top - 13:57:26 up  3:14,  1 user,  load average: 1,09, 1,10, 1,04
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s):  0,0 us,  0,8 sy,  0,0 ni, 99,2 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
KiB Mem : 65532208 total, 35117428 free, 17669824 used, 12744956 buff/cache
KiB Swap: 39062524 total, 39062524 free,        0 used. 45520816 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   3352 meb       20   0 2905940   1,2g  39292 S   0,0   2,0  23:24.01 awesome
```

最終行から5番目と8番目のフィールド（それぞれ`RES`と`%CPU`）を取得します。

