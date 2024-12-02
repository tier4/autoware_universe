# autoware_universe_utils

## 目的

このパッケージには他のパッケージで一般的に使用される関数が多数含まれています。必要に応じてそれらを参照してください。

## 開発者向け

`autoware_universe_utils.hpp` ヘッダーファイルは、このファイルを直接または間接的にインクルードするソースファイルの前処理に時間がかかるため、削除されました。

## `autoware::universe_utils`

### `systems`

#### `autoware::universe_utils::TimeKeeper`

##### コンストラクタ


```cpp
template <typename... Reporters>
explicit TimeKeeper(Reporters... reporters);
```

- `TimeKeeper` の初期化にリポーターのリストを使用。

##### メソッド

- `void add_reporter(std::ostream * os);`

  - 処理時間を `ostream` に出力するためのリポーターを追加。
  - `os`: `ostream` オブジェクトへのポインタ。

- `void add_reporter(rclcpp::Publisher<ProcessingTimeDetail>::SharedPtr publisher);`

  - 処理時間を `rclcpp` パブリッシャーへ発行するためのリポーターを追加。
  - `publisher`: `rclcpp` パブリッシャーの共有ポインタ。

- `void add_reporter(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);`

  - 処理時間を `std_msgs::msg::String` を持つ `rclcpp` パブリッシャーへ発行するためのリポーターを追加。
  - `publisher`: `rclcpp` パブリッシャーの共有ポインタ。

- `void start_track(const std::string & func_name);`

  - 関数の処理時間の追跡を開始。
  - `func_name`: 追跡対象の関数の名前。

- `void end_track(const std::string & func_name);`

  - 関数の処理時間の追跡を終了。
  - `func_name`: 追跡を終了する関数の名前。

- `void comment(const std::string & comment);`
  - 追跡中の現在の関数に対してコメントを追加。
  - `comment`: 追加するコメント。

##### 注意

- 処理時間の測定は、以下のように `start_track` と `end_track` を使用して開始および終了できます。


  ```cpp
  time_keeper.start_track("example_function");
  // Your function code here
  time_keeper.end_track("example_function");
  ```

- 安全性と適切な追跡を確保するために、`ScopedTimeTrack` の使用が推奨されます。

##### 例


```cpp
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode() : Node("time_keeper_example")
  {
    publisher_ =
      create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 1);

    time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(publisher_, &std::cerr);
    // You can also add a reporter later by add_reporter.
    // time_keeper_->add_reporter(publisher_);
    // time_keeper_->add_reporter(&std::cerr);

    timer_ =
      create_wall_timer(std::chrono::seconds(1), std::bind(&ExampleNode::func_a, this));
  }

private:
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_str_;
  rclcpp::TimerBase::SharedPtr timer_;

  void func_a()
  {
    // Start constructing ProcessingTimeTree (because func_a is the root function)
    autoware::universe_utils::ScopedTimeTrack st("func_a", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    time_keeper_->comment("This is a comment for func_a");
    func_b();
    // End constructing ProcessingTimeTree. After this, the tree will be reported (publishing
    // message and outputting to std::cerr)
  }

  void func_b()
  {
    autoware::universe_utils::ScopedTimeTrack st("func_b", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    time_keeper_->comment("This is a comment for func_b");
    func_c();
  }

  void func_c()
  {
    autoware::universe_utils::ScopedTimeTrack st("func_c", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    time_keeper_->comment("This is a comment for func_c");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## 自動運転ソフトウェアのドキュメント

### 出力 (コンソール)


  ```text
  ==========================
  func_a (6.243ms) : This is a comment for func_a
      └── func_b (5.116ms) : This is a comment for func_b
          └── func_c (3.055ms) : This is a comment for func_c
  ```

- 出力（`ros2 topic echo /processing_time`）


  ```text
  ---
  nodes:
  - id: 1
    name: func_a
    processing_time: 6.366
    parent_id: 0
    comment: This is a comment for func_a
  - id: 2
    name: func_b
    processing_time: 5.237
    parent_id: 1
    comment: This is a comment for func_b
  - id: 3
    name: func_c
    processing_time: 3.156
    parent_id: 2
    comment: This is a comment for func_c
  ```

#### `autoware::universe_utils::ScopedTimeTrack`

##### 説明

スコープ内で関数の処理時間を自動的に追跡するためのクラスです。

##### コンストラクタ


```cpp
ScopedTimeTrack(const std::string & func_name, TimeKeeper & time_keeper);
```

- `func_name`: 追跡される関数の名前
- `time_keeper`: `TimeKeeper`オブジェクトへの参照

##### デストラクタ


```cpp
~ScopedTimeTrack();
```

- `ScopedTimeTrack` オブジェクトを破棄し、関数の追跡を終了します。

