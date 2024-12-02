## テンプレート

autoware_behavior_velocity_speed_bump_module に基づいた動作速度モジュールのテンプレート。

# Autoware 動作速度モジュール テンプレート

## `Scene`

### `TemplateModule` クラス

`TemplateModule` クラスは、Autoware 動作速度プランナー内でシーンモジュールを作成するための基盤として機能します。モジュールの動作に必要なコアメソッドと機能を定義しています。プレースホルダーコードを、特定の動作速度モジュールに合わせた実際の実装に置き換える必要があります。

#### コンストラクター

- `TemplateModule` のコンストラクターはモジュールを作成するための必須パラメーターを受け取ります: `const int64_t module_id`, `const rclcpp::Logger & logger`, and `const rclcpp::Clock::SharedPtr clock`. これらのパラメーターは、新しいモジュールを登録するときに `TemplateModuleManager` によって供給されます。特定のモジュール実装で必要に応じて、他のパラメーターをコンストラクターに追加できます。

#### `modifyPathVelocity` メソッド

- `TemplateModule` クラスで定義されているこのメソッドは、特定の条件に基づいて入力パスの速度を変更すると想定されています。提供されているコードでは、テンプレートモジュールが実行されるときに一度情報メッセージを記録します。
- 速度変更の具体的なロジックは、モジュールの要件に基づいて、このメソッド内で実装する必要があります。

#### `createDebugMarkerArray` メソッド

- `TemplateModule` クラスで定義されているこのメソッドは、デバッグマーカーを視覚化して `visualization_msgs::msg::MarkerArray` として返す役割を持ちます。提供されているコードでは、空の `MarkerArray` を返します。
- モジュールの機能に固有のデバッグマーカーを生成するロジックを実装する必要があります。

#### `createVirtualWalls` メソッド

- `createVirtualWalls` メソッドは、シーンの仮想壁を作成し、それらを `autoware::motion_utils::VirtualWalls` として返します。提供されているコードでは、空の `VirtualWalls` オブジェクトを返します。
- モジュールの要件に基づいて、仮想壁を作成するロジックを実装する必要があります。

## `Manager`

モジュールの管理は manager.hpp と manager.cpp で定義されています。管理は次の 2 つのクラスによって処理されます。

- `TemplateModuleManager` クラスは、behavior_velocity_template シーン (behavior_velocity_template_module/src/scene.cpp/hpp で定義) を管理して起動するためのコアロジックを定義します。親クラス `SceneModuleManagerInterface` から必須マネージャ属性を継承します。
- `TemplateModulePlugin` クラスは、`TemplateModuleManager` を動作速度プランナーのロジックに統合するための方法を提供します。

### `TemplateModuleManager` クラス

#### コンストラクター `TemplateModuleManager`

- これは `TemplateModuleManager` クラスのコンストラクターで、`rclcpp::Node` 参照をパラメーターとして受け取ります。
- メンバー変数 `dummy_parameter_` を 0.0 に初期化します。

#### `getModuleName()` メソッド

- これは `SceneModuleManagerInterface` クラスの仮想メソッドをオーバーライドしたものです。
- モジュールの名前である constant 文字列へのポインターを返します。この場合、「template」をモジュール名として返します。

#### `launchNewModules()` メソッド

- これは `tier4_planning_msgs::msg::PathWithLaneId` 型の引数を取るプライベートメソッドです。
- 提供されたパス情報 (PathWithLaneId) に基づいて新しいモジュールを起動します。このメソッドの実装には、`TemplateModule` クラスを使用して動作速度プランナー固有のモジュールを初期化して構成することが含まれます。
- 提供されたソースコードでは、`module_id` を 0 に初期化し、同じ ID を持つモジュールがすでに登録されているかどうかを確認します。登録されていない場合は、`TemplateModule` を新しいモジュール ID で登録します。`TemplateModuleManager` によって管理される各モジュールは固有の ID を持つ必要があることに注意してください。テンプレートコードは単一のモジュールを登録するため、`module_id` は単純化するために 0 に設定されています。

#### `getModuleExpiredFunction()` メソッド

- これは `tier4_planning_msgs::msg::PathWithLaneId` 型の引数を取るプライベートメソッドです。

- このメソッドの実装は、モジュールの有効期限ステータスを確認するために使用できる関数を返すことが期待されます。

`launchNewModules()` および `getModuleExpiredFunction()` メソッドの具体的な機能は、行動速度モジュールの詳細と、それらが Autoware システム内でどのように管理されるかによって異なります。モジュールの要件に従ってこれらのメソッドを実装する必要があります。

### `TemplateModulePlugin` クラス

#### `TemplateModulePlugin` クラス

- このクラスは `PluginWrapper<TemplateModuleManager>` から継承します。これは基本的に `TemplateModuleManager` クラスをプラグイン内にラップし、動的にロードおよび管理できます。

## `Example Usage`

次の例では、パスの各点を取得して 2 倍します。速度を本質的に複製します。すべての動作速度モジュールが実行されると、速度スムージングによりパスの速度がさらに変更されることに注意してください。


```cpp
bool TemplateModule::modifyPathVelocity(
  [[maybe_unused]] PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  for (auto & p : path->points) {
    p.point.longitudinal_velocity_mps *= 2.0;
  }

  return false;
}
```

