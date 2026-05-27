# Data-Driven Simulator Development TODO Status

このファイルは `data-driven-sim-dev` plan の進捗を、実装済み範囲と未完了範囲が分かるように記録する。

## Summary

現在の実装は、`bag -> dataset -> baseline runtime -> evaluator -> deterministic learned artifact -> runtime learned inference` の最小縦通しのうち、手動 param 指定による runtime learned inference までの経路を実装している。

ただし、plan に書かれている全 Phase のうち、Koopman / GP residual / physics-informed constraints などの advanced backend は未実装である。また、sequence model は現時点では placeholder artifact に近く、history-window feature の生成、時系列学習、GRU/LSTM/TCN のような recurrent backend は今後の追加実装対象である。

## TODO Status

| ID | Plan item | Status | Notes |
| --- | --- | --- | --- |
| `create-package-skeleton` | Create package skeleton with CMake, package.xml, launch, param, config, scripts, and tests directories | Done | `autoware_data_driven_planning_simulator` package を追加済み。 |
| `implement-dataset-extractor` | Implement ROS 2 sqlite3/MCAP bag reader, topic synchronization, resampling, filtering, metadata, and data validation report generation | Minimal Done | `rosbag2_py` reader、storage id 判定、dataset CSV/NPZ/YAML、maneuver validation を実装済み。厳密な interpolation、latency estimation、全 filter の詳細実装は拡張余地あり。 |
| `implement-baseline-models` | Implement exact/RK4 2D baseline models for Ackermann, differential/skid-steer, and holonomic vehicles with tests | Done | C++ / Python に exact constant body twist integration を実装済み。C++ unit test あり。 |
| `implement-runtime-node` | Implement component runtime simulator node with Autoware-style outputs, TF ownership, reset service, diagnostics, and fallback behavior | Minimal Done | Component node、odometry、velocity、steering、TF、reset service、baseline fallback を実装済み。Diagnostics は今後拡張。 |
| `implement-offline-evaluator` | Implement one-step and short-horizon rollout evaluator comparing simple baseline, vehicle-profile baseline, and learned model outputs | Done | no-motion、vehicle-profile baseline、linear/MLP learned artifact の one-step / multi-step rollout 経路、horizon metrics、segment-wise report を実装済み。baseline rollout、linear learned artifact rollout、MLP learned artifact rollout、horizon skip、segment boundary、不等間隔 timestamp、CLI horizon validation、segment-wise report 受入項目、synthetic bag 相当 dataset の unit test あり。実運用 rosbag での評価は外部入力が必要な manual integration verification として扱う。 |
| `implement-training-artifacts` | Implement deterministic MLP direct/residual/sequence training, normalization, model artifact metadata, and export path | Minimal Done | deterministic linear direct、MLP direct、MLP residual、normalization、metadata、CSV/NPZ export を実装済み。sequence_model は artifact placeholder であり、history-window feature 生成や時系列 backend は未実装。 |
| `add-runtime-inference` | Add deterministic CPU runtime inference backend with fixed provider/thread metadata and baseline fallback | Minimal Done | `linear_cpu` と `numpy_mlp_cpu` の runtime inference、baseline fallback を実装済み。ただし `model_artifact.yaml` を runtime が直接解釈せず、MLP は CSV path の個別 param 指定が必要。provider/thread metadata は artifact に記録されるが、runtime enforce は未実装。 |
| `add-fixtures-tests-docs` | Add sqlite3/MCAP fixtures, C++/Python tests, vehicle profile examples, and developer documentation | Minimal Done | synthetic bag generator、fixture README、C++/Python tests、vehicle profiles、README を追加済み。実 binary fixture は commit せず生成式。 |

## Completed Verification

以下は実行済み。

```bash
PYTHONPATH=src/autoware/universe/simulator/autoware_data_driven_planning_simulator \
  python3 -m pytest -q \
  src/autoware/universe/simulator/autoware_data_driven_planning_simulator/test/test_dataset_tools.py
```

Result:

```text
13 passed
```

```bash
CCACHE_DISABLE=1 colcon --log-base /tmp/ddsim_log build \
  --base-paths src/autoware/autoware_msgs/autoware_vehicle_msgs \
               src/autoware/autoware_msgs/autoware_control_msgs \
               src/autoware/universe/simulator/autoware_data_driven_planning_simulator \
  --build-base /tmp/ddsim_build \
  --install-base /tmp/ddsim_install \
  --packages-up-to autoware_data_driven_planning_simulator
```

Result:

```text
autoware_data_driven_planning_simulator build succeeded
```

```bash
CCACHE_DISABLE=1 colcon --log-base /tmp/ddsim_log test \
  --base-paths src/autoware/autoware_msgs/autoware_vehicle_msgs \
               src/autoware/autoware_msgs/autoware_control_msgs \
               src/autoware/universe/simulator/autoware_data_driven_planning_simulator \
  --build-base /tmp/ddsim_build \
  --install-base /tmp/ddsim_install \
  --packages-select autoware_data_driven_planning_simulator
```

Result:

```text
18 tests, 0 errors, 0 failures, 0 skipped
```

上記の検証は unit / minimal smoke test の範囲であり、以下は未検証である。

- `model_artifact.yaml` から runtime param への自動ロード。
- MLP artifact の C++ runtime load / inference の end-to-end test。
- ユーザー提供の実運用 rosbag を使った sqlite3 / MCAP integration verification。

## Remaining Work

次に進めるべき作業は以下。

1. Dataset extractor の補強
   - command latency estimation の実装。
   - time continuity / command freshness / outlier filters の詳細化。
   - state interpolation と command hold の厳密化。
   - sqlite3 / MCAP generated bag を使った integration test。

2. Evaluator の補強
   - ユーザー提供の実運用 rosbag 由来 dataset での 1 s / 3 s / 5 s / 10 s horizon metrics manual verification。
   - 評価対象車両ごとの segment-wise report 受入閾値の定義。

3. Learned models の補強
   - history window generation。
   - recurrent / temporal convolution backend。
   - artifact schema の versioning。

4. Runtime inference の補強
   - `model_artifact.yaml` から runtime param への自動ロード。
   - inference latency / timeout / fallback diagnostics。
   - reset event topic or diagnostics。

5. Advanced backend
   - Koopman / EDMDc offline backend。
   - GP residual backend。
   - physics-informed constraints。

