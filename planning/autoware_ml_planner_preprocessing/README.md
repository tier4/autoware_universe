# ML-Planner preprocessing

This package builds raw ML-Planner input features directly from a T4Scene window. Its C++ library can be used in a CPU-only build with a ROS 2 Autoware dependency workspace. It uses the T4Scene v0.0.0 SDK and the Lanelet2 map geometry referenced by each window.

Import [the ML-Planner dependency manifest](../autoware_ml_planner/t4scene.repos) into a workspace
with access to the pinned T4Scene repository. This manifest is for authenticated builds.

## C++

Use `find_package(autoware_ml_planner_preprocessing REQUIRED)` and link `autoware_ml_planner_preprocessing::autoware_ml_planner_preprocessing`. Include `<autoware/ml_planner/preprocessing/scene_input_builder.hpp>`. Call `build_map_context` once per Lanelet2 map and reuse the context when calling `build_scene_input` for each window. Provide a 10 Hz Scene with 30 history rows before `current_index`, the ego track and vehicle dimensions, route and map registration. The converter returns a `tl::expected` with input tensors, selected track IDs in neighbor tensor order, and the ego-to-map transform.

`TrafficLightFuture::Infer` is the online policy. Offline label preparation can use `TrafficLightFuture::Recorded` with 80 future rows of traffic-light observations. Both modes build the remaining input features using the current and historical rows. Apply `normalize_input_data` and supply the model's initial noise separately when running inference.

## Generate Scenes from a recording

For raw recordings, call `t4scene_generation.SceneBuilder.open_bag` with the planner's installed
`config/t4scene_online.yaml`. Build each window at its decision timestamp. For a replay with
known receipt times, supply the decision's receipt cutoff to `build_window`. The recording's map
geometry, map registration, and vehicle dimensions are attached before converting the Scene.
Evaluation may use a copy of the profile with 80 future steps when it needs recorded signal data.

## Python

Build T4Scene with `-DT4SCENE_WITH_MAPS=ON` and build this package with `-DAUTOWARE_ML_PLANNER_PREPROCESSING_PYTHON=ON`. Source the installed workspace before importing the module. Use a Python interpreter compatible with the one used to build the module.

```python
from pathlib import Path

from autoware_ml_planner_preprocessing import SceneInputBuilder

builder = SceneInputBuilder(Path("map_resource.pb").read_bytes())
result = builder.build(Path("scene.pb").read_bytes())
tensors = result["tensors"]
```

Reuse one builder for windows with the same map resource. It decodes the map and builds the lane context once. Export the map as `LANELET2_OSM` with the Python T4Scene `MapResourceStore(map_encoding="lanelet2_osm")` or `LANELET2_BIN` with the C++ SDK. The T4Scene protobuf must reference the exported resource by its SHA-256.

The tensor dictionary holds owned `float32` NumPy arrays with the same names and shapes as the C++ output. The result also includes `selected_track_ids` and the `float64` matrix `ego_to_map`. Pass `normalize=True` to apply the same scaling used by the online node.
