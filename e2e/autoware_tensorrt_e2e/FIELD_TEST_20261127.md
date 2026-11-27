# Field test 2026-11-27: four E2E models on the Japan taxi (step-by-step)

This is the step-by-step manual for the person running today's test. Every block can be copied as is. Background, the
list of nodes and how the build was verified are in [`RELEASE.md`](../../RELEASE.md) at the root of this branch.

One build runs all four models. Switching model = restarting Autoware with a different `e2e_model:=...`. Nothing is
rebuilt between models.

## The four models

| No. | Model | Directory under `data_path` (= the `e2e_model` value) | WebAuto package release id |
|---|---|---|---|
| **1** | ResWorld / OnePlanner 1.4.4.4 | `OnePlanner` | `5db45bcf-8829-49f7-ba5d-a389c8353091` |
| **2** | ML-Planner 2-stage A | `mlplanner-2stage-a` | `6c701579-ca84-407c-a5b3-0454f5994590` |
| **3** | ML-Planner 2-stage B | `mlplanner-2stage-b` | `87efa0a9-30d5-46cc-ae7d-f36b7cc72168` |
| **4** | ML-Planner 1.5-stage B | `mlplanner-1p5stage-b` | `33ed57d8-45dc-475a-904d-73a4b3b9a90b` |

Keep the directory names exactly as written. The launch file picks the planner from the name: a name containing
`mlplanner-2stage` starts the 2-stage planner, `mlplanner-1p5stage` the 1.5-stage planner, anything else ResWorld.
Model 1 must be called `OnePlanner`, because models 2 and 3 also read it (their detections come from the ResWorld
node).

WebAuto: project `prd_jt`, ML package `OnePlanner` (`22d5712f-c615-4135-a50b-ede4ab2764fc`). The release names are
`20261125_resworld__oneplanner-1.4.4.4-jpntaxi` and `20261125_mlplanner__oneplanner-mlplanner-{2stage-a,2stage-b,1p5stage-b}-jpntaxi`.

## 0. Variables used below

Set these in every new terminal (adjust the two paths to the car):

```bash
WS=~/pilot-auto.xx1                 # the car's Autoware workspace (the directory with src/ build/ install/)
DATA=$HOME/autoware_data            # the car's data_path (the launch default); the current ResWorld files are in $DATA/OnePlanner
DL=$HOME/e2e_models_1127            # where the WebAuto downloads go
```

Check `DATA` is right: `ls $DATA/OnePlanner` must list `resworld_planner.simplified.onnx` (the model the car ran last
time). If it does not, the car passes another `data_path` in its launch command: use that one.

## 1. Download the four models from WebAuto

On the car, or on a PC with the `webauto` CLI logged in (`webauto auth login`), then copy `$DL` to the car.

```bash
P=22d5712f-c615-4135-a50b-ede4ab2764fc
webauto ml package-release pull --project-id prd_jt --package-id $P --package-release-id 5db45bcf-8829-49f7-ba5d-a389c8353091 --target-dir $DL/1
webauto ml package-release pull --project-id prd_jt --package-id $P --package-release-id 6c701579-ca84-407c-a5b3-0454f5994590 --target-dir $DL/2
webauto ml package-release pull --project-id prd_jt --package-id $P --package-release-id 87efa0a9-30d5-46cc-ae7d-f36b7cc72168 --target-dir $DL/3
webauto ml package-release pull --project-id prd_jt --package-id $P --package-release-id 33ed57d8-45dc-475a-904d-73a4b3b9a90b --target-dir $DL/4
```

Each release lands in `<target-dir>/<release id>/OnePlanner/` (all four use the folder name `OnePlanner`, the package
name), so the next step renames them.

## 2. Put the models in place

```bash
# keep whatever ResWorld version is there now (restore it with the reverse mv)
[ -d $DATA/OnePlanner ] && mv $DATA/OnePlanner $DATA/OnePlanner.before-1127

cp -r $DL/1/*/OnePlanner $DATA/OnePlanner
cp -r $DL/2/*/OnePlanner $DATA/mlplanner-2stage-a
cp -r $DL/3/*/OnePlanner $DATA/mlplanner-2stage-b
cp -r $DL/4/*/OnePlanner $DATA/mlplanner-1p5stage-b

# the first start of each model writes its TensorRT engines next to the ONNX files
chmod -R u+w $DATA/OnePlanner $DATA/mlplanner-*
```

Check every file (all lines must say `OK`):

```bash
cd $DATA && sha256sum -c <<'EOF'
4b5e9f4f64c5aa5b156ae10c31ca0d75169ac6e033e15e7d5dbff1703d8dbb92  OnePlanner/resworld_planner.simplified.onnx
d701f6c37b44af6a04deab43535c0ff7ed81142def56126bd0df4690f4e57e8e  OnePlanner/bevfusion_lidar_feature.onnx
df55e869bed3ea3387653a3806cd6a0e56f6b78f5e9e5b39c20cd4d6c0b87432  OnePlanner/ml_package_resworld.param.yaml
50054e7014f9e3a425e51615de788b893771652b4e41df52ac19429c2fb5f4bf  OnePlanner/deployment_manifest.json
d42dbdf0195cddfe5329abcc5c4fe28e7e4f47d108f2975bf54335c8ac2892ee  mlplanner-2stage-a/ml_planner_sampler.onnx
202f013acf8f96191ca8c5cd82a3e7cf6ccb55300105d9234607a3af3822a1aa  mlplanner-2stage-a/metadata.json
8b33591904a1d6a5958ff7cfa1cd434d900af291dda4ee3b58aa3f28b56dbe86  mlplanner-2stage-b/ml_planner_sampler.onnx
b5e95a6bed8353ae8fd6add1bc92c35b23d316b099b05498fba642f517456016  mlplanner-2stage-b/metadata.json
aba79716c35603f96e430da558920661bf759016ea5cff4ec7e55c20fac17a37  mlplanner-1p5stage-b/ml_planner_bev.onnx
4bf246ba529ffd149c43de15ae1bad4692219ead5424c6737535008ccbc258c3  mlplanner-1p5stage-b/metadata.json
d701f6c37b44af6a04deab43535c0ff7ed81142def56126bd0df4690f4e57e8e  mlplanner-1p5stage-b/bevfusion_lidar_feature.onnx
EOF
```

## 3. Switch the code and build (once)

Only two repositories change; every other repository stays as the car has it (pilot-auto.xx1 `beta/v0.53`).

| Workspace path | Repository | Branch |
|---|---|---|
| `src/autoware/universe` | tier4/autoware_universe | `e2e-release/1127/jpntaxi-unified` |
| `src/autoware/launcher` | tier4/autoware_launch.xx1 | `e2e-release/1127/jpntaxi-unified` |

```bash
cd $WS
# note the current commits, to go back after the test
git -C src/autoware/universe rev-parse HEAD > ~/universe-before-1127.txt
git -C src/autoware/launcher rev-parse HEAD > ~/launcher-before-1127.txt

git -C src/autoware/universe fetch origin e2e-release/1127/jpntaxi-unified
git -C src/autoware/launcher fetch origin e2e-release/1127/jpntaxi-unified

# which packages change: expect exactly e2e/autoware_tensorrt_e2e and planning/autoware_ml_planner (+ RELEASE.md)
git -C src/autoware/universe diff --name-only HEAD FETCH_HEAD | cut -d/ -f1-2 | sort -u

git -C src/autoware/universe checkout FETCH_HEAD
git -C src/autoware/launcher checkout FETCH_HEAD
```

If the `diff` line lists anything besides `e2e/autoware_tensorrt_e2e`, `planning/autoware_ml_planner` and the docs,
the car was not on the ResWorld branch `dev/e2e-resworld-full` before: add those packages (the last part of each path
is the package name) to the build below.

Before building, check acados, which the ML-Planner models need for their internal optimizer:

```bash
A=${ACADOS_SOURCE_DIR:-/opt/acados}
ls $A/lib/libacados.so $A/lib/libblasfeo.so $A/lib/libhpipm.so $A/.venv/bin/python3 $A/interfaces/acados_template
```

All five must exist. If one is missing the build still succeeds, but the ML-Planner models then run **without** their
optimizer: fix `ACADOS_SOURCE_DIR` first.

Build only these four packages:

```bash
cd $WS
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select autoware_ml_planner autoware_tensorrt_e2e tier4_planning_launch autoware_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# must print nothing (otherwise acados was not found: see above)
grep -h "acados was not found" log/latest_build/autoware_ml_planner/stdout_stderr.log
```

| Package | Repository | Why |
|---|---|---|
| `autoware_ml_planner` | universe | new: the ML-Planner node (models 2 and 3) |
| `autoware_tensorrt_e2e` | universe | the ResWorld node, plus the 1.5-stage node (model 4) and the 2-stage launch |
| `tier4_planning_launch` | launcher | `resworld.launch.xml`: picks the planner from `e2e_model` |
| `autoware_launch` | launcher | `autoware.launch.xml` gains the `e2e_model` argument; ML-Planner parameter file |

## 4. Start a model

Use the car's usual launch command for the ResWorld test (with `planning_setting:=resworld`) and add `e2e_model`:

```bash
cd $WS && source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml <the car's usual arguments> planning_setting:=resworld e2e_model:=OnePlanner
```

| No. | Add to the launch command |
|---|---|
| 1 | `e2e_model:=OnePlanner` (or leave it out: same thing) |
| 2 | `e2e_model:=mlplanner-2stage-a` |
| 3 | `e2e_model:=mlplanner-2stage-b` |
| 4 | `e2e_model:=mlplanner-1p5stage-b` |

**If the launch command cannot be edited** (started from a fixed script or GUI), change the argument's default instead.
With `--symlink-install` this edits the launch file in `src/`, takes effect at the next start and needs no rebuild:

```bash
M=mlplanner-2stage-a     # OnePlanner | mlplanner-2stage-a | mlplanner-2stage-b | mlplanner-1p5stage-b
sed -i --follow-symlinks -E "s|(<arg name=\"e2e_model\" default=\")[^\"]*\"|\1$M\"|" \
  $WS/install/autoware_launch/share/autoware_launch/launch/autoware.launch.xml
grep 'name="e2e_model" default' $WS/install/autoware_launch/share/autoware_launch/launch/autoware.launch.xml
```

Set it back to `OnePlanner` after the test.

**First start of each model**: the node builds its TensorRT engines (a few minutes, once per model; later starts take
seconds). Do this for all four models once while parked, before the driving starts.

## 5. Checks after every start (before engaging)

```bash
ros2 node list | grep -i bevfusion                       # must print nothing
ros2 node list | grep neural_network_based_planner       # see the table below
ros2 topic echo --once /planning/generator/resworld/candidate_trajectories | grep -A1 generator_name
ros2 topic hz /planning/generator/resworld/candidate_trajectories    # ~10 Hz (Ctrl-C to stop)
ros2 topic hz /planning/trajectory_generator/trajectory              # ~10 Hz
ros2 topic hz /perception/object_recognition/detection/bevfusion/objects   # ~10 Hz
ros2 topic hz /planning/turn_indicators_cmd                          # ~10 Hz
```

| No. | Nodes under `.../neural_network_based_planner/` (besides `trajectory_modifier`, `trajectory_optimizer_node`) | `generator_name` |
|---|---|---|
| 1 | `tensorrt_e2e` | `TensorrtE2eResworld_batch_0` |
| 2, 3 | `ml_planner_node` and `tensorrt_e2e` (the latter only for detections) | `MLPlanner_batch_0` |
| 4 | `ml_planner_node` | `MLPlanner_batch_0` |

ResWorld waits for a short BEV history after a start or a localization jump. If `Waiting for the first input from
'bev_feature'` keeps repeating while the LiDAR runs, the point cloud is not reaching the node: check
`/sensing/lidar/concatenated/pointcloud`. Model 2's chain ran at 5.6 Hz in one replay on a small machine: check its
rate here.

## 6. Switching between models (every time)

1. Stop the car and go to manual driving.
2. Stop Autoware (Ctrl-C in the launch terminal) and wait until it has exited: `ros2 node list` prints nothing.
3. Start it again as in step 4 with the next model's `e2e_model` (or change the default with the `sed` block).
4. Run the checks of step 5, then engage.
5. Write down the model number, the start time and the bag name (table below).

Nothing else changes between models: same build, same files, same post-processing (`trajectory_modifier` →
`trajectory_optimizer` with the unchanged parameter files).

| No. | `e2e_model` | Start time | Bag | Notes |
|---|---|---|---|---|
| | | | | |

## 7. After the test: back to the previous state

```bash
cd $WS
git -C src/autoware/universe checkout $(cat ~/universe-before-1127.txt)
git -C src/autoware/launcher checkout $(cat ~/launcher-before-1127.txt)
colcon build --symlink-install --packages-select autoware_tensorrt_e2e tier4_planning_launch autoware_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The model directories can stay. To put the earlier ResWorld version back:
`mv $DATA/OnePlanner $DATA/OnePlanner-1.4.4.4 && mv $DATA/OnePlanner.before-1127 $DATA/OnePlanner`.

Without switching anything back, `e2e_model:=OnePlanner` (or no `e2e_model`) is exactly the previous ResWorld test
with the 1.4.4.4 model.
