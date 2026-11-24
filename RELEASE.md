# Release 2026-11-27: ML-Planner 1.5-stage B (jpntaxi)

The same file is on both branches of this model:

| | |
|---|---|
| Python | tier4/OnePlanner `e2e-release/1127/mlplanner-1p5stage-b` |
| Node | tier4/autoware_universe `e2e-release/1127/mlplanner-1p5stage-b`: `feat/e2e-mlplanner-bev-node-v064`, i.e. `feat/v0.64/e2e` at ef8d906c16 plus the `ml_planner_bev_node` in `e2e/autoware_tensorrt_e2e` |
| WebAuto | project `prd_jt`, ML package `OnePlanner`, package release `20261125_mlplanner__oneplanner-1p5stage-b-test1-jpntaxi` (WebAuto names are at most 64 characters) |
| Release | `oneplanner-mlplanner-1p5stage-b-test1-jpntaxi` |

The four releases of 2026-11-27 are `resworld-1442`, `mlplanner-2stage-a`, `mlplanner-2stage-b` and
`mlplanner-1p5stage-b`. Every one has a branch of that name in both repositories.

## Model

The model is ML-Planner 2-stage B (car node map inputs, acceleration input `acc5`, turn-indicator head) plus one
extra input, `bev_feature`:

- The source is the frozen BEVFusion-L backbone (awml-2.8.2-jpntaxi): 512 x 180 x 180 at 0.17 m.
- That is the same extractor graph the ResWorld car package ships.
- The planner pools it to a token grid.

Weights are the uniform average of epochs 16-25 of run
`mlplanner-1p5stage-large33m-bev128-upmain183-nospeedaug-cos1-notlfuture-nodemap-acc5-t4-prdjt-x2dev-bs2048-ep50-20261120`
(W&B `kmg5ygq3`).

## Files

`<RELEASE_ROOT>` is the cluster's shared release store; the hand-over note gives its path.

| File | sha256 |
|---|---|
| `<RELEASE_ROOT>/mlplanner-1p5stage-b/python/avg16to25-epoch_0025.pth` | `2045dcdd8de28b9814949722e34e60bbb5aef8495eea98f66e568d9ce9c27bc2` |
| `<RELEASE_ROOT>/mlplanner-1p5stage-b/node/oneplanner-mlplanner-1p5stage-b-test1-jpntaxi/ml_planner_bev.onnx` | `aba79716c35603f96e430da558920661bf759016ea5cff4ec7e55c20fac17a37` |
| `<RELEASE_ROOT>/mlplanner-1p5stage-b/node/oneplanner-mlplanner-1p5stage-b-test1-jpntaxi/metadata.json` | `4bf246ba529ffd149c43de15ae1bad4692219ead5424c6737535008ccbc258c3` |
| `<RELEASE_ROOT>/mlplanner-1p5stage-b/node/oneplanner-mlplanner-1p5stage-b-test1-jpntaxi/bevfusion_lidar_feature.onnx` | `d701f6c37b44af6a04deab43535c0ff7ed81142def56126bd0df4690f4e57e8e` |

The `node/` directory holds exactly the WebAuto package release.

Other inputs:

- BEVFusion-L checkpoint and config: `<RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi/`.
- Golden-val eval inputs: `<RELEASE_ROOT>/common/mlplanner-golden-val-inputs/`.

## Python: open-loop evaluation

The BEV backbone comes from this repository's `src` (the `oneplanner` package):

```bash
cd projects/mlplanner && uv sync
PYTHONPATH=packages/mlplanner/src:../../src:<t4-e2e-devkit checkout at 802ae479> .venv/bin/python scripts/eval/open_loop.py \
    <RELEASE_ROOT>/mlplanner-1p5stage-b/python/avg16to25-epoch_0025.pth --temperature 0 \
    --datalist <T4_ROOT>/lists/20261030/golden-val.json \
    --sidecar <RELEASE_ROOT>/common/mlplanner-golden-val-inputs/sidecar \
    --node-map <RELEASE_ROOT>/common/mlplanner-golden-val-inputs/node-map \
    --bev-config <RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi/bevfusion_lidar_voxel_second_secfpn_30e_8xb24_jpntaxi_base_120m.py \
    --bev-checkpoint <RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi/best_epoch_28.pth \
    --batch 16 --out open_loop.json
```

Reference result: golden-val, one frame per second, 35,303 frames in 769 scenes.

| FDE (m) | 2 s | 4 s | 6 s |
|---|---|---|---|
| all | 0.227 | 0.822 | 1.806 |
| taxi (prd_jt, 444 scenes) | 0.274 | 0.994 | 2.170 |
| X2 (x2_dev, 325 scenes) | 0.162 | 0.584 | 1.299 |

To rebuild the graph, run `scripts/export/export_onnx.py <checkpoint>`. It writes `ml_planner_bev.onnx` with a
static batch of 1 and the input `bev_feature [1, 512, 180, 180]`. Its `metadata.json` carries
`bev_feature_shape`.

## Node

1. Build `e2e-release/1127/mlplanner-1p5stage-b` (package `autoware_tensorrt_e2e`).
2. Put the three node files into one directory. The WebAuto package `OnePlanner` unpacks them flat into
   `~/autoware_data/OnePlanner` by default.
3. Launch `ros2 launch autoware_tensorrt_e2e ml_planner_bev.launch.xml data_path:=<that directory>`.

The launch file is the stock ML-Planner launch with two additions:

- the input `~/input/pointcloud` (the concatenated LiDAR cloud);
- `config/ml_planner_bev.param.yaml`, which sets the extractor and the one-frame BEV cache.

Everything after inference is the stock ML-Planner code: post-processing, the optimizer and the turn indicator.

Checked:

- TensorRT 10.8 (the car's version) against PyTorch, fp32: trajectory max error 3.7e-4 m, turn-indicator logits 2.1e-3.
- Replay of a recorded Odaiba drive on this node:
  - raw-output FDE at 2 / 4 s: 0.159 / 0.473 m, matching 2-stage B on its stock node (0.159 / 0.452)
  - rate: 8.9 Hz
  - inference: median 10.2 ms
