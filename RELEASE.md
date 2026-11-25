# Release 2026-11-27: ML-Planner 2-stage B (jpntaxi)

The same file is on both branches of this model:

| | |
|---|---|
| Python | tier4/OnePlanner `e2e-release/1127/mlplanner-2stage-b` |
| Node | tier4/autoware_universe `e2e-release/1127/mlplanner-2stage-b` = `feat/v0.64/e2e` at ef8d906c16, stock `autoware_ml_planner` |
| WebAuto | project `prd_jt`, ML package `OnePlanner`, package release `20261125_mlplanner__oneplanner-mlplanner-2stage-b-jpntaxi` |
| Release | `oneplanner-mlplanner-2stage-b-jpntaxi` |

The four releases of 2026-11-27 are `resworld-1.4.4.4`, `mlplanner-2stage-a`, `mlplanner-2stage-b` and
`mlplanner-1p5stage-b`. Every one has a branch of that name in both repositories.

## Model

- ML-Planner (tier4/ML-Planner main f618cf7, ported to T4 scenes in `projects/mlplanner`), large (33 M parameters).
- Map inputs are the car node's own map tensors. The ego history carries the current acceleration (`acc5`).
- Turn-indicator head: the graph output `turn_indicator_logits` (DISABLE / LEFT / RIGHT).
- Weights: the uniform average of epochs 16-25 of run
  `mlplanner-2stage-large33m-upmain183-nospeedaug-cos1-notlfuture-nodemap-acc5-t4-prdjt-x2dev-bs2048-ep50-20261120`
  (W&B `94ktj8qr`).
- Trained on `<T4_ROOT>/lists/20261030/prdjt-x2dev-train-s1-6s.json` (taxi + X2).

## Files

`<RELEASE_ROOT>` is the cluster's shared release store; the hand-over note gives its path.

| File | sha256 |
|---|---|
| `<RELEASE_ROOT>/mlplanner-2stage-b/python/avg16to25-epoch_0025.pth` | `9ac79c96de241f1a8cd959efdf2884653884bafb98aa383ed832325a9d6080fa` |
| `<RELEASE_ROOT>/mlplanner-2stage-b/node/oneplanner-mlplanner-2stage-b-jpntaxi/ml_planner_sampler.onnx` | `8b33591904a1d6a5958ff7cfa1cd434d900af291dda4ee3b58aa3f28b56dbe86` |
| `<RELEASE_ROOT>/mlplanner-2stage-b/node/oneplanner-mlplanner-2stage-b-jpntaxi/metadata.json` | `b5e95a6bed8353ae8fd6add1bc92c35b23d316b099b05498fba642f517456016` |

The `node/` directory holds exactly the WebAuto package release. Golden-val eval inputs (the sidecar and the node's
map tensors) are in `<RELEASE_ROOT>/common/mlplanner-golden-val-inputs/`.

## Python: open-loop evaluation

```bash
cd projects/mlplanner && uv sync
PYTHONPATH=packages/mlplanner/src:<t4-e2e-devkit checkout at 802ae479> .venv/bin/python scripts/eval/open_loop.py \
    <RELEASE_ROOT>/mlplanner-2stage-b/python/avg16to25-epoch_0025.pth --temperature 0 \
    --datalist <T4_ROOT>/lists/20261030/golden-val.json \
    --sidecar <RELEASE_ROOT>/common/mlplanner-golden-val-inputs/sidecar \
    --node-map <RELEASE_ROOT>/common/mlplanner-golden-val-inputs/node-map \
    --out open_loop.json
```

`--temperature 0` is the node's sampler. Reference result: golden-val, one frame per second, 35,303 frames in
769 scenes; six golden-val scenes have no node map and are skipped.

| FDE (m) | 2 s | 4 s | 6 s |
|---|---|---|---|
| all | 0.230 | 0.822 | 1.788 |
| taxi (prd_jt, 444 scenes) | 0.279 | 0.994 | 2.147 |
| X2 (x2_dev, 325 scenes) | 0.163 | 0.582 | 1.289 |

The ONNX graph comes from `scripts/export/export_onnx.py <checkpoint>`, which writes
`<checkpoint dir>/onnx/<stem>/ml_planner.onnx` and `metadata.json`, and checks ONNX against PyTorch. The node
loads the graph as `ml_planner_sampler.onnx`. Copy the checkpoint out of the store before you export it.

## Node

1. Build `e2e-release/1127/mlplanner-2stage-b`.
2. Put the two node files into `<data_path>/ml_planner/`, the stock `model.base_model_directory`.
   The WebAuto package `OnePlanner` unpacks flat into its own model directory (default
   `~/autoware_data/OnePlanner`). Either copy the two files into `ml_planner/`, or set
   `model.base_model_directory` to that directory.
3. Launch `autoware_ml_planner` as usual. The graph's inputs are the stock node's 17 inputs, and the major version
   is 7.

Checked:

- TensorRT 10.8 (the car's version) against PyTorch, fp32: trajectory max error 4.5e-4 m.
- Replay of a recorded Odaiba drive on this node:
  - raw-output FDE at 2 / 4 s: 0.159 / 0.452 m
  - inference: median 4.7 ms
