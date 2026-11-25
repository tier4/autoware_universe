# Release 2026-11-27: ML-Planner 2-stage A (jpntaxi)

The same file is on both branches of this model:

| | |
|---|---|
| Python | tier4/OnePlanner `e2e-release/1127/mlplanner-2stage-a` |
| Node | tier4/autoware_universe `e2e-release/1127/mlplanner-2stage-a` = `feat/v0.64/e2e` at ef8d906c16, stock `autoware_ml_planner` |
| WebAuto | project `prd_jt`, ML package `OnePlanner`, package release `20261125_mlplanner__oneplanner-mlplanner-2stage-a-jpntaxi` |
| Release | `oneplanner-mlplanner-2stage-a-jpntaxi` |

The four releases of 2026-11-27 are `resworld-1.4.4.4`, `mlplanner-2stage-a`, `mlplanner-2stage-b` and
`mlplanner-1p5stage-b`. Every one has a branch of that name in both repositories.

## Model

- ML-Planner (tier4/ML-Planner main f618cf7, ported to T4 scenes in `projects/mlplanner`), large (33 M parameters).
- Map inputs are the car node's own map tensors. There is no acceleration input: this is B without `acc5`.
- Turn-indicator head: the graph output `turn_indicator_logits` (DISABLE / LEFT / RIGHT).
- Weights: the uniform average of epochs 16-25 of run
  `mlplanner-2stage-large33m-upmain183-nospeedaug-cos1-notlfuture-nodemap-t4-prdjt-x2dev-bs2048-ep50-20261120`
  (W&B `lbratzsn`).
- Trained on `<T4_ROOT>/lists/20261030/prdjt-x2dev-train-s1-6s.json` (taxi + X2).

## Files

`<RELEASE_ROOT>` is the cluster's shared release store; the hand-over note gives its path.

| File | sha256 |
|---|---|
| `<RELEASE_ROOT>/mlplanner-2stage-a/python/avg16to25-epoch_0025.pth` | `34304788b2809b027fd6a7ff9173a991cc0eb144af585d691c2d2e8d107c9dab` |
| `<RELEASE_ROOT>/mlplanner-2stage-a/node/oneplanner-mlplanner-2stage-a-jpntaxi/ml_planner_sampler.onnx` | `d42dbdf0195cddfe5329abcc5c4fe28e7e4f47d108f2975bf54335c8ac2892ee` |
| `<RELEASE_ROOT>/mlplanner-2stage-a/node/oneplanner-mlplanner-2stage-a-jpntaxi/metadata.json` | `202f013acf8f96191ca8c5cd82a3e7cf6ccb55300105d9234607a3af3822a1aa` |

The `node/` directory holds exactly the WebAuto package release. Golden-val eval inputs (the sidecar and the node's
map tensors) are in `<RELEASE_ROOT>/common/mlplanner-golden-val-inputs/`.

## Python: open-loop evaluation

```bash
cd projects/mlplanner && uv sync
PYTHONPATH=packages/mlplanner/src:<t4-e2e-devkit checkout at 802ae479> .venv/bin/python scripts/eval/open_loop.py \
    <RELEASE_ROOT>/mlplanner-2stage-a/python/avg16to25-epoch_0025.pth --temperature 0 \
    --datalist <T4_ROOT>/lists/20261030/golden-val.json \
    --sidecar <RELEASE_ROOT>/common/mlplanner-golden-val-inputs/sidecar \
    --node-map <RELEASE_ROOT>/common/mlplanner-golden-val-inputs/node-map \
    --out open_loop.json
```

`--temperature 0` is the node's sampler. Reference result: golden-val, one frame per second, 35,303 frames in
769 scenes; six golden-val scenes have no node map and are skipped.

| FDE (m) | 2 s | 4 s | 6 s |
|---|---|---|---|
| all | 0.321 | 1.032 | 2.095 |
| taxi (prd_jt, 444 scenes) | 0.389 | 1.238 | 2.495 |
| X2 (x2_dev, 325 scenes) | 0.227 | 0.743 | 1.536 |

The ONNX graph comes from `scripts/export/export_onnx.py <checkpoint>`, which writes
`<checkpoint dir>/onnx/<stem>/ml_planner.onnx` and `metadata.json`, and checks ONNX against PyTorch. The node
loads the graph as `ml_planner_sampler.onnx`. Copy the checkpoint out of the store before you export it.

## Node

1. Build `e2e-release/1127/mlplanner-2stage-a`.
2. Put the two node files into `<data_path>/ml_planner/`, the stock `model.base_model_directory`.
   The WebAuto package `OnePlanner` unpacks flat into its own model directory (default
   `~/autoware_data/OnePlanner`). Either copy the two files into `ml_planner/`, or set
   `model.base_model_directory` to that directory.
3. Launch `autoware_ml_planner` as usual. The graph's inputs are the stock node's 17 inputs, and the major version
   is 7.

Checked:

- TensorRT 10.8 (the car's version) against PyTorch, fp32: trajectory max error 1.7e-4 m.
- Replay of a recorded Odaiba drive on this node:
  - raw-output FDE at 2 / 4 s: 0.182 / 0.544 m
  - inference: median 5.6 ms
