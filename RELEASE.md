# Release 2026-11-27: ResWorld 1.4.4.4 (jpntaxi)

The same file is on both branches of this model:

| | |
|---|---|
| Python | tier4/OnePlanner `e2e-release/1127/resworld-1.4.4.4` |
| Node | tier4/autoware_universe `e2e-release/1127/resworld-1.4.4.4` = `dev/e2e-resworld-full` at 368a6d97b2, the `autoware_tensorrt_e2e` ResWorld node |
| WebAuto | project `prd_jt`, ML package `OnePlanner`, package release `20261125_resworld__oneplanner-1.4.4.4-jpntaxi` |
| Release | `oneplanner-1.4.4.4-jpntaxi` |

The four releases of 2026-11-27 are `resworld-1.4.4.4`, `mlplanner-2stage-a`, `mlplanner-2stage-b` and
`mlplanner-1p5stage-b`. Every one has a branch of that name in both repositories.

## Model

ResWorld (this repository, `ARCH=resworld`) on the frozen BEVFusion-L backbone (awml-2.8.2-jpntaxi):

- A 6 s displacement head with the measured crab angle. This is the recipe of 1.4.4.2, the winner of the
  11-13 car test.
- Launch labels: every clear launch that is not at a red light is shifted, and the start decision comes after a
  departing lead vehicle.
- Closed-loop DAgger rows: states the planner reached when it drove itself, longitudinal only, 8 % of the batch.

Run: `resworld-6s-disp-lat-crabm-noegohist-launchclear-lead-dagger4nolatcl08-big-d8-ffn1024-tok128-2node-bs1024-lr1.41e-4-ep10-seed32-v12data-20261123`.

- W&B: `b1e95a767001d32b`.
- Checkpoint: step 33,250, whose deployable weights are the EMA.
- Training list: `<T4_ROOT>/lists/20261030/prdjt-x2dev-train-s1-6s.json`.

The turn-indicator head is a separate classifier on the frozen planner's features
(`projects/resworld/scripts/train_indicator_head.py`, 8 GPUs, every training row). The release graph runs it:
output `turn_indicator_logits`. Its golden-val result, on 20,000 rows: accuracy 0.910, macro F1 0.896, NLL 0.381
(recall: none 0.941 / left 0.858 / right 0.871).

## Files

`<RELEASE_ROOT>` is the cluster's shared release store; the hand-over note gives its path.

| File | sha256 |
|---|---|
| `<RELEASE_ROOT>/resworld-1.4.4.4/python/late-0033250.ckpt` | `d6d97002cadcf08bbf88b6f310e498fcc90b5b5a2d3f4c024b11956bab6524eb` |
| `<RELEASE_ROOT>/resworld-1.4.4.4/python/indicator_head.pt` | `30318e04f2ecf832352f66dc1477a70c70ee7d5a6ca552abace5d939d8196611` |
| `<RELEASE_ROOT>/resworld-1.4.4.4/node/oneplanner-1.4.4.4-jpntaxi/` | the four files' sha256 are in its `deployment_manifest.json` |

The `node/` directory holds exactly the WebAuto package release:

- `resworld_planner.simplified.onnx`
- `bevfusion_lidar_feature.onnx`
- `ml_package_resworld.param.yaml`
- `deployment_manifest.json`

BEVFusion-L checkpoint and config: `<RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi/`.

## Python: open-loop evaluation

The devkit is installed by `uv sync` at the pinned commit (`devkit.pin`, 802ae479). `ARCH=resworld` selects
the project.

```bash
uv sync
ARCH=resworld uv run python scripts/export_e2e_predictions.py \
    --checkpoint <RELEASE_ROOT>/resworld-1.4.4.4/python/late-0033250.ckpt \
    --data-list <T4_ROOT>/lists/20261030/golden-val.json \
    --bevfusion-config <RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi/bevfusion_lidar_voxel_second_secfpn_30e_8xb24_jpntaxi_base_120m.py \
    --bevfusion-ckpt <RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi/best_epoch_28.pth \
    --batch-size 8 --output predictions.jsonl
PYTHONPATH=<t4-e2e-devkit checkout at c234eb17> .venv/bin/python \
    -c 'import sys; from t4_e2e_devkit.cli import main; sys.exit(main())' score-manifest \
    <T4_ROOT>/lists/20261030/golden-val.json --predictions predictions.jsonl --output-dir score --backend gpu
```

Score with the devkit at c234eb17 (the pin of OnePlanner `main`) or later. At this branch's own pin, 802ae479,
`score-manifest` keeps scenes from every batch in memory (fixed upstream in tier4/e2e-devkit#134) and runs out of memory after
about 18,000 of the 348,124 frames. At c234eb17 it holds about 7 GB and scores 2.5-4 frames per second on one GPU: about 1.5 days for every
frame, or about 2 hours in 16 shards (see below).

Reference result: golden-val, every frame (10 Hz), 348,124 frames in 775 scenes. It was run with batch 8. Another
batch size changes the BEV backbone's float rounding, so per-frame values are then not bit-exact.

| FDE (m) | 2 s | 4 s | 6 s |
|---|---|---|---|
| all | 0.256 | 0.909 | 1.890 |
| taxi (prd_jt, 450 scenes) | 0.295 | 1.040 | 2.153 |
| X2 (x2_dev, 325 scenes) | 0.200 | 0.720 | 1.511 |

These are the scorer's `fde_2s` / `fde_4s` / `fde_6s`, split by fleet by scene name. The ML-Planner releases'
reference takes one frame per second. On the 34,810 of its 35,303 frames that this evaluation also covers, this
model gives 0.255 / 0.903 / 1.883 (taxi 0.294 / 1.034 / 2.145, X2 0.199 / 0.717 / 1.510).

The scorer's full report on the same frames (`score/aggregate.json`, devkit c234eb17, version v2):

| EPDMS | NC | DAC | DDC | TLC | TTC | EP | LK | comfort | EC | ADE (m) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.962 | 0.998 | 0.999 | 0.990 | 1.000 | 0.995 | 0.981 | 0.995 | 1.000 | 0.860 | 0.689 |

Each value is the mean over the frames. To score in parallel, run `--shard-index i --num-shards n` and take each
column's mean over the shards' `per_window.csv` rows. That is the devkit's own aggregation (`aggregate_navsim_results`),
and here it reproduced the FDE above to the sixth decimal.

To rebuild the car package, run `projects/resworld/scripts/release_resworld.py --checkpoint <ckpt> --version
<x.y.z.w> --indicator-head <indicator_head.pt> --backbone-dir
<RELEASE_ROOT>/common/bevfusion-l-awml-2.8.2-jpntaxi --output-dir <dir>`. It exports both graphs, checks them
against PyTorch and writes the manifest.

## Node

1. Build `e2e-release/1127/resworld-1.4.4.4`.
2. Put the four node files into the model directory. The default is `~/autoware_data/OnePlanner`, which is where
   the WebAuto package `OnePlanner` unpacks.
3. Launch `ros2 launch autoware_tensorrt_e2e e2e_planner_resworld.launch.xml`.

The graph's inputs and outputs are identical to 1.4.4.2's, so the node and launch of the 11-13 car test run
it unchanged.

Checked:

- TensorRT 10.8 (the car's version) against ONNX Runtime, on 80 recorded start rows: trajectory max error 5.5e-5 m,
  turn-indicator logits 1.0e-5, the indicator's argmax agrees on 80 / 80. ONNX Runtime against PyTorch at export:
  trajectory 7.6e-6 m, logits 3.9e-6.
- Closed-loop replay battery, against the run 1.4.4.2 came from, at the same step: 13 / 13 criteria pass.
  - Never launched at a start: 16 / 1068 runs (that run: 51).
  - False starts while holding: 31 / 471 (that run: 32).
  - Braking deficit: 1.75 m (that run: 1.89 m).
  - Turn oscillation: 1.00x of that run's.
