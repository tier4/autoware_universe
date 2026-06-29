# autoware_camera_streampetr

## Purpose

The `autoware_camera_streampetr` package is used for 3D object detection based on images only.

## Inner-workings / Algorithms

This package implements a TensorRT powered inference node for StreamPETR [1]. This is the first camera-only 3D object detection node in autoware.

This node has been optimized for multi-camera systems where the camera topics are published in a sequential manner, not all at once. The node takes
advantage of this by preprocessing (resize, crop, normalize) the images and storing them appropriately on GPU, so that delay due to preprocessing can be minimized.

```pgsql

Topic for image_i arrived                                     -------------------------
  |                                                                                   |
  |                                                                                   |
  |                                                                                   |
  v                                                                                   |
Is image distorted?                                                                   |
  |              \                                                                    |
  |               \                                                                   |
Yes               No                                                                  |Image Updates
  |                |                                                                  |done in parallel, if multitheading is on
  v                |                                                                  |otherwise done sequentially in FIFO order
Undistort          |                                                                  |
  |                |                                                                  |
  v                v                                                                  |
Load image into GPU memory                                                            |
  |                                                                                   |
  v                                                                                   |
Preprocess image (scale & crop ROI & normalize)                                       |
  |                                                                                   |
  v                                                                                   |
Store in GPU memory binding location for model input                                  |
  |                                                          -------------------------|
  v                                                                                   |
Is image the `anchor_image`?                                                          |
  |                \                                                                  |
  |                 \                                                                 |
No                  Yes                                                               |
  |                  |                                                                |
  v                  v                                                                | If multithreading is on
(Wait)     Are all images synced within `max_time_difference`?                        | image Updates are temporarily frozen
                      |                           \                                   | until this part completes.
                      |                            \                                  |
                    Yes                             No                                |
                      |                             |                                 |
                      v                             v                                 |
         Perform model forward pass            (Sync failed! Skip prediction)         |
                      |                                                               |
                      v                                                               |
         Postprocess (NMS + ROS2 format)                                              |
                      |                                                               |
                      v                                                               |
             Publish predictions                             -------------------------|

```

## Inputs / Outputs

### Input

| Name                          | Type                                                             | Description                                                     |
| ----------------------------- | ---------------------------------------------------------------- | --------------------------------------------------------------- |
| `~/input/camera*/image`       | `sensor_msgs::msg::Image` or `sensor_msgs::msg::CompressedImage` | Input image topics (supports both compressed and uncompressed). |
| `~/input/camera*/camera_info` | `sensor_msgs::msg::CameraInfo`                                   | Input camera info topics, for camera parameters.                |

### Output

| Name                            | Type                                                | Description                                                               | RTX 3090 Latency (ms) |
| ------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------- | --------------------- |
| `~/output/objects`              | `autoware_perception_msgs::msg::DetectedObjects`    | Detected objects.                                                         | —                     |
| `latency/preprocess`            | `autoware_internal_debug_msgs::msg::Float64Stamped` | Preprocessing time per image(ms).                                         | 3.25                  |
| `latency/total`                 | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total processing time (ms): preprocessing + inference + postprocessing.   | 26.04                 |
| `latency/inference`             | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total inference time (ms).                                                | 22.13                 |
| `latency/inference/backbone`    | `autoware_internal_debug_msgs::msg::Float64Stamped` | Backbone inference time (ms).                                             | 16.21                 |
| `latency/inference/ptshead`     | `autoware_internal_debug_msgs::msg::Float64Stamped` | Points head inference time (ms).                                          | 5.45                  |
| `latency/inference/pos_embed`   | `autoware_internal_debug_msgs::msg::Float64Stamped` | Position embedding inference time (ms).                                   | 0.40                  |
| `latency/inference/postprocess` | `autoware_internal_debug_msgs::msg::Float64Stamped` | nms + filtering + converting network predictions to autoware format (ms). | 0.40                  |
| `latency/cycle_time_ms`         | `autoware_internal_debug_msgs::msg::Float64Stamped` | Time between two consecutive predictions (ms).                            | 110.65                |

## Parameters

### StreamPETR node

The `autoware_camera_streampetr` node has various parameters for configuration:

#### Model Parameters

- `model_params.backbone_path`: Path to the backbone ONNX model
- `model_params.head_path`: Path to the head ONNX model
- `model_params.position_embedding_path`: Path to the position embedding ONNX model
- `model_params.fp16_mode`: Enable FP16 inference mode
- `model_params.use_temporal`: Enable temporal modeling
- `model_params.input_image_height`: Input image height for preprocessing
- `model_params.input_image_width`: Input image width for preprocessing
- `model_params.class_names`: List of detection class names
- `model_params.num_proposals`: Number of object proposals
- `model_params.detection_range`: Detection range for filtering objects

#### Post-processing Parameters

- `post_process_params.iou_nms_search_distance_2d`: 2D search distance for IoU NMS
- `post_process_params.circle_nms_dist_threshold`: Distance threshold for circle NMS
- `post_process_params.iou_nms_threshold`: IoU threshold for NMS
- `post_process_params.confidence_threshold`: Confidence threshold for detections
- `post_process_params.yaw_norm_thresholds`: Yaw normalization thresholds

#### Ego vehicle mask

Masking the area of the ego vehicle in order to reduce FP caused by reflection. Configure via **launch** or `camera_streampetr.param.yaml`, not `tensorrt_stream_petr.param.yaml` (model/post-process only).

- `ego_mask.enabled`: Enable masking (default: `false`)
- `ego_mask.fill_value_bgr`: BGR fill inside polygons, 0–255 (default: `[0, 0, 0]`)
- `ego_mask.roi_polygons_yaml`: One YAML path per model ROI index; empty string disables that ROI.

Mask polygons are rasterized at the subscribed image resolution before StreamPETR preprocessing
resizes, crops, and normalizes the image. If `is_distorted_image` is `true`, the mask is applied
after CUDA undistortion/remap at the full image resolution. If `normalized` is `true`, polygon
coordinates are scaled by the current full image width and height.

Example polygon files: `config/camera9_polygons.yaml`, `config/camera10_polygons.yaml`.

**X2 five-camera layout** (`tensorrt_stream_petr.x2.launch.xml`): ROI 2 → camera10 (left strip), ROI 4 → camera9 (right strip). Ego mask params are set in that launch file.

The standalone live designer subscribes to available raw and compressed camera image topics, pairs
them with `CameraInfo`, shows undistorted full-resolution frames in a browser dropdown, and exports
both `camera_N_mask` parameter snippets and `polygons` YAML:

```bash
python3 tools/camera_mask_designer.py \
  --param-path config/camera_streampetr.param.yaml \
  --output-dir /tmp/streampetr_mask_editor/evidence \
  --cache-dir /tmp/streampetr_mask_editor/frame_cache \
  --base-frame base_link
```

Open the URL printed by the tool, select a stream, freeze a representative frame, draw the mask,
then copy the generated output. The default port is `8766`; if it is already in use, the tool tries
the next ports automatically. If a matching `CameraInfo` topic is not available, the preview falls
back to the original image and marks the stream as `original/fallback`.

The UI can load the current `camera_N_mask` values from the parameter YAML, display the selected
camera's existing mask over the undistorted preview, save the updated `camera_N_mask` block back to
the YAML, and write PNG evidence images to the output folder. Evidence export can write multiple
patterns from the same frame: semi-transparent overlay, filled-mask preview, and outline-only. The
parameter path also accepts a `file://` URL. On startup, the UI tries to load the default parameter
path automatically; if the file or the selected camera's mask is unavailable, that camera is treated
as having no mask.

`camera_N_mask.mask` accepts one polygon with at least three `(x, y)` points; four or more points
are valid. Disabled masks are treated as empty even if their YAML `mask` field contains placeholder
coordinates.

Use `Save Current Frame` to cache the selected undistorted frame and stream metadata. In live mode
the tool also listens to `/tf` and `/tf_static`; once `base_link -> CameraInfo.header.frame_id` is
available, the cache JSON includes `camera_info_p`, `base_to_camera`, full-resolution `lidar2img`,
model-space `lidar2img_model`, and `img2lidar`. A later session can avoid ROS 2 subscriptions
entirely and use only cached frames. By default, if `--cache-dir` already contains valid cached
frames, the UI automatically starts in offline cache mode. Use `--live` to force fresh ROS 2 image
and TF subscriptions, or `--no-auto-offline-cache` to disable the automatic fallback:

```bash
python3 tools/camera_mask_designer.py \
  --offline-cache-dir /tmp/streampetr_mask_editor/frame_cache \
  --param-path config/camera_streampetr.param.yaml
```

An experimental ONNX overlay helper can run the three StreamPETR ONNX files from cached frames and
write per-camera PNGs with projected 3D boxes:

```bash
python3 tools/streampetr_onnx_overlay.py \
  --cache-dir /tmp/streampetr_mask_editor/frame_cache \
  --model-dir /opt/autoware/mlmodels/streampetr \
  --output-dir /tmp/streampetr_mask_editor/onnx_overlay \
  --projection-json /path/to/lidar2img_by_camera.json
```

The helper requires `onnxruntime`. The browser UI checks the frame cache before `Run ONNX Overlay`;
if the requested camera ids are missing or lack projection metadata, live mode automatically saves a
fresh batch frame cache first. It also passes the currently loaded/designed `camera_N_mask` values
to the ONNX helper, which applies them to the full-resolution undistorted cached image before
StreamPETR resize/crop/normalize. Older cached metadata or manual workflows can still use
`--projection-json` to provide `lidar2img` or `img2lidar` matrices. Without those projection
matrices, it can still run inference but cannot place boxes on the image correctly.

The ONNX overlay is a debugging aid, not a bit-exact replacement for the ROS 2 node output. It
reimplements only the basic score, yaw-norm, range filtering, score sort, and max-box limit in
Python. The production CUDA postprocess also applies circle NMS and may apply BEV IoU NMS depending
on `post_process_params.iou_nms_threshold`; the helper also runs with zeroed temporal memory and
identity ego pose. Therefore overlay boxes can differ from `~/output/objects`.

For offline workflows, save a one-frame undistorted snapshot and open it in the static editor:

```bash
python3 tools/save_undistorted_snapshot.py \
  --image-topic /sensing/camera/camera0/image_raw \
  --camera-info-topic /sensing/camera/camera0/camera_info \
  --output /tmp/streampetr_mask_editor/snapshots/streampetr_camera0_undistorted.png
```

For compressed input, add `--compressed` and pass the compressed image topic. Then open
`tools/camera_mask_editor.html` locally. Both helpers are separate from the StreamPETR node and only
use the subscribed image plus `CameraInfo` to reproduce the same full-resolution OpenCV
undistortion map.

#### Node Parameters

- `max_camera_time_diff`: Maximum allowed time difference between cameras (seconds)
- `rois_number`: Number of camera ROIs/cameras (default: 6)
- `is_compressed_image`: Whether input images are compressed
- `is_distorted_image`: Whether input images are distorted
- `multithreading`: Whether to use multithreading for handling image callbacks
- `anchor_camera_id`: ID of the anchor camera for synchronization (default: 0)
- `debug_mode`: Enable debug mode for timing measurements
- `build_only`: Build TensorRT engines and exit without running inference

### The `build_only` option

The `autoware_camera_streampetr` node has a `build_only` option to build the TensorRT engine files from the specified ONNX files, after which the program exits.

```bash
ros2 launch autoware_camera_streampetr tensorrt_stream_petr.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `autoware_camera_streampetr` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch autoware_camera_streampetr tensorrt_stream_petr.launch.xml log_level:=debug
```

## Assumptions / Known limits

This node is camera-only and does not require pointcloud input. It assumes:

- All cameras are synchronized within the specified `max_camera_time_diff`
- Camera calibration information is available and accurate
- The anchor camera (specified by `anchor_camera_id`) triggers the inference cycle
- Transform information between camera frames and base_link is available via tf
- Transform information between map and base_link is available via tf for ego motion compensation
- **The input images are undistorted**

## Trained Models

You can download the ONNX model files for StreamPETR. The files should be placed in the appropriate model directory as specified in the launch configuration.

Required model files:

- [Backbone ONNX Model](https://awf.ml.dev.web.auto/perception/models/streampetr/v1/simplify_extract_img_feat.onnx)
- [Head ONNX Model](https://awf.ml.dev.web.auto/perception/models/streampetr/v1/simplify_pts_head_memory.onnx)
- [Position Embedding ONNX Model](https://awf.ml.dev.web.auto/perception/models/streampetr/v1/simplify_position_embedding.onnx)

If you want to train and deploy your own model, you can find the source code for that in [AWML](https://github.com/tier4/AWML/tree/main/projects/StreamPETR).

## Changelog

## References/External links

[1] Wang, Shihao and Liu, Yingfei and Wang, Tiancai and Li, Ying and Zhang, Xiangyu. "Exploring Object-Centric Temporal Modeling for Efficient Multi-View 3D Object Detection." 2023 <!-- cspell:disable-line -->

## (Optional) Future extensions / Unimplemented parts

- Enable 2d object detection. Because 2d object detection is used as an auxiliary loss during training, the same node can easily support 2d object detection with minor updates.
- Implement int8 quantization for the backbone to further reduce inference latency
- Execute the image backbone for each image as they arrive, to further reduce latency.
- Add velocity to predictions.
