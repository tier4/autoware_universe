#!/usr/bin/env python3

# Copyright 2026 TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Experimental StreamPETR ONNX inference and 2D bbox overlay from cached frames.

This script is deliberately separate from the live mask designer. It consumes the
frame cache produced by camera_mask_designer.py, runs the three StreamPETR ONNX
models with onnxruntime, performs a CPU version of the simple score/yaw/range
postprocess, and saves per-camera PNGs with projected 3D boxes.

Projection requires camera extrinsics. Cached metadata can provide either:
  - lidar2img: a 4x4 row-major matrix, or
  - img2lidar: a 4x4 row-major matrix, which will be inverted.

If projection matrices are absent, inference can still run, but 2D bbox overlay
is skipped unless --allow-identity-projection is set for rough debugging.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


DEFAULT_CLASS_NAMES = ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN", "TRAFFIC_CONE", "BARRIER"]
DEFAULT_SCORE_THRESHOLDS = [0.36, 0.39, 0.38, 0.41, 0.43, 0.43, 0.43]
DEFAULT_YAW_NORM_THRESHOLDS = [0.3, 0.3, 0.3, 0.3, 0.0, 0.0, 0.0]
DEFAULT_DETECTION_RANGE = [-61.2, -61.2, -10.0, 61.2, 61.2, 10.0]
WORK_DIR = Path("/tmp/streampetr_mask_editor")


@dataclass
class CachedFrame:
    key: str
    image_topic: str
    image_path: Path
    camera_id: int
    image_bgr: np.ndarray
    metadata: dict[str, object]


@dataclass
class Box3D:
    x: float
    y: float
    z: float
    width: float
    length: float
    height: float
    yaw: float
    score: float
    label: int


def main() -> None:
    args = parse_args()
    ort = import_onnxruntime()
    camera_ids = parse_camera_ids(args.camera_ids)
    frames = load_cached_frames(Path(args.cache_dir), args.rois_number, camera_ids)
    if len(frames) < args.rois_number:
        raise RuntimeError(
            f"need at least {args.rois_number} cached frames, found {len(frames)} in {args.cache_dir}"
        )
    if args.projection_json:
        apply_projection_overrides(frames, Path(args.projection_json))

    model_dir = Path(args.model_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print("loading ONNX sessions")
    backbone = ort.InferenceSession(
        str(model_dir / "simplify_extract_img_feat.onnx"), providers=["CPUExecutionProvider"]
    )
    pos_embed = ort.InferenceSession(
        str(model_dir / "simplify_position_embedding.onnx"), providers=["CPUExecutionProvider"]
    )
    head = ort.InferenceSession(
        str(model_dir / "simplify_pts_head_memory.onnx"), providers=["CPUExecutionProvider"]
    )

    imgs = np.stack(
        [preprocess_image(frame.image_bgr, args.input_height, args.input_width) for frame in frames],
        axis=0,
    )[None, ...].astype(np.float32)
    intrinsics = build_intrinsics(frames, args.input_height, args.input_width).astype(np.float32)
    img2lidar = build_img2lidar(frames, args.allow_identity_projection).astype(np.float32)
    img_metas_pad = np.array([args.input_height, args.input_width, 3], dtype=np.float32)

    print("running position embedding")
    pos_outputs = pos_embed.run(
        None,
        {
            "img_metas_pad": img_metas_pad,
            "intrinsics": intrinsics[None, ...],
            "img2lidar": img2lidar[None, ...],
        },
    )
    pos_embed_out, cone = pos_outputs[:2]

    print("running backbone")
    img_feats = backbone.run(None, {"img": imgs})[0]

    print("running head")
    identity = np.eye(4, dtype=np.float32)[None, ...]
    head_inputs = {
        "x": img_feats,
        "pos_embed": pos_embed_out,
        "cone": cone,
        "data_ego_pose": identity,
        "data_ego_pose_inv": identity,
        "pre_memory_embedding": np.zeros((1, 1024, 256), dtype=np.float32),
        "pre_memory_reference_point": np.zeros((1, 1024, 3), dtype=np.float32),
        "pre_memory_timestamp": np.zeros((1, 1024, 1), dtype=np.float32),
        "pre_memory_egopose": np.zeros((1, 1024, 4, 4), dtype=np.float32),
        "pre_memory_velo": np.zeros((1, 1024, 2), dtype=np.float32),
    }
    head_outputs = dict(zip([out.name for out in head.get_outputs()], head.run(None, head_inputs)))
    boxes = postprocess_boxes(
        head_outputs["all_cls_scores"],
        head_outputs["all_bbox_preds"],
        score_thresholds=args.score_thresholds,
        yaw_norm_thresholds=args.yaw_norm_thresholds,
        detection_range=args.detection_range,
        max_boxes=args.max_boxes,
    )
    print(f"postprocessed boxes: {len(boxes)}")

    save_boxes_json(output_dir / "streampetr_onnx_boxes.json", boxes)
    for index, frame in enumerate(frames):
        lidar2img = projection_for_frame(frame, allow_identity=args.allow_identity_projection)
        if lidar2img is None:
            print(f"skip overlay for {frame.image_topic}: missing lidar2img/img2lidar metadata")
            continue
        overlay = draw_projected_boxes(
            frame.image_bgr.copy(),
            boxes,
            lidar2img,
            class_names=args.class_names,
            max_depth=args.max_projection_depth,
        )
        out_path = output_dir / f"camera_{frame.camera_id}_streampetr_onnx_bbox_overlay.png"
        cv2.imwrite(str(out_path), overlay)
        print(f"saved {out_path}")


def import_onnxruntime():
    try:
        import onnxruntime as ort  # type: ignore

        return ort
    except Exception as error:  # noqa: BLE001
        raise RuntimeError(
            "onnxruntime is required. Run this with a Python environment that has onnxruntime, "
            "for example /home/yoshiri/Documents/yolox_visualize/.venv/bin/python on this host."
        ) from error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", required=True, help="Frame cache folder from mask designer")
    parser.add_argument(
        "--model-dir",
        default="/home/yoshiri/autoware_data/camera_streampetr",
        help="Folder containing the three StreamPETR ONNX files",
    )
    parser.add_argument("--output-dir", default=str(WORK_DIR / "onnx_overlay"))
    parser.add_argument("--rois-number", type=int, default=5)
    parser.add_argument(
        "--camera-ids",
        default="8,6,10,7,9",
        help="Comma-separated camera ids to load from the frame cache, in ONNX input order",
    )
    parser.add_argument("--input-height", type=int, default=480)
    parser.add_argument("--input-width", type=int, default=640)
    parser.add_argument("--max-boxes", type=int, default=100)
    parser.add_argument("--max-projection-depth", type=float, default=120.0)
    parser.add_argument(
        "--projection-json",
        default="",
        help="Optional JSON mapping camera id or image topic to lidar2img/img2lidar matrices",
    )
    parser.add_argument(
        "--allow-identity-projection",
        action="store_true",
        help="Use identity projection when cached metadata lacks lidar2img/img2lidar",
    )
    parser.add_argument("--class-names", nargs="*", default=DEFAULT_CLASS_NAMES)
    parser.add_argument("--score-thresholds", nargs="*", type=float, default=DEFAULT_SCORE_THRESHOLDS)
    parser.add_argument(
        "--yaw-norm-thresholds", nargs="*", type=float, default=DEFAULT_YAW_NORM_THRESHOLDS
    )
    parser.add_argument(
        "--detection-range", nargs=6, type=float, default=DEFAULT_DETECTION_RANGE
    )
    return parser.parse_args()


def parse_camera_ids(text: str) -> list[int]:
    if not text.strip():
        return []
    camera_ids = []
    for item in re.split(r"[,\s]+", text.strip()):
        if not item:
            continue
        camera_id = int(item)
        if camera_id < 0:
            raise RuntimeError("camera ids must be non-negative")
        camera_ids.append(camera_id)
    return camera_ids


def load_cached_frames(
    cache_dir: Path, rois_number: int, camera_ids: Optional[list[int]] = None
) -> list[CachedFrame]:
    frames = []
    for metadata_path in sorted(cache_dir.expanduser().resolve().glob("*.json")):
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        image_path = metadata_path.parent / str(metadata.get("image_file", ""))
        if not image_path.exists():
            continue
        image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            continue
        topic = str(metadata.get("image_topic", metadata_path.stem))
        frames.append(
            CachedFrame(
                key=str(metadata.get("key", metadata_path.stem)),
                image_topic=topic,
                image_path=image_path,
                camera_id=int(metadata.get("camera_id", infer_camera_id(topic, len(frames)))),
                image_bgr=image_bgr,
                metadata=metadata,
            )
        )
    if camera_ids:
        by_camera_id = {frame.camera_id: frame for frame in frames}
        missing = [camera_id for camera_id in camera_ids if camera_id not in by_camera_id]
        if missing:
            raise RuntimeError(f"missing cached frames for camera ids: {missing}")
        return [by_camera_id[camera_id] for camera_id in camera_ids]

    frames.sort(key=lambda frame: (frame.camera_id, frame.image_topic))
    return frames[:rois_number]


def apply_projection_overrides(frames: list[CachedFrame], projection_json: Path) -> None:
    data = json.loads(projection_json.expanduser().read_text(encoding="utf-8"))
    for frame in frames:
        candidates = [
            str(frame.camera_id),
            f"camera_{frame.camera_id}",
            frame.image_topic,
        ]
        override = None
        for key in candidates:
            if key in data:
                override = data[key]
                break
        if override is None:
            continue
        if isinstance(override, list) and len(override) == 16:
            frame.metadata["lidar2img"] = override
        elif isinstance(override, dict):
            if "lidar2img" in override:
                frame.metadata["lidar2img"] = override["lidar2img"]
            if "img2lidar" in override:
                frame.metadata["img2lidar"] = override["img2lidar"]


def infer_camera_id(topic: str, fallback: int) -> int:
    match = re.search(r"camera[_/\-]?(\d+)", topic, re.IGNORECASE)
    return int(match.group(1)) if match else fallback


def preprocess_image(image_bgr: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
    in_h, in_w = image_bgr.shape[:2]
    resize = max(out_h / in_h, out_w / in_w)
    new_w = int(in_w * resize)
    new_h = int(in_h * resize)
    resized = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    crop_h = max(0, new_h - out_h)
    crop_w = max(0, (new_w - out_w) // 2)
    cropped = resized[crop_h : crop_h + out_h, crop_w : crop_w + out_w]
    if cropped.shape[0] != out_h or cropped.shape[1] != out_w:
        cropped = cv2.copyMakeBorder(
            cropped,
            0,
            max(0, out_h - cropped.shape[0]),
            0,
            max(0, out_w - cropped.shape[1]),
            cv2.BORDER_CONSTANT,
            value=(0, 0, 0),
        )[:out_h, :out_w]
    image = cropped.astype(np.float32)
    mean = np.array([103.530, 116.280, 123.675], dtype=np.float32)
    std = np.array([57.375, 57.120, 58.395], dtype=np.float32)
    image = (image - mean) / std
    return image.transpose(2, 0, 1)


def build_intrinsics(frames: list[CachedFrame], out_h: int, out_w: int) -> np.ndarray:
    intrinsics = []
    for frame in frames:
        matrix = matrix_from_metadata(frame.metadata, "intrinsics")
        if matrix is None:
            matrix = camera_info_projection(frame.metadata)
        if matrix is None:
            matrix = np.eye(4, dtype=np.float32)
        intrinsics.append(adjust_intrinsic_for_preprocess(matrix, frame.image_bgr.shape[:2], out_h, out_w))
    return np.stack(intrinsics, axis=0)


def build_img2lidar(frames: list[CachedFrame], allow_identity: bool) -> np.ndarray:
    matrices = []
    for frame in frames:
        matrix = matrix_from_metadata(frame.metadata, "img2lidar")
        if matrix is None:
            lidar2img = matrix_from_metadata(frame.metadata, "lidar2img")
            if lidar2img is not None:
                matrix = np.linalg.inv(lidar2img)
        if matrix is None:
            if not allow_identity:
                print(f"warning: missing img2lidar for {frame.image_topic}; using identity for ONNX input")
            matrix = np.eye(4, dtype=np.float32)
        matrices.append(matrix.astype(np.float32))
    return np.stack(matrices, axis=0)


def projection_for_frame(frame: CachedFrame, allow_identity: bool) -> Optional[np.ndarray]:
    lidar2img = matrix_from_metadata(frame.metadata, "lidar2img")
    if lidar2img is not None:
        return lidar2img
    img2lidar = matrix_from_metadata(frame.metadata, "img2lidar")
    if img2lidar is not None:
        return np.linalg.inv(img2lidar)
    if allow_identity:
        return np.eye(4, dtype=np.float32)
    return None


def matrix_from_metadata(metadata: dict[str, object], key: str) -> Optional[np.ndarray]:
    value = metadata.get(key)
    if not isinstance(value, list) or len(value) != 16:
        return None
    return np.array(value, dtype=np.float32).reshape(4, 4)


def camera_info_projection(metadata: dict[str, object]) -> Optional[np.ndarray]:
    p = metadata.get("camera_info_p")
    if not isinstance(p, list) or len(p) != 12:
        return None
    return np.array(
        [[p[0], p[1], p[2], p[3]], [p[4], p[5], p[6], p[7]], [p[8], p[9], p[10], p[11]], [0, 0, 0, 1]],
        dtype=np.float32,
    )


def adjust_intrinsic_for_preprocess(
    intrinsic: np.ndarray, original_shape: tuple[int, int], out_h: int, out_w: int
) -> np.ndarray:
    raw_h, raw_w = original_shape
    resize = max(out_h / raw_h, out_w / raw_w)
    new_w = int(raw_w * resize)
    new_h = int(raw_h * resize)
    crop_h = max(0, new_h - out_h)
    crop_w = max(0, (new_w - out_w) // 2)
    adjusted = intrinsic.copy()
    adjusted[0, :] *= resize
    adjusted[1, :] *= resize
    adjusted[0, 2] -= crop_w
    adjusted[1, 2] -= crop_h
    return adjusted.astype(np.float32)


def postprocess_boxes(
    cls_scores: np.ndarray,
    bbox_preds: np.ndarray,
    score_thresholds: list[float],
    yaw_norm_thresholds: list[float],
    detection_range: list[float],
    max_boxes: int,
) -> list[Box3D]:
    cls = np.asarray(cls_scores)[0]
    bbox = np.asarray(bbox_preds)[0]
    num_classes, num_proposals = cls.shape
    boxes = []
    for point_idx in range(num_proposals):
        scores = sigmoid(cls[:, point_idx])
        label = int(np.argmax(scores))
        score = float(scores[label])
        score_threshold = score_thresholds[label] if label < len(score_thresholds) else score_thresholds[-1]
        if score < score_threshold:
            continue
        yaw_sin = float(bbox[6, point_idx])
        yaw_cos = float(bbox[7, point_idx])
        yaw_norm = math.sqrt(yaw_sin * yaw_sin + yaw_cos * yaw_cos)
        yaw_threshold = (
            yaw_norm_thresholds[label] if label < len(yaw_norm_thresholds) else yaw_norm_thresholds[-1]
        )
        if yaw_norm < yaw_threshold:
            continue
        x, y, z = float(bbox[0, point_idx]), float(bbox[1, point_idx]), float(bbox[2, point_idx])
        if not (
            detection_range[0] <= x <= detection_range[3]
            and detection_range[1] <= y <= detection_range[4]
            and detection_range[2] <= z <= detection_range[5]
        ):
            continue
        boxes.append(
            Box3D(
                x=x,
                y=y,
                z=z,
                width=math.exp(float(bbox[3, point_idx])),
                length=math.exp(float(bbox[4, point_idx])),
                height=math.exp(float(bbox[5, point_idx])),
                yaw=math.atan2(yaw_sin, yaw_cos),
                score=score,
                label=label,
            )
        )
    boxes.sort(key=lambda box: box.score, reverse=True)
    return boxes[:max_boxes]


def sigmoid(values: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-values))


def draw_projected_boxes(
    image_bgr: np.ndarray,
    boxes: list[Box3D],
    lidar2img: np.ndarray,
    class_names: list[str],
    max_depth: float,
) -> np.ndarray:
    for box in boxes:
        corners = box_corners(box)
        projected = project_points(corners, lidar2img)
        if projected is None:
            continue
        if np.any(projected[:, 2] <= 0.1) or np.all(projected[:, 2] > max_depth):
            continue
        points = projected[:, :2].astype(np.int32)
        draw_box_edges(image_bgr, points)
        label = class_names[box.label] if box.label < len(class_names) else str(box.label)
        text = f"{label} {box.score:.2f}"
        origin = tuple(points[np.argmin(points[:, 1])])
        cv2.putText(image_bgr, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 255), 1)
    return image_bgr


def box_corners(box: Box3D) -> np.ndarray:
    dx = box.width / 2.0
    dy = box.length / 2.0
    dz = box.height / 2.0
    local = np.array(
        [
            [dx, dy, dz],
            [dx, -dy, dz],
            [-dx, -dy, dz],
            [-dx, dy, dz],
            [dx, dy, -dz],
            [dx, -dy, -dz],
            [-dx, -dy, -dz],
            [-dx, dy, -dz],
        ],
        dtype=np.float32,
    )
    c = math.cos(box.yaw)
    s = math.sin(box.yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    return local @ rot.T + np.array([box.x, box.y, box.z], dtype=np.float32)


def project_points(points_xyz: np.ndarray, lidar2img: np.ndarray) -> Optional[np.ndarray]:
    points_h = np.concatenate([points_xyz, np.ones((points_xyz.shape[0], 1), dtype=np.float32)], axis=1)
    projected = points_h @ lidar2img.T
    depth = projected[:, 2]
    if np.all(np.abs(depth) < 1e-6):
        return None
    uv = projected[:, :2] / depth[:, None]
    return np.concatenate([uv, depth[:, None]], axis=1)


def draw_box_edges(image_bgr: np.ndarray, points: np.ndarray) -> None:
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]
    for start, end in edges:
        cv2.line(image_bgr, tuple(points[start]), tuple(points[end]), (0, 255, 255), 2, cv2.LINE_AA)


def save_boxes_json(path: Path, boxes: list[Box3D]) -> None:
    path.write_text(
        json.dumps([box.__dict__ for box in boxes], indent=2),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
