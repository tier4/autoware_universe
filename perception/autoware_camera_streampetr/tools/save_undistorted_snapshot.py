#!/usr/bin/env python3

# Copyright 2026 TIER IV
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Save a StreamPETR-style undistorted full-resolution camera snapshot.

This is intentionally a standalone helper. It does not import or modify the
StreamPETR node; it only subscribes to an image topic and its CameraInfo topic,
then reproduces the node's OpenCV undistortion map generation:

  cv2.initUndistortRectifyMap(K, D, None, P[0:3, 0:3], full_resolution, CV_32FC1)
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class UndistortedSnapshotSaver(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("streampetr_undistorted_snapshot_saver")
        self._camera_info: Optional[CameraInfo] = None
        self._output_path = Path(args.output).expanduser()
        self._saved = False
        self._use_compressed = args.compressed

        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._on_camera_info, qos_profile_sensor_data
        )
        image_msg_type = CompressedImage if self._use_compressed else Image
        self.create_subscription(
            image_msg_type, args.image_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info(f"waiting for camera_info: {args.camera_info_topic}")
        self.get_logger().info(f"waiting for image: {args.image_topic}")
        self.get_logger().info(f"output: {self._output_path}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_image(self, msg: Image | CompressedImage) -> None:
        if self._saved:
            return
        if self._camera_info is None:
            self.get_logger().warn("image received before camera_info; waiting", throttle_duration_sec=2.0)
            return

        if self._use_compressed:
            image_bgr = _compressed_image_to_bgr(msg)
        else:
            image_bgr = _image_msg_to_bgr(msg)

        undistorted = self._undistort_like_streampetr(image_bgr, self._camera_info)
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self._output_path), undistorted):
            raise RuntimeError(f"failed to write image: {self._output_path}")

        self._saved = True
        self.get_logger().info(f"saved undistorted image: {self._output_path}")
        rclpy.shutdown()

    @staticmethod
    def _undistort_like_streampetr(image_bgr: np.ndarray, camera_info: CameraInfo) -> np.ndarray:
        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(camera_info.d, dtype=np.float64).reshape(1, len(camera_info.d))
        p = np.array(
            [
                [camera_info.p[0], camera_info.p[1], camera_info.p[2]],
                [camera_info.p[4], camera_info.p[5], camera_info.p[6]],
                [camera_info.p[8], camera_info.p[9], camera_info.p[10]],
            ],
            dtype=np.float64,
        )

        width = int(camera_info.width)
        height = int(camera_info.height)
        if image_bgr.shape[1] != width or image_bgr.shape[0] != height:
            raise RuntimeError(
                "image size does not match camera_info "
                f"({image_bgr.shape[1]}x{image_bgr.shape[0]} != {width}x{height})"
            )

        map_x, map_y = cv2.initUndistortRectifyMap(
            k, d, None, p, (width, height), cv2.CV_32FC1
        )
        return cv2.remap(image_bgr, map_x, map_y, interpolation=cv2.INTER_LINEAR)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save one full-resolution undistorted image using StreamPETR camera_info math."
    )
    parser.add_argument("--image-topic", required=True, help="sensor_msgs/Image topic to capture")
    parser.add_argument(
        "--camera-info-topic", required=True, help="sensor_msgs/CameraInfo topic paired with image"
    )
    parser.add_argument(
        "--output",
        default="/tmp/streampetr_undistorted_snapshot.png",
        help="PNG/JPEG output path",
    )
    parser.add_argument(
        "--compressed",
        action="store_true",
        help="Subscribe to sensor_msgs/CompressedImage instead of sensor_msgs/Image",
    )
    return parser.parse_args()


def _compressed_image_to_bgr(msg: CompressedImage) -> np.ndarray:
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    image_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError("failed to decode compressed image")
    return image_bgr


def _image_msg_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    if encoding not in channels_by_encoding:
        raise RuntimeError(f"unsupported image encoding for snapshot helper: {msg.encoding}")

    channels = channels_by_encoding[encoding]
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    row_size = width * channels
    if step < row_size:
        raise RuntimeError(
            f"invalid image step for {msg.encoding}: step={step}, expected at least {row_size}"
        )

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw.reshape((height, step))
    packed = rows[:, :row_size].reshape((height, width, channels))

    if encoding in ("bgr8", "8uc3"):
        return packed.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)
    if encoding in ("mono8", "8uc1"):
        return cv2.cvtColor(packed, cv2.COLOR_GRAY2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(packed, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(packed, cv2.COLOR_BGRA2BGR)

    raise RuntimeError(f"unsupported image encoding for snapshot helper: {msg.encoding}")


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = UndistortedSnapshotSaver(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
