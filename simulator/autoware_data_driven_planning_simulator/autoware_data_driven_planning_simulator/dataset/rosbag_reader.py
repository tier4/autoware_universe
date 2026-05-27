"""Small ROS 2 bag reader wrapper for sqlite3 and MCAP bags."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


@dataclass
class BagMessage:
    topic: str
    timestamp_ns: int
    message: object


def infer_storage_id(uri: str) -> str:
    path = Path(uri)
    if path.suffix == ".mcap":
        return "mcap"
    if path.suffix == ".db3":
        return "sqlite3"
    metadata = path / "metadata.yaml"
    if metadata.exists():
        text = metadata.read_text()
        if "storage_identifier: mcap" in text:
            return "mcap"
    return "sqlite3"


def read_bag(uri: str, topics: set[str] | None = None) -> Iterator[BagMessage]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("rosbag2_py, rclpy, and rosidl_runtime_py are required to read bags") from exc

    storage_id = infer_storage_id(uri)
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topics is not None and topic not in topics:
            continue
        msg_type = get_message(topic_types[topic])
        yield BagMessage(topic=topic, timestamp_ns=timestamp_ns, message=deserialize_message(data, msg_type))

