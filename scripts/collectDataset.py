#!/usr/bin/env python3
"""Save camera frames from Clover/Gazebo for the FireSafety dataset."""

from __future__ import annotations

import argparse
import re
import time
from pathlib import Path
from threading import Lock

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Collector:
    def __init__(self, topic: str, output_dir: Path, quality: int) -> None:
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.quality = quality
        self.lock = Lock()
        self.frame = None
        self.count = 0
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.subscriber = rospy.Subscriber(topic, Image, self._on_image, queue_size=1)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("Camera conversion failed: %s", exc)
            return
        with self.lock:
            self.frame = frame

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def save(self, label: str) -> Path | None:
        frame = self.latest()
        if frame is None:
            return None
        self.count += 1
        safe_label = re.sub(r"[^a-zA-Z0-9_.-]+", "_", label).strip("_") or "frame"
        path = self.output_dir / f"{self.count:06d}_{safe_label}_{time.time_ns()}.jpg"
        ok = cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return path if ok else None


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/main_camera/image_raw")
    parser.add_argument("--output-dir", type=Path, default=project_root / "dataset" / "images_raw")
    parser.add_argument("--label", default="firesafety")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("firesafety_dataset_collector")
    collector = Collector(args.image_topic, args.output_dir, args.jpeg_quality)
    deadline = time.time() + args.duration
    next_save = 0.0
    while not rospy.is_shutdown() and time.time() < deadline:
        now = time.time()
        if now >= next_save:
            path = collector.save(args.label)
            if path:
                rospy.loginfo("Saved %s", path)
            next_save = now + args.interval
        rospy.sleep(0.05)
    rospy.loginfo("Saved %d images to %s", collector.count, args.output_dir)


if __name__ == "__main__":
    main()
