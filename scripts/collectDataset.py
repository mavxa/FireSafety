#!/usr/bin/env python3
"""Autonomous camera-frame collection for the FireSafety dataset."""

from __future__ import annotations

import argparse
import math
import re
import time
from pathlib import Path
from threading import Lock

import cv2
import rospy
from clover import srv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


WAYPOINTS = [
    ("takeoff_area", 0.0, 0.0),
    ("yellow_marker_8", 1.0, 5.0),
    ("violet_marker_12", 5.0, 5.0),
    ("red_marker_17", 3.0, 4.0),
    ("pink_marker_38", 3.0, 1.0),
    ("green_left", 0.5, 3.5),
    ("blue_left", 1.5, 2.5),
    ("orange_right", 4.5, 2.5),
    ("cyan_right", 5.5, 3.5),
    ("center", 3.0, 3.0),
]


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

    def wait_for_frame(self, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.latest() is not None:
                return
            rate.sleep()
        raise RuntimeError("No camera frames received")

    def save(self, label: str) -> Path | None:
        frame = self.latest()
        if frame is None:
            return None
        self.count += 1
        safe_label = re.sub(r"[^a-zA-Z0-9_.-]+", "_", label).strip("_") or "frame"
        path = self.output_dir / f"{self.count:06d}_{safe_label}_{time.time_ns()}.jpg"
        ok = cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return path if ok else None


def navigate_wait(
    navigate,
    get_telemetry,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 1.0,
    frame_id: str = "aruco_map",
    speed: float = 0.35,
    tolerance: float = 0.22,
    timeout: float = 45.0,
    auto_arm: bool = False,
) -> None:
    response = navigate(
        x=x,
        y=y,
        z=z,
        speed=speed,
        frame_id=frame_id,
        auto_arm=auto_arm,
    )
    if not response.success:
        raise RuntimeError(response.message)

    deadline = time.time() + timeout
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id="navigate_target")
        distance = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
        if distance < tolerance:
            rospy.sleep(0.4)
            return
        if time.time() > deadline:
            raise RuntimeError(f"Navigation timeout, distance={distance:.2f}")
        rate.sleep()


def capture_burst(collector: Collector, label: str, count: int, interval: float) -> None:
    for _ in range(count):
        if rospy.is_shutdown():
            return
        path = collector.save(label)
        if path:
            rospy.loginfo("Saved %s", path)
        rospy.sleep(interval)


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/main_camera/image_raw")
    parser.add_argument("--output-dir", type=Path, default=project_root / "dataset" / "images_raw")
    parser.add_argument("--frame-id", default="aruco_map")
    parser.add_argument("--altitude", type=float, default=1.15)
    parser.add_argument("--takeoff-altitude", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--shots-per-point", type=int, default=10)
    parser.add_argument("--shot-interval", type=float, default=0.25)
    parser.add_argument("--settle-time", type=float, default=0.6)
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--skip-flight", action="store_true")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--skip-land", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("firesafety_dataset_collector")
    collector = Collector(args.image_topic, args.output_dir, args.jpeg_quality)
    collector.wait_for_frame()

    if args.skip_flight:
        deadline = time.time() + args.duration
        next_save = 0.0
        while not rospy.is_shutdown() and time.time() < deadline:
            now = time.time()
            if now >= next_save:
                collector.save("static")
                next_save = now + args.interval
            rospy.sleep(0.05)
        rospy.loginfo("Saved %d images to %s", collector.count, args.output_dir)
        return

    rospy.wait_for_service("get_telemetry")
    rospy.wait_for_service("navigate")
    rospy.wait_for_service("land")
    get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
    navigate = rospy.ServiceProxy("navigate", srv.Navigate)
    land = rospy.ServiceProxy("land", Trigger)

    points = WAYPOINTS[: args.max_points] if args.max_points > 0 else WAYPOINTS
    rospy.loginfo("Taking off")
    navigate_wait(
        navigate,
        get_telemetry,
        z=args.takeoff_altitude,
        frame_id="body",
        speed=args.speed,
        auto_arm=True,
    )

    try:
        for label, x, y in points:
            rospy.loginfo("Navigate to %s: x=%.2f y=%.2f", label, x, y)
            navigate_wait(
                navigate,
                get_telemetry,
                x=x,
                y=y,
                z=args.altitude,
                frame_id=args.frame_id,
                speed=args.speed,
            )
            rospy.sleep(args.settle_time)
            capture_burst(collector, label, args.shots_per_point, args.shot_interval)
    finally:
        if not args.skip_land and not rospy.is_shutdown():
            rospy.loginfo("Landing")
            try:
                navigate_wait(
                    navigate,
                    get_telemetry,
                    x=0.0,
                    y=0.0,
                    z=args.altitude,
                    frame_id=args.frame_id,
                    speed=args.speed,
                )
                land()
            except Exception as exc:
                rospy.logwarn("Landing failed: %s", exc)

    rospy.loginfo("Saved %d images to %s", collector.count, args.output_dir)


if __name__ == "__main__":
    main()
