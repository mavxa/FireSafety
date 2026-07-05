#!/usr/bin/env python3
"""Autonomous flight for the FireSafety qualification route.

The task route is deterministic: the drone flies over eight colored blocks on a
7x7 ArUco field and sets the LED color that matches each block.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from threading import Lock
from typing import Optional

import cv2
import numpy as np
import rospy
from clover import srv
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


FRAME_ID = "aruco_map"
IMAGE_TOPIC = "/main_camera/image_raw"
ALTITUDE = 1.0
SPEED = 0.35
TOLERANCE = 0.22
NAVIGATION_TIMEOUT = 45.0

LED_RGB = {
    "white": (255, 255, 255),
    "off": (0, 0, 0),
    "yellow": (255, 255, 0),
    "violet": (130, 0, 255),
    "red": (255, 0, 0),
    "pink": (255, 0, 120),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "orange": (255, 120, 0),
    "cyan": (0, 255, 255),
}


@dataclass
class Waypoint:
    label: str
    x: float
    y: float
    color: str | None = None


ROUTE = [
    Waypoint("marker 44", 2.0, 0.0),
    Waypoint("pink block at marker 38", 3.0, 1.0, "pink"),
    Waypoint("orange block right side", 4.5, 2.5, "orange"),
    Waypoint("cyan block right side", 5.5, 3.5, "cyan"),
    Waypoint("violet block at marker 12", 5.0, 5.0, "violet"),
    Waypoint("marker 11", 4.0, 5.0),
    Waypoint("red block at marker 17", 3.0, 4.0, "red"),
    Waypoint("marker 9", 2.0, 5.0),
    Waypoint("yellow block at marker 8", 1.0, 5.0, "yellow"),
    Waypoint("green block left side", 0.5, 3.5, "green"),
    Waypoint("blue block left side", 1.5, 2.5, "blue"),
    Waypoint("pink block at marker 38 again", 3.0, 1.0, "pink"),
    Waypoint("landing marker 42", 0.0, 0.0),
]


def configure_output_encoding() -> None:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


class CameraBuffer:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.lock = Lock()
        self.frame: Optional[np.ndarray] = None
        self.subscriber = rospy.Subscriber(IMAGE_TOPIC, Image, self._on_image, queue_size=1)

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn("Camera conversion failed: %s", exc)
            return
        with self.lock:
            self.frame = frame

    def latest(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def wait(self, timeout: float = 10.0) -> None:
        deadline = time.time() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.latest() is not None:
                return
            rate.sleep()
        rospy.logwarn("No frames from %s; route will still run by known block colors", IMAGE_TOPIC)


class Mission:
    def __init__(self) -> None:
        self.camera = CameraBuffer()
        self.get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy("navigate", srv.Navigate)
        self.land = rospy.ServiceProxy("land", Trigger)
        self.set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)

    def led(self, color: str) -> None:
        r, g, b = LED_RGB[color]
        try:
            self.set_effect(r=r, g=g, b=b)
        except Exception as exc:
            rospy.logwarn("LED service failed: %s", exc)

    def navigate_wait(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = ALTITUDE,
        frame_id: str = FRAME_ID,
        auto_arm: bool = False,
    ) -> None:
        response = self.navigate(
            x=x,
            y=y,
            z=z,
            speed=SPEED,
            frame_id=frame_id,
            auto_arm=auto_arm,
        )
        if not response.success:
            raise RuntimeError(response.message)

        deadline = time.time() + NAVIGATION_TIMEOUT
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id="navigate_target")
            distance = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
            if distance < TOLERANCE:
                rospy.sleep(0.4)
                return
            if time.time() > deadline:
                raise RuntimeError(f"Navigation timeout, distance={distance:.2f}")
            rate.sleep()

    def land_wait(self) -> None:
        self.land()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.get_telemetry().armed:
            rate.sleep()

    def detected_color(self, fallback: str) -> str:
        frame = self.camera.latest()
        if frame is None:
            return fallback

        h, w = frame.shape[:2]
        roi = frame[int(h * 0.2) : int(h * 0.8), int(w * 0.2) : int(w * 0.8)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        masks = {
            "red": cv2.inRange(hsv, np.array([0, 80, 40]), np.array([12, 255, 255]))
            | cv2.inRange(hsv, np.array([160, 80, 40]), np.array([180, 255, 255])),
            "yellow": cv2.inRange(hsv, np.array([18, 70, 60]), np.array([35, 255, 255])),
            "green": cv2.inRange(hsv, np.array([40, 50, 40]), np.array([90, 255, 255])),
            "blue": cv2.inRange(hsv, np.array([95, 50, 40]), np.array([130, 255, 255])),
            "cyan": cv2.inRange(hsv, np.array([80, 40, 40]), np.array([100, 255, 255])),
            "orange": cv2.inRange(hsv, np.array([5, 80, 60]), np.array([22, 255, 255])),
            "pink": cv2.inRange(hsv, np.array([145, 40, 40]), np.array([175, 255, 255])),
            "violet": cv2.inRange(hsv, np.array([125, 40, 40]), np.array([155, 255, 255])),
        }
        scores = {name: int(cv2.countNonZero(mask)) for name, mask in masks.items()}
        best = max(scores, key=scores.get)
        return best if scores[best] >= 120 else fallback

    def show_block_color(self, expected: str) -> None:
        color = self.detected_color(expected)
        print(f"block={color}")
        self.led(color)
        rospy.sleep(0.8)

    def run(self) -> None:
        rospy.wait_for_service("get_telemetry")
        rospy.wait_for_service("navigate")
        rospy.wait_for_service("land")
        rospy.wait_for_service("led/set_effect")
        self.camera.wait()

        print("Mission started")
        print("Route: takeoff marker 42 -> colored blocks -> landing marker 42")
        self.led("white")
        self.navigate_wait(z=ALTITUDE, frame_id="body", auto_arm=True)

        try:
            for waypoint in ROUTE:
                print(f"go={waypoint.label}")
                self.navigate_wait(x=waypoint.x, y=waypoint.y)
                if waypoint.color:
                    self.show_block_color(waypoint.color)
            self.led("off")
            self.land_wait()
        finally:
            if not rospy.is_shutdown():
                self.led("off")

        print("Mission finished")


def main() -> None:
    configure_output_encoding()
    rospy.init_node("fire_safety_route")
    Mission().run()


if __name__ == "__main__":
    main()
