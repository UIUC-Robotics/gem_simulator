#!/usr/bin/env python3
"""
Keyboard teleop: WASD -> ackermann_cmd. Run: ros2 run gem_gazebo drive.py
"""
import argparse
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from pynput import keyboard


class DriveControl(Node):
    def __init__(self, args) -> None:
        super().__init__("DriveControl")
        self._keypresses = set()
        self._max_speed = args.max_speed
        self._max_steer = args.max_steer
        self._drive_pub = self.create_publisher(
            AckermannDrive,
            args.ackermann_topic,
            10,
        )
        self.create_timer(1 / 50, self._timer_callback)
        self._keylogger = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._keylogger.start()

    def _timer_callback(self) -> None:
        speed = 0.0
        steer = 0.0
        if "w" in self._keypresses:
            speed = self._max_speed
        elif "s" in self._keypresses:
            speed = -self._max_speed
        if "a" in self._keypresses:
            steer = self._max_steer
        elif "d" in self._keypresses:
            steer = -self._max_steer

        msg = AckermannDrive()
        msg.speed = speed
        msg.steering_angle = steer
        msg.steering_angle_velocity = 0.0
        msg.acceleration = 0.0
        self._drive_pub.publish(msg)

    def _on_press(self, key):
        try:
            c = key.char
        except AttributeError:
            return
        if c in "wasd":
            self._keypresses.add(c)
        elif c == "q":
            self._shutdown()

    def _on_release(self, key):
        try:
            c = key.char
        except AttributeError:
            return
        if c in "wasd":
            self._keypresses.discard(c)

    def _shutdown(self):
        self._keylogger.stop()
        rclpy.shutdown()


def main():
    sys.argv = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(description="Keyboard teleop -> ackermann_cmd")
    parser.add_argument(
        "--ackermann_topic",
        default="/ackermann_cmd",
        help="Ackermann command topic",
    )
    parser.add_argument(
        "--max_speed",
        type=float,
        default=5.0,
        help="Max speed (m/s)",
    )
    parser.add_argument(
        "--max_steer",
        type=float,
        default=0.8,
        help="Max steering angle (rad)",
    )
    args = parser.parse_args()

    print("WASD: drive  |  q: quit  |  max_speed=%.1f  max_steer=%.2f" % (args.max_speed, args.max_steer))

    rclpy.init()
    node = DriveControl(args)
    rclpy.spin(node)
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
