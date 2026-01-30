import argparse
import os
from pynput import keyboard
import torch
import rich

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDrive

from dataset import CaptureDataset
from model_utils import load_model, inference


class DriveControl(Node):
    def __init__(self, args) -> None:
        super().__init__("DriveControl")

        self._world_tf = args.world_tf
        self._cap_tf = args.cap_tf
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._image_msg = None
        self._cap_ds = CaptureDataset(args.dataset_path)
        self._keypresses = set()
        self._cv_bridge = CvBridge()
        self._max_speed = args.max_speed
        self._max_steer = args.max_steer
        self._dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        try:
            self._model = load_model()
            if self._model is not None:
                self._model = self._model.to(self._dev)
                rich.print("[green]loaded SimpleEnet :o")
        except Exception as e:
            self.get_logger().error(f"could not load SimpleEnet model x_X: {e}")

        self._mask_pub = self.create_publisher(
            Image,
            args.mask_topic,
            10
        )  
        self.create_subscription(
            Image,
            args.camera_topic,
            self._on_image,
            10
        )
        self._drive_pub = self.create_publisher(
            AckermannDrive,
            args.ackermann_topic,
            10
        )
        self.create_timer(1/50, self._timer_callback)

        self._keylogger = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
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
        except:
            return
        if c in "wasd":
            self._keypresses.add(c)
        else:
            match c:
                case "c":
                    self._capture()
                case "q":
                    self._shutdown()

    def _on_release(self, key):
        try:
            c = key.char
        except:
            return
        if c in "wasd":
            self._keypresses.discard(c)

    def _on_image(self, msg) -> None:
        self._image_msg = msg
        if self._model is not None:
            image = self._cv_bridge.imgmsg_to_cv2(self._image_msg, "bgr8")
            mask = inference(self._model, image, self._dev)
            import numpy as np
            import cv2
            m = mask.astype(np.uint8) * 255
            msg = self._cv_bridge.cv2_to_imgmsg(m, "mono8")
            self._mask_pub.publish(msg)

    def _capture(self) -> None:
        try:
            now = rclpy.time.Time()
            trans = self._tf_buf.lookup_transform(
                self._world_tf,
                self._cap_tf, 
                time=now)
            
            image = self._cv_bridge.imgmsg_to_cv2(self._image_msg, "bgr8")

            pos = trans.transform.translation
            rot = trans.transform.rotation

            pose = {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z,
                "qx": rot.x,
                "qy": rot.y,
                "qz": rot.z,
                "qw": rot.w
            }
            self._cap_ds.capture(image, pose)
            print(f"\nCapturing image (total: {len(self._cap_ds)}) at ({pose['x']:03f}, {pose['y']:03f}, {pose['z']:03f})")
            
        except TransformException as e:
            self.get_logger().error(f"\nCould not get transform: {e}")
            print("Maybe wait a few seconds? o.O")
    
    def _shutdown(self):
        print("Exiting ...")
        self._keylogger.stop()
        rclpy.shutdown()


def print_instructions(args):
    print(f"""
Ackermann Drive Keyboard Teleop Control
----------------------------------------
Control Keys:
  w - Move forward (hold)
  s - Move backward (hold)
  a - Steer left (hold)
  d - Steer right (hold)
  q - Quit
  c - Capture and save current camera image
  
Current Settings:
  Max Speed: {args.max_speed} m/s
  Max Steering Angle: {args.max_steer} rad""")


def main():
    parser = argparse.ArgumentParser(description="Teleop control of GEM")
    parser.add_argument(
        "--world_tf",
        default="silverstone",
        help="Name of world_tf"
    )
    parser.add_argument(
        "--cap_tf",
        default="gem",
        help="Name of capture_tf"
    )
    parser.add_argument(
        "--dataset_path",
        default=os.path.join("data", "capture"),
        help="Path to store captured data"
    )
    parser.add_argument(
        "--camera_topic",
        default="/camera/image_raw",
        help="Topic name of camera stream"
    )
    parser.add_argument(
        "--ackermann_topic",
        default="/ackermann_cmd",
        help="Topic name of ackermann cmd topic"
    )
    parser.add_argument(
        "--max_speed",
        default=5.0,
        help="Max speed of gem"
    )
    parser.add_argument(
        "--max_steer",
        default=0.8,
        help="Max steer of gem"
    )
    parser.add_argument(
        "--mask_topic",
        default="/lane_mask",
        help="publishing topic of lane line mask"
    )
    args = parser.parse_args()

    print_instructions(args)

    rclpy.init()
    node = DriveControl(args)
    rclpy.spin(node)
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()