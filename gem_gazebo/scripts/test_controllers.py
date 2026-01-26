#!/usr/bin/env python3

"""
Test script to verify the controllers are working properly.
This script sends simple commands to the steering and wheel controllers.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ControllerTester(Node):
    def __init__(self):
        super().__init__('controller_tester')
        
        # Create publishers for each controller - using Float64MultiArray for JointGroup controllers
        self.left_steering_pub = self.create_publisher(
            Float64MultiArray, '/left_steering_controller/commands', 10)
        self.right_steering_pub = self.create_publisher(
            Float64MultiArray, '/right_steering_controller/commands', 10)
        self.left_front_wheel_pub = self.create_publisher(
            Float64MultiArray, '/left_front_wheel_controller/commands', 10)
        self.right_front_wheel_pub = self.create_publisher(
            Float64MultiArray, '/right_front_wheel_controller/commands', 10)
        self.left_rear_wheel_pub = self.create_publisher(
            Float64MultiArray, '/left_rear_wheel_controller/commands', 10)
        self.right_rear_wheel_pub = self.create_publisher(
            Float64MultiArray, '/right_rear_wheel_controller/commands', 10)
        
        # Create a timer that publishes commands at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Alternate between turning left and right, and moving forward and backward
        self.direction = 1
        self.count = 0
    
    def timer_callback(self):
        # Create messages - Float64MultiArray for JointGroup controllers
        steering_msg_left = Float64MultiArray()
        steering_msg_right = Float64MultiArray()
        wheel_msg_left_front = Float64MultiArray()
        wheel_msg_right_front = Float64MultiArray()
        wheel_msg_left_rear = Float64MultiArray()
        wheel_msg_right_rear = Float64MultiArray()
        
        # Every 50 iterations (5 seconds), change direction
        if self.count % 50 == 0:
            self.direction *= -1
            self.get_logger().info(f"Changing direction to {self.direction}")
        
        # Set steering angle (positive is left, negative is right)
        steering_angle = 0.3 * self.direction
        
        # Set wheel velocity (positive is forward, negative is backward)
        wheel_velocity = 2.0 * self.direction
        
        # Set the data for each message (array of commands for each joint)
        steering_msg_left.data = [steering_angle]
        steering_msg_right.data = [steering_angle]
        wheel_msg_left_front.data = [wheel_velocity]
        wheel_msg_right_front.data = [wheel_velocity]
        wheel_msg_left_rear.data = [wheel_velocity]
        wheel_msg_right_rear.data = [wheel_velocity]
        
        # Publish commands
        self.left_steering_pub.publish(steering_msg_left)
        self.right_steering_pub.publish(steering_msg_right)
        self.left_front_wheel_pub.publish(wheel_msg_left_front)
        self.right_front_wheel_pub.publish(wheel_msg_right_front)
        self.left_rear_wheel_pub.publish(wheel_msg_left_rear)
        self.right_rear_wheel_pub.publish(wheel_msg_right_rear)
        
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    controller_tester = ControllerTester()
    rclpy.spin(controller_tester)
    controller_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 