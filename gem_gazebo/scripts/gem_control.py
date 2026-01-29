#!/usr/bin/env python3

#================================================================
# File name: gem_control.py                                                                  
# Description: take care of GEM control in Gazebo                                                             
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 06/05/2021                                                                
# Date last modified: 07/12/2021                                                          
# Version: 0.1                                                                    
# Usage: ros2 run gem_gazebo gem_control.py                                                                    
# Python version: 3.8                                                                                                      
#================================================================

import math
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import ListControllers


PI = 3.141592653589739


def get_steer_angle(phi):
    if phi >= 0.0:
        return (PI / 2) - phi
    return (-PI / 2) - phi


class GEMController(Node):

    def __init__(self):
        super().__init__('gem_ackermann_controller')

        # Initialize all variables at the beginning
        self.right_rear_link = None
        self.ackermann_cmd_lock = threading.Lock()  # Initialize the lock here
        self.steer_ang = 0.0   
        self.steer_ang_vel = 0.0   
        self.speed = 0.0
        self.accel = 0.0   
        self.last_steer_ang = 0.0   
        self.theta_left = 0.0   
        self.theta_left_old = 0.0
        self.theta_right = 0.0
        self.theta_right_old = 0.0   
        self.last_speed = 0.0
        self.last_accel_limit = 0.0   
        self.left_front_ang_vel = 0.0
        self.right_front_ang_vel = 0.0
        self.left_rear_ang_vel = 0.0
        self.right_rear_ang_vel = 0.0

        # Create a Buffer and TransformListener at the beginning
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        (left_steer_link, left_steer_controller, left_front_wheel_controller, self.left_front_inv_circ) = \
            self.get_front_wheel_params("left_front_wheel")

        (right_steer_link, right_steer_controller, right_front_wheel_controller, self.right_front_inv_circ) = \
            self.get_front_wheel_params("right_front_wheel")

        (left_rear_link, left_rear_wheel_controller, self.left_rear_inv_circ) =  \
            self.get_rear_wheel_params("left_rear_wheel")

        (right_rear_link, right_rear_wheel_controller, self.right_rear_inv_circ) =  \
            self.get_rear_wheel_params("right_rear_wheel")

        self.right_rear_link = right_rear_link

        # ROS2 service client for controller manager
        self.list_controllers_client = self.create_client(ListControllers, 'controller_manager/list_controllers')
        while not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('controller_manager/list_controllers service not available, waiting...')

        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value

        # Get link positions
        ls = self.get_link_position(left_steer_link) 
        rs = self.get_link_position(right_steer_link) 
        lrw = self.get_link_position(left_rear_link)
        rrw = np.array([0.0] * 3)

        self.steer_joint_dist_div_2 = np.linalg.norm(ls-rs)/2.0
        self.wheelbase = np.linalg.norm((ls+rs)/2.0 - (lrw+rrw)/2.0)
        # self.wheelbase = 1.75
        self.get_logger().info(f"Wheelbase set to {self.wheelbase:.2f} using transforms")
        self.wheelbase_inv = 1 / (self.wheelbase*1.0)
        self.wheelbase_sqr = self.wheelbase**2

        self.last_cmd_time = self.get_clock().now()

        # ROS2 publishers - use Float64MultiArray for JointGroup controllers
        self.left_steer_pub = self.create_publisher(Float64MultiArray, left_steer_controller + '/commands', 1)
        self.right_steer_pub = self.create_publisher(Float64MultiArray, right_steer_controller + '/commands', 1)
        self.left_front_wheel_pub = self.create_publisher(Float64MultiArray, left_front_wheel_controller + '/commands', 1)
        self.right_front_wheel_pub = self.create_publisher(Float64MultiArray, right_front_wheel_controller + '/commands', 1)
        self.left_rear_wheel_pub = self.create_publisher(Float64MultiArray, left_rear_wheel_controller + '/commands', 1)
        self.right_rear_wheel_pub = self.create_publisher(Float64MultiArray, right_rear_wheel_controller + '/commands', 1)
        
        # ROS2 subscription
        self.gem_ackermann_sub = self.create_subscription(
            AckermannDrive, 'ackermann_cmd', self.ackermann_callback, 1)
        
        # ROS2 Timer instead of rate sleep - set this up last
        self.timer = self.create_timer(1.0/30.0, self.update)  # 30Hz

    def update(self):
        t = self.get_clock().now()
        delta_t = (t - self.last_cmd_time).nanoseconds / 1e9

        # Check if command has timed out
        if self.cmd_timeout > 0.0 and delta_t > self.cmd_timeout:
            steer_ang_changed, center_y = self.control_steering(self.last_steer_ang, 0.0, 0.001)
            self.control_wheels(0.0, 0.0, 0.0, steer_ang_changed, center_y)
        elif delta_t > 0.0:
            with self.ackermann_cmd_lock:
                steer_ang = self.steer_ang
                steer_ang_vel = self.steer_ang_vel
                speed = self.speed
                accel = self.accel

            steer_ang_changed, center_y = self.control_steering(steer_ang, steer_ang_vel, delta_t)
            self.control_wheels(speed, accel, delta_t, steer_ang_changed, center_y)

        # Publish commands using Float64MultiArray
        left_steer_msg = Float64MultiArray()
        left_steer_msg.data = [self.theta_left]
        self.left_steer_pub.publish(left_steer_msg)
        
        right_steer_msg = Float64MultiArray()
        right_steer_msg.data = [self.theta_right]
        self.right_steer_pub.publish(right_steer_msg)

        if self.left_front_wheel_pub:
            left_front_msg = Float64MultiArray()
            left_front_msg.data = [self.left_front_ang_vel]
            self.left_front_wheel_pub.publish(left_front_msg)

        if self.right_front_wheel_pub:
            right_front_msg = Float64MultiArray()
            right_front_msg.data = [self.right_front_ang_vel]
            self.right_front_wheel_pub.publish(right_front_msg)

        if self.left_rear_wheel_pub:
            left_rear_msg = Float64MultiArray()
            left_rear_msg.data = [self.left_rear_ang_vel]
            self.left_rear_wheel_pub.publish(left_rear_msg)

        if self.right_rear_wheel_pub:
            right_rear_msg = Float64MultiArray()
            right_rear_msg.data = [self.right_rear_ang_vel]
            self.right_rear_wheel_pub.publish(right_rear_msg)

        self.last_cmd_time = t

    def ackermann_callback(self, ackermann_cmd):
        self.last_cmd_time = self.get_clock().now()
        with self.ackermann_cmd_lock:
            self.steer_ang = ackermann_cmd.steering_angle
            self.steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self.speed = ackermann_cmd.speed
            self.accel = ackermann_cmd.acceleration

    def get_front_wheel_params(self, prefix):
        steer_link = self.declare_parameter(prefix + '.steering_link_name', 'default').value
        steer_controller = self.declare_parameter(prefix + '.steering_controller_name', 'default').value
        wheel_controller = self.declare_parameter(prefix + '.axle_controller_name', 'default').value
        diameter = float(self.declare_parameter(prefix + '.diameter', 0.0).value)
        return steer_link, steer_controller, wheel_controller, 1 / (PI * diameter)

    def get_rear_wheel_params(self, prefix):
        link = self.declare_parameter(prefix + '.link_name', 'default').value
        wheel_controller = self.declare_parameter(prefix + '.axle_controller_name', 'default').value
        diameter = float(self.declare_parameter(prefix + '.diameter', 0.0).value)
        return link, wheel_controller, 1 / (PI * diameter)

    def get_link_position(self, link):
        max_attempts = 10
        attempt = 0
        retry_delay = 0.5  # seconds
        
        if not self.right_rear_link:
            # If right_rear_link is not set yet, use 'base_link' as a fallback
            self.get_logger().warn("right_rear_link is not set, using base_link instead")
            reference_frame = 'base_link'
        else:
            reference_frame = self.right_rear_link
            
        while attempt < max_attempts:
            try:
                # Wait for transform to be available with a timeout
                # self.get_logger().info(f"Looking up transform from {reference_frame} to {link}")
                trans = self.tf_buffer.lookup_transform(reference_frame, link, rclpy.time.Time())
                return np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            except Exception as e:
                attempt += 1
                self.get_logger().info(f"Waiting for transform: {str(e)}, attempt {attempt}/{max_attempts}")
                if attempt >= max_attempts:
                    self.get_logger().error(f"Could not get transform after {max_attempts} attempts, returning zeros, check if model spawned")
                    # Return zeros as a fallback to prevent complete failure
                    return np.array([0.0, 0.0, 0.0])
                rclpy.spin_once(self, timeout_sec=retry_delay)

    def control_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        if steer_ang_vel_limit > 0.0:
            ang_vel = (steer_ang - self.last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit, min(ang_vel, steer_ang_vel_limit))
            theta = self.last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        center_y = self.wheelbase * math.tan((PI/2)-theta)

        steer_ang_changed = theta != self.last_steer_ang

        if steer_ang_changed:
            self.last_steer_ang = theta
            self.theta_left = get_steer_angle(math.atan(self.wheelbase_inv * (center_y - self.steer_joint_dist_div_2)))
            self.theta_right = get_steer_angle(math.atan(self.wheelbase_inv * (center_y + self.steer_joint_dist_div_2)))

        return steer_ang_changed, center_y

    def control_wheels(self, speed, accel_limit, delta_t, steer_ang_changed, center_y):
        if accel_limit > 0.0:
            self.last_accel_limit = accel_limit
            accel = (speed - self.last_speed) / delta_t
            accel = max(-accel_limit, min(accel, accel_limit))
            veh_speed = self.last_speed + accel * delta_t
        else:
            self.last_accel_limit = accel_limit
            veh_speed = speed

        if veh_speed != self.last_speed or steer_ang_changed:
            self.last_speed = veh_speed
            left_dist = center_y - self.steer_joint_dist_div_2
            right_dist = center_y + self.steer_joint_dist_div_2
            gain = (2 * PI) * veh_speed / abs(center_y) 
            r = math.sqrt(left_dist ** 2 + self.wheelbase_sqr)
            self.left_front_ang_vel = gain * r * self.left_front_inv_circ
            r = math.sqrt(right_dist ** 2 + self.wheelbase_sqr)
            self.right_front_ang_vel = gain * r * self.right_front_inv_circ
            gain = (2 * PI) * veh_speed / center_y
            self.left_rear_ang_vel = gain * left_dist * self.left_rear_inv_circ
            self.right_rear_ang_vel = gain * right_dist * self.right_rear_inv_circ


def main(args=None):
    rclpy.init(args=args)
    controller = GEMController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()





