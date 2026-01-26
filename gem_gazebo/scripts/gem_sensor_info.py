#!/usr/bin/env python3

#================================================================
# File name: gem_sensor_info.py                                                                  
# Description: show sensor info in Rviz                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 06/10/2021                                                                
# Date last modified: 07/02/2021                                                          
# Version: 0.1                                                                    
# Usage: ros2 run gem_gazebo gem_sensor_info.py                                                                     
# Python version: 3.8                                                             
#================================================================

# Python Headers
import math
import random
import numpy as np

# ROS Headers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, Vector3
from rviz_2d_overlay_msgs.msg import OverlayText  # ROS 2 overlay text for RViz
from std_msgs.msg import ColorRGBA, Float32
from tf2_ros import Buffer, TransformListener
from transforms3d.euler import quat2euler  # You may need to install this package

# Gazebo Headers
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState


class GEMOverlay(Node):

    def __init__(self):
        super().__init__('gem_sensor_info')
          
        self.sensor_info_pub = self.create_publisher(OverlayText, "/gem/sensor_info", 1)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.update_info)  # 10Hz timer

        self.sensor_overlaytext = self.update_sensor_overlaytext()
        self.sensor_info_update = False

        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.imu_yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.gazebo_yaw = 0.0

        # Create a client for the GetEntityState service (ROS 2)
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/get_entity_state service not available, waiting...')
        
        self.pending_request = None


    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
        self.alt = round(msg.altitude, 6)


    def imu_callback(self, msg):
        orientation_q = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_accel = msg.linear_acceleration
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = quat2euler(orientation_list, 'sxyz')
        self.imu_yaw = round(yaw, 6)   


    def get_gem_state(self):
        # Check if there's a pending request and if it's done
        if self.pending_request is not None and self.pending_request.done():
            try:
                response = self.pending_request.result()
                if response is not None and response.success:
                    self.x = round(response.state.pose.position.x, 3)
                    self.y = round(response.state.pose.position.y, 3)

                    orientation_q = response.state.pose.orientation
                    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                    (roll, pitch, yaw) = quat2euler(orientation_list, 'sxyz')
                    self.gazebo_yaw = round(yaw, 3)

                    self.x_dot = round(response.state.twist.linear.x, 3)
                    self.y_dot = round(response.state.twist.linear.y, 3)
            except Exception as e:
                self.get_logger().error(f'Failed to get entity state: {str(e)}')
            
            self.pending_request = None
        
        # Start a new request if there isn't one pending
        if self.pending_request is None:
            request = GetEntityState.Request()
            request.name = 'gem'
            request.reference_frame = ''
            self.pending_request = self.get_entity_state_client.call_async(request)
        
        # Return current cached values
        return self.x, self.y, self.gazebo_yaw, self.x_dot, self.y_dot


    def update_sensor_overlaytext(self, lat=0.0, lon=0.0, alt=0.0, imu_yaw=0.0, x=0.0, y=0.0, gazebo_yaw=0.0, x_dot=0.0, y_dot=0.0, f_vel=0.0):
        text = OverlayText()
        text.width = 230
        text.height = 290
        text.text_size = 12.0
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = """----------------------
                       Sensor (Measurement):
                       Lat = %s
                       Lon = %s
                       Alt = %s
                       Yaw = %s
                       ----------------------
                       Gazebo (Ground Truth):
                       X     = %s
                       Y     = %s
                       X_dot = %s
                       Y_dot = %s
                       F_vel = %s
                       Yaw   = %s
                       ----------------------
                    """ % (str(lat), str(lon), str(alt), str(imu_yaw), str(x), str(y), str(x_dot), str(y_dot), str(f_vel), str(gazebo_yaw))
        text.fg_color = ColorRGBA(r=25 / 255.0, g=1.0, b=240.0 / 255.0, a=1.0)
        text.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.2)
        return text
    

    def update_sensor_overlaytext_only(self, new_text):
        self.sensor_overlaytext.text = new_text


    def update_info(self):
        self.x, self.y, self.gazebo_yaw, self.x_dot, self.y_dot = self.get_gem_state()
        f_vel = round(np.sqrt(self.x_dot**2 + self.y_dot**2), 3)

        if self.sensor_info_update:
            sensor_text = """----------------------
                             Sensor (Measurement):
                             Lat = %s
                             Lon = %s
                             Alt = %s
                             Yaw = %s
                             ----------------------
                             Gazebo (Ground Truth):
                             X_pos = %s
                             Y_pos = %s
                             X_dot = %s
                             Y_dot = %s
                             F_vel = %s
                             Yaw   = %s
                             ----------------------                               
                          """ % (str(self.lat), str(self.lon), str(self.alt), str(self.imu_yaw), \
                                 str(self.x), str(self.y), str(self.x_dot), str(self.y_dot), str(f_vel), str(self.gazebo_yaw))
            self.update_sensor_overlaytext_only(sensor_text)
        else:
            self.sensor_overlaytext = self.update_sensor_overlaytext()
            self.sensor_info_update = True

        self.sensor_info_pub.publish(self.sensor_overlaytext)
  
  
def main(args=None):
    rclpy.init(args=args)
    
    gem_overlay_node = GEMOverlay()
    
    try:
        rclpy.spin(gem_overlay_node)
    except KeyboardInterrupt:
        pass
    finally:
        gem_overlay_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()