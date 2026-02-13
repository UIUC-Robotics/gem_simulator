# Gem Autonomous Electric Vehicle Simulator in ROS2 with Gazebo Sim Ignition

This project features the simulation of a Polaris Gem E2 vehicle with **Ackermann steering capabilities**, developed using **ROS2** and the **Gazebo Sim Ignition environment**. The model integrates a variety of sensors and navigation tools for autonomous operation.

<!-- | **3D LiDAR Point Cloud Visualization** | **Warehouse Environment Model** |
| ----- | ----- |
| ![3D Point Cloud](saye_msgs/readme_files/3d_lidar_pointcloud.png) | ![Warehouse Model](saye_msgs/readme_files/warehouse_environment.png) | -->

## Table of Contents

<!--- - [Ackermann Steering Vehicle Simulation in ROS2 with Gazebo Sim Ignition](#ackermann-steering-vehicle-simulation-in-ros2-with-gazebo-sim-ignition) --->
- [Features](#features)

  - [1 Ackermann Steering](#1-ackermann-steering)
  - [2 ROS2 Communication](#2-ros2-communication)
  - [3 Sensors](#3-sensors)

  <!-- - [4 Navigation](#4-navigation) -->

  <!-- - [5 Manual Control with external joystick](#5-manual-control-with-external-joystick) -->

  - [4 Visualization](#4-visualization)
- [Requirements](#requirements)
- [Local Installation](#local-installation)
<!--- - [Docker Installation](#docker-installation) --->
- [Usage](#usage)

  - [1 Basic Simulation and Manual Control](#1-basic-simulation-and-manual-control)

  <!-- - [2 SLAM Simultaneous Localization and Mapping](#2-slam-simultaneous-localization-and-mapping) -->

  <!-- - [3 Navigation with Nav2](#3-navigation-with-nav2) -->

<!-- - [Future Work](#future-work)
- [Gallery](#gallery)
- [TF Tree](#tf-tree) -->

## Features

### 1. **Ackermann Steering**

- A custom vehicle model built with realistic Ackermann steering dynamics for accurate maneuverability.

### 2. **ROS2 Communication**

- All sensor data and control signals are fully integrated into the ROS2 ecosystem for seamless interoperability.

### 3. **Sensors**

- **IMU**: Provides orientation and angular velocity.
- **Odometry**: Ensures accurate vehicle state feedback.
- **LiDAR**: Mounted for obstacle detection and environmental scanning. Supports 3D point cloud generation for advanced perception tasks.
- **Cameras**:

  - Front-facing

  <!-- - Rear-facing
  - Left-side
  - Right-side -->

  <!-- > **Note:** By default, only the front camera is bridged to ROS 2.If you want to use all cameras (left, right, rear) in ROS 2,remove the `#` at the beginning of the relevant camera sections in `saye_bringup/config/ros_gz_bridge.yaml` to activate them  (e.g., `/camera/left_raw`, `/camera/right_raw`, `/camera/rear_raw`). -->

<!-- ### 4. **Navigation**

- Integrated with the **Nav2 stack** for autonomous navigation.
- **AMCL (Adaptive Monte Carlo Localization)** for improved positional accuracy.
- **SLAM** techniques implemented for real-time mapping and understanding of the environment.
- Fine-tuned parameters for optimized navigation performance.

### 5. **Manual Control (with external joystick)**

- Added support for joystick-based manual control in the simulation environment, enabling users to test vehicle movement interactively. -->

### 4. **Visualization**

- Full model and sensor data visualization in **RViz2**, providing insights into robot states and environmental feedback.

## Requirements

- **ROS2 (Humble)**
- **Gazebo Sim Ignition**
- **RViz2**
- **ROS2_CONTROL**

## Local Installation

### 0. Make sure you have Gazebo Ignition and ROS–Gazebo integration (`ros_gz`) installed for ROS 2 Humble:

  ```bash
  sudo apt-get install ros-${ROS_DISTRO}-ros-gz
  sudo apt-get install ros-${ROS_DISTRO}-gz-ros2-control
  ```

   More details about installing Gazebo and ROS:  [Link](https://gazebosim.org/docs/latest/ros_installation/)

### 1. Clone the repository:
  ```bash
  mkdir -p gem_sim/src
  cd gem_sim/src
  git clone https://github.com/UIUC-Robotics/gem_simulator.git
  cd ..
  ```
### 2. Build the project:
  ```bash
  colcon build --symlink-install && source install/setup.bash
  ```

## Usage

### 1. Basic Simulation and Manual Control

  ####  Launch the simulation:
     ```bash
     ros2 launch gem_launch gem_init.launch.py
  
     # launch with parameters
     ros2 launch gem_launch gem_init.launch.py x:=0 y:=0 yaw:=0.9 world_name:=silverstone.world
     ```
  ####  Control car:
     ```bash
     # in different terminal
     ros2 topic pub -r 10 /ackermann_cmd ackermann_msgs/msg/AckermannDrive \
     "{steering_angle: 0.35, steering_angle_velocity: 0.5, speed: 1.0, acceleration: 0.5}"
  
     # or for Teleop cmds
     source install/setup.zsh
     ros2 launch gem_launch gem_drive.launch.py
     ```

<!-- ### 2. SLAM (Simultaneous Localization and Mapping)

-   To run SLAM Toolbox for mapping, launch the following after starting the simulation:
    ```bash
    ros2 launch saye_bringup slam.launch.py
    ```
    [![SLAM- Youtube](https://img.youtube.com/vi/QWcJ9TlqFOU/0.jpg)](https://www.youtube.com/watch?v=QWcJ9TlqFOU "Proje Tanıtımı")

### 3. Navigation with Nav2

-   To run the simulation with the Nav2 stack for autonomous navigation, launch the following after starting the simulation:
    ```bash
    ros2 launch saye_bringup navigation_bringup.launch.py
    ```
    [![Autonomus Navigation - Youtube](https://img.youtube.com/vi/SJ4NrbdlNZo/0.jpg)](https://www.youtube.com/watch?v=SJ4NrbdlNZo "NAV2")

> **Note:** The YouTube videos above are played at 4x speed. You can reach the videos by click on the images.

## Future Work

1. **3D SLAM Support:**
   - Train the vehicle to handle complex scenarios autonomously using advanced DRL algorithms.
2. **Enhanced Features:**
   - Explore additional sensor configurations and navigation strategies.
3. **Nav2 entegration with 3D Localization**
   - İnstead of AMCL(2D), More accurate and robust algorithms implementation.
## Gallery

![Screenshot from 2024-09-23 00-09-48.png](https://github.com/user-attachments/assets/dd5604c6-014e-4a7a-9a2f-c4dd237abb37) -->

<!-- ### 3D LiDAR Point Cloud & Environment

| **3D LiDAR Point Cloud Visualization**                             | **Warehouse Environment Model**                                   |
| ----------------------------------------------------------------- | ----------------------------------------------------------------- |
| ![3D Point Cloud](saye_msgs/readme_files/3d_lidar_pointcloud.png) | ![Warehouse Model](saye_msgs/readme_files/warehouse_environment.png) | -->
