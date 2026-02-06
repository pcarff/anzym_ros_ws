# Anzym ROS2 Project Roadmap

This roadmap outlines the step-by-step development of the Anzym ROS2 stack.

## Phase 1: Basic Control & Hardware Abstraction
**Goal**: Get the robot moving via joystick and establish base hardware communication.
- [x] **Step 1.1**: Workspace Setup & Repo Initialization
- [x] **Step 1.2**: Mecanum Wheel Teleoperation (Joystick Control)
    - [x] Implement/Port low-level motor driver (Serial communication).
    - [x] Configure `teleop_twist_joy` for input.
    - [x] Implement kinematics for Mecanum drive (geometry_msgs/Twist -> Motor speeds).

## Phase 2: Sensor Integration
**Goal**: Visualize the robot's environment.
- [x] **Step 2.1**: Lidar Setup
    - [x] Bring up the lidar driver (YDLidar).
    - [x] Visualize data in RViz (on remote PC).
- [ ] **Step 2.2**: Camera Integration
    - Configure Jetson Orin camera/USB camera.
    - specialized node for camera stream.

## Phase 3: Odometry & State Estimation
**Goal**: The robot knows roughly where it is relative to start.
- [ ] **Step 3.1**: Wheel Odometry
    - Calculate position based on encoder feedback.
- [ ] **Step 3.2**: IMU Fusion (Robot_Localization)
    - Fuse IMU and Encoder data using EKF for stable odometry.

## Phase 4: Mapping (SLAM)
**Goal**: Create a map of the room.
- [ ] **Step 4.1**: SLAM Implementation
    - Use `slam_toolbox` or `cartographer`.
    - Generate occupancy grid maps.

## Phase 5: Autonomous Navigation
**Goal**: Move from Point A to Point B automatically.
- [ ] **Step 5.1**: Nav2 Setup
    - Configure costmaps (global/local).
    - Configure planner and controller servers.
