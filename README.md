# Anzym ROS2 Workspace

## Project Overview
This repository contains the ROS2 software stack for the **Rosmaster X3 Plus** robot, powered by an **Nvidia Jetson Orin NX (16GB)**.
The project is built step-by-step, prioritizing modularity, clear documentation, and stability.

## Hardware Specifications
- **Robot Platform**: Rosmaster X3 Plus (Mecanum Drive)
- **Compute Unit**: Nvidia Jetson Orin NX (16GB RAM)
- **OS**: Ubuntu 20.04/22.04 (Jetpack)
- **Middleware**: ROS2 Humble Hawksbill (or Foxy/Iron depending on exact Jetpack version)

## Documentation
Documentation is maintained in the `docs/` directory.
- [Project Roadmap](docs/ROADMAP.md)
- [Development Progress](docs/PROGRESS.md)
- [Setup & Guides](docs/guides/)

## Repository Structure
- `src/`: Source code for ROS2 packages.
    - `anzym_core`: Core package for custom configurations and launch files.
- `docs/`: Project documentation and guides.

## Usage
All core hardware components are integrated into a single bringup launch.

**1. On the Robot:**
```bash
source install/setup.bash
ros2 launch anzym_core bringup_launch.py
```
This starts:
- Motor Driver & Mecanum Kinematics
- IMU (Acceleration/Gyro)
- YDLidar (Laser Scan)
- Robot Arm visualization
- Triple Camera Feed (Arm RGB, Astra RGB-D)

**2. On the Workstation:**
- Use RViz to visualize `/scan` and image topics.
- Run `ros2 launch anzym_core joystick_control_launch.py launch_driver:=False` for teleop.
