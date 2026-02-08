# Running Notes & Operational Calls

This document serves as a quick reference for running the various subsystems of the Anzym ROSMaster X3.

## Phase 1: Joystick Teleoperation
**Objective**: Manually control the robot using a gamepad (Xbox/PS4/Logitech).

### 1. Hardware Setup (Confirmed)
*   **Robot Connection**:
    *   **Motor Board**: `/dev/rosmaster` (Aliased via UDEV).
    *   **Lidar**: `/dev/ydlidar` (Aliased via UDEV).
    *   *Note*: If these aliases are missing, run `scripts/setup_udev_rules.sh` on the robot.
*   **Car Type**: Use `car_type:=1` (X3).

### 2. Execution Command (Robot Side)
Open a terminal (SSH) and run:

```bash
# Source the workspace
source ~/anzym_ros_ws/install/setup.bash

# Launch Core Drivers (Automatically uses /dev/rosmaster)
ros2 launch anzym_core bringup_launch.py
```

### 3. Control Command (Laptop/Remote Side)
Connect joystick to workstation:

```bash
# Launch joystick only (disable local driver)
ros2 launch anzym_core joystick_control_launch.py launch_driver:=False
```

### 3. Control Scheme (Mecanum Drive)
The `teleop_twist_joy` is configured for holonomic movement.

| Input | Action |
| :--- | :--- |
| **Enable Button** | *Always On* (or hold L1/LB if configured safely) |
| **Left Stick (Up/Down)** | Move Forward / Backward |
| **Left Stick (Left/Right)** | Strafe Left / Right |
| **Right Stick (Left/Right)** | Turn (Rotate) Left / Right |

### 4. Troubleshooting
**"Package 'joy' not found" or "Launch file not found"**
You may need to install the standard ROS2 teleop packages if they aren't on your Jetson yet:
```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

**Robot doesn't move**
*   Check the terminal for "Serial Port Open Success".
### 4. Lidar Verification
The system now includes the **YDLidar 4ROS** driven by `ydlidar_ros2_driver`.
It launches automatically with `bringup_launch.py`.

**To visualize scanner data:**
1.  On your workstation:
    ```bash
    ros2 run rviz2 rviz2
    ```
2.  **RViz Settings**:
    *   **Fixed Frame**: Set to `laser` (or `base_link`).
    *   **Add**: Select **LaserScan**.
    *   **Topic**: Select `/scan`.
    *   **CRITICAL: QoS Settings**:
        *   Expand "Topic" -> "Reliability Policy".
        *   Set to **Best Effort** (if scan is not showing).

**Troubleshooting Lidar:**
*   Port: Auto-detected as `/dev/ydlidar` (usually mapped to `/dev/ttyUSB0`).
*   Baud: 512000 (auto-negotiated).

### 5. Camera Setup (Integrated Triple Feed)
The camera stack is now fully integrated into `bringup_launch.py`. It uses a split-driver approach to ensure stability and all three feeds (Arm RGB, Astra RGB, Astra Depth) are available simultaneously.

**Topic Namespaces:**
*   `/arm_camera/image_raw`: RGB feed from the camera mounted on the robot arm (`/dev/video0`).
*   `/camera/color/image_raw`: RGB feed from the main Astra camera (`/dev/video2`).
*   `/camera/depth/image_raw`: Depth feed from the main Astra camera.

**How to Visualize:**
In RViz, add an **Image** display for each topic. 
*Note: Due to USB 2.0 bandwidth sharing, framerates will be around 4-10 FPS when all three are active.*

**Manual Restart (if needed):**
If a camera fails to initialize, you can restart just the camera stack:
```bash
ros2 launch anzym_core camera_launch.py
```

## Deployment Guide

### 1. Configuration
Before deploying, edit `scripts/deploy_to_robot.sh` on your workstation:
```bash
ROBOT_USER="jetson"
ROBOT_IP="192.168.1.XXX" # Set your robot's actual IP
ROBOT_WS_PATH="~/anzym_ros_ws"
```

### 2. Deploying Code
Run the script from the root of your workspace to sync code to the robot:
```bash
./scripts/deploy_to_robot.sh
```

### 3. Remote Build & Launch
After syncing, SSH into the robot to build and run:
```bash
ssh jetson@<ROBOT_IP>
cd ~/anzym_ros_ws
colcon build --symlink-install
source install/setup.bash

# To launch just the robot core (Drivers)
ros2 launch anzym_core bringup_launch.py

# To launch with Joystick
ros2 launch anzym_core joystick_control_launch.py
```

### 5. Troubleshooting: "No Movement"
If the robot receives `/joy` (joystick) data but doesn't move:

1.  **Check for Conflicting Drivers**:
    *   Do **NOT** run `joystick_control_launch.py` on the robot if you are *also* running `bringup_launch.py`. This starts two driver nodes that fight for the USB port.
    *   **Correct Remote Control Setup**:
        *   **Robot**: Run `ros2 launch anzym_core bringup_launch.py`
        *   **Workstation (Joystick)**: Run `ros2 launch anzym_core joystick_control_launch.py launch_driver:=False`

2.  **Verify Command Velocity**:
    *   On the robot, run: `ros2 topic echo /cmd_vel`
    *   Move the joystick. You should see linear/angular values changing.
    *   If `/cmd_vel` is silent: The joystick node on your workstation isn't publishing. Check the `enable_button` setting or joystick device (`/dev/input/js0`).

3.  **Check Car Type**:
    *   Default is `car_type:=2` (X3 Plus). If you have the standard X3, launch with `ros2 launch anzym_core bringup_launch.py car_type:=1`.

### 5b. IMU Troubleshooting / Port Issues
The robot driver must connect to `/dev/rosmaster` (aliased to `/dev/ttyUSB2`) to receive IMU and Motor data.
*   **Verification**:
    ```bash
    ros2 topic echo /imu/data --field linear_acceleration --once
    ```
    Should show Z-axis approx `-9.8`. If values are all `0.0`, the driver is likely connected to the wrong port (e.g., Lidar port).

### 6. SLAM Mapping
The robot is configured to use `slam_toolbox` for simultaneous localization and mapping.

**1. Launch Base Robot:**
```bash
ros2 launch anzym_core bringup_launch.py
```

**2. Launch SLAM:**
In a separate terminal on the robot:
```bash
ros2 launch anzym_core slam_launch.py
```

**3. Visualize & Teleop:**
On your workstation:
1.  **RViz**: Add **Map** display listening to `/map` topic.
2.  **Teleop**: Drive the robot around to build the map.
    ```bash
    ros2 launch anzym_core joystick_control_launch.py launch_driver:=False
    ```
