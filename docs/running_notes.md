# Running Notes & Operational Calls

This document serves as a quick reference for running the various subsystems of the Anzym ROSMaster X3.

## Phase 1: Joystick Teleoperation
**Objective**: Manually control the robot using a gamepad (Xbox/PS4/Logitech).

### 1. Hardware Setup (Confirmed)
*   **Robot Connection**: The Motor Board is on **/dev/ttyUSB1** (not USB0).
*   **Hardware Anomaly**: The motor wiring for the Rear Right (M3) and Front Right (M4) is **SWAPPED** compared to standard documentation.
    *   *Correction*: This is handled automatically by the custom mixer in `driver_node.py`. **Do not change the wiring.**
*   **Car Type**: Use `car_type:=1` (X3).

### 2. Execution Command (Robot Side)
Open a terminal (SSH) and run:

```bash
# Source the workspace
source ~/anzym_ros_ws/install/setup.bash

# Launch Core Drivers (Port defaults to /dev/ttyUSB1)
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

### 5. Camera Setup (Hybrid Astra Pro)
Your camera requires a **split driver** approach because the RGB sensor is UVC (Webcam) while Depth is OpenNI.

**1. For Depth/IR (Obstacle Avoidance):**
```bash
ros2 launch astra_camera astra.launch.xml enable_color:=false enable_depth:=true enable_ir:=false
```

**2. For RGB Color (Streaming/ML):**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

*Note: Running both simultaneously requires good USB bandwidth. Ensure you are plugged into USB 3.0 ports.*
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
