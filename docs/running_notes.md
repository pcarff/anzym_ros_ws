# Running Notes & Operational Calls

This document serves as a quick reference for running the various subsystems of the Anzym ROSMaster X3.

## Phase 1: Joystick Teleoperation
**Objective**: Manually control the robot using a gamepad (Xbox/PS4/Logitech).

### 1. Hardware Setup
*   **Joystick**: Connect your game controller to the Jetson Orin NX (via USB or Bluetooth).
    *   Verify connection: `ls /dev/input/js*` (usually `js0`).
*   **Robot Connection**: Ensure the Rosmaster X3 control board is connected via USB.
    *   Verify port: `ls /dev/ttyUSB*` (usually `/dev/ttyUSB0`).
    *   *Note: You may need permissions:* `sudo chmod 666 /dev/ttyUSB0`

### 2. Execution Command
Open a terminal and run:

```bash
# Source the workspace
source ~/anzym_ros_ws/install/setup.bash

# Launch the joystick control stack
ros2 launch anzym_core joystick_control_launch.py
```

*Optional Arguments:*
*   Specify a different port: `port:=/dev/ttyUSB1`
*   Specify a different joystick: `joy_dev:=/dev/input/js1`

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
*   If "Permission denied", run the `chmod` command above.
*   If the robot beeps or lights up but doesn't move, ensure the battery is charged and the motor switch is ON.

## Workstation <-> Robot Deployment

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
