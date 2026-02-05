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
