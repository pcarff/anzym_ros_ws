# Development Progress Log

## Current Sprint: Phase 1 & 2 - Setup and Core Teleop

### Session [Date: 2026-02-05]
**Status**: Critical Hardware Integration COMPLETE

**Accomplished**:
- **Hardware Integration**:
    - Identified motor controller on `/dev/ttyUSB1` (fixed incorrect ttyUSB0 default).
    - Downgraded `Rosmaster_Lib.py` to V3.3.1 (Official) to match firmware protocol.
    - Set default `car_type=1` (X3).
- **Physics Calibration**:
    - Diagnosed incorrect motor wiring (Right Side M3/M4 swapped).
    - Implemented custom Kinematic Mixer in `driver_node.py` to support correct Mecanum strafing without physical rewiring.
- **Controls**:
    - Tuned rotation sensitivity (0.5 scale).
    - Mapped Joystick: Left Stick (Fwd/Back), Right Stick (Turn), Left Stick (Strafe).

**Next Steps**:
- Begin Phase 3: TF/URDF and SLAM.
    - Create `chassis_description` package.
    - Calibrate Lidar position.
    - Run `slam_toolbox`.

### Session [Date: 2026-02-06] (Part 3)
**Status**: **Phase 1 Complete** (Teleop Verified) + **Phase 2 Complete** (Drivers Installed)

**Accomplished**:
- **Teleop Fixed**:
    - Identified incorrect USB port mapping (Motor board was on `/dev/ttyUSB2`, while `/dev/ttyUSB1` was deceptive).
    - Created **UDEV Rules** (`scripts/setup_udev_rules.sh`) to permanently alias:
        - `/dev/rosmaster` -> Motor Board (CH340 at Hub Port 2)
        - `/dev/ydlidar` -> Lidar (CP210x at Hub Port 1)
    - Reverted `driver_node.py` to use standard `set_car_motion` kinematics (smoother PID).
    - Validated smooth movement in all directions.
- **Drivers**:
    - `ydlidar_ros2_driver` installed (needs tuning).
    - `ros2_astra_camera` installed.

**Next Steps (Phase 3)**:
- **Lidar Tuning**: Fix `Failed to get scan` error (likely sample rate or power issue).
- **URDF**: Build TF tree (`base_link` -> `laser` -> `camera`).
- **SLAM**: Generate map with `slam_toolbox`.
    - Configured `teleop_twist_joy` for Holonomic (Mecanum) drive.
    - Mapped "Enable" button and axes for Forward/Strafe/Turn.
- **Launch System**:
    - Created `bringup_launch.py` as the main robot entry point.
    - Created `joystick_control_launch.py` for manual control.
- **Deployment Tools**:
    - Created `scripts/deploy_to_robot.sh` for one-click code syncing to the Jetson Orin.
- **Documentation**:
    - Created `docs/running_notes.md` with operational guides.
    - Maintained `ROADMAP.md` and `setup_and_workflow.md`.

**Next Actions**:
- **Validate on Hardware**: User to deploy and test joystick control.
- **Phase 2: Sensors**:
    - Identify and configure the Lidar driver (likely `sllidar` or `rplidar`).
    - Integrate the Orin-native camera or USB camera.
