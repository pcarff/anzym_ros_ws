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

### Session [Date: 2026-02-07]
**Status**: **Phase 2 Complete** (Full Sensor Suite Integration)

**Accomplished**:
- **Camera Integration (Triple Feed)**:
    - Resolved issue where only two of three camera feeds were visible.
    - **Astra RGB Fix**: Identified that the native `astra_camera` driver often fails to publish the RGB stream on Jetson.
    - **Split Driver Architecture**: 
        - Configured `usb_cam` to handle the **Arm Camera** (`/dev/video0`) in `/arm_camera` namespace.
        - Configured `usb_cam` to handle the **Astra RGB** stream (`/dev/video2`) in `/camera/color` namespace using `yuyv` format.
        - Configured `astra_camera` (OpenNI) to handle **Depth only** in `/camera/depth` namespace.
    - **Bandwidth Optimization**: Reduced resolutions/framerates (320x240 for arm, 640x480 for Astra) to stabilize 3 concurrent streams on the USB 2.0 bus.
- **SLAM Implementation**:
    - Created `slam_launch.py` and `mapper_params_online_async.yaml` for `slam_toolbox`.
    - Verified full transform tree connection: `map` -> `odom` -> `base_footprint` -> `base_link` -> `laser_link`.
    - Validated `/map` topic publication on the robot.
- **IMU Fix**:
    - Identified incorrect serial port usage (`/dev/ttyUSB0` instead of `/dev/rosmaster`).
    - Updated `bringup_launch.py` and `driver_node.py` to use `/dev/rosmaster` (aliased to `/dev/ttyUSB2`).
    - Enabled IMU data publishing in `driver_node.py` on topic `/imu/data`.
    - Uncommented `imu_link` in URDF to ensure valid TF tree.
    - Verified valid acceleration data (~9.8 m/s^2 on Z-axis).
- **Sensor Fusion (EKF)**:
    - Integrated `robot_localization` package.
    - Configured `ekf_params.yaml` to fuse:
        - Wheel Odometry (`/odom`): Velocity (vx, vy, vyaw).
        - IMU (`/imu/data`): Gyro (vyaw) and Accel (ax).
    - Updated `bringup_launch.py` to:
        - Conditionally launch `ekf_node` (`use_ekf:=True`, default).
        - Disable `driver_node` internal TF publishing when EKF is active.
        - Add `use_cameras` argument to optionally disable cameras for resource saving.
    - Verified correct TF generation: `odom` -> `base_footprint` published by EKF.
- **Workflow & Access**:
    - Established SSH keys and `sshpass` workflow for automated deployment and remote debugging on `jetson@192.168.8.246`.
    - Integrated all sensors (Lidar, 2x RGB, 1x Depth) into `bringup_launch.py`.

**Next Steps (Phase 3)**:
- **URDF Calibration**: Fine-tune joint offsets and camera extrinsics in the Xacro.
- **Visual SLAM**: Test `rtabmap_ros` or `vins-fusion` using the synchronized Astra RGB-D pair.
- **Behavior**: Implement a "Follow Cat" node using the stabilized camera feeds.
