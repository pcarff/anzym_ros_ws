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
- Begin Phase 2: Sensor Integration (Lidar).
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
