# Workspace Setup Guide

## 1. Prerequisites
- **OS**: Linux (Ubuntu 22.04 recommended for ROS2 Humble)
- **ROS2**: Humble Hawksbill installed.
- **Git**: Version control.
- **Tools**: `colcon`, `rosdep`.

## 2. Initialization Process
We created the workspace using the following standard ROS2 procedure:

```bash
# Create directory
mkdir -p ~/anzym_ros_ws/src
cd ~/anzym_ros_ws

# Initialize Git
git init
# (Created .gitignore)

# Create Source Package
cd src
ros2 pkg create --build-type ament_python anzym_core

# Build
cd ..
colcon build
source install/setup.bash
```

## 3. GitHub Integration
The repository is hosted at `https://github.com/pcarff/anzym_ros_ws`.
To push changes:
1. Ensure documentation in `docs/` is updated.
2. `git add .`
3. `git commit -m "Your message"`
4. `git push`

## 4. Environment
Remember to always source the setup file in every new terminal:
```bash
source ~/anzym_ros_ws/install/setup.bash
```
(Formatting of the path may vary based on where you cloned the repo).
