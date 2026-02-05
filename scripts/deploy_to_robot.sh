#!/bin/bash

# Configuration
# Replace these with your actual robot details
ROBOT_USER="jetson"
ROBOT_IP="192.168.1.XXX" # PLACEHOLDER
ROBOT_WS_PATH="~/anzym_ros_ws"

# Colors for output
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN}Deploying to Robot ($ROBOT_USER@$ROBOT_IP)...${NC}"

# Check if IP is set
if [[ "$ROBOT_IP" == *"XXX"* ]]; then
    echo "Error: Please update the ROBOT_IP in scripts/deploy_to_robot.sh before running."
    exit 1
fi

# Sync src directory
# --delete: removes files on robot that were deleted locally
# --exclude: skip git, build artifacts, etc.
rsync -avz --delete \
    --exclude '.git' \
    --exclude '.vscode' \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    --exclude '__pycache__' \
    src/ \
    $ROBOT_USER@$ROBOT_IP:$ROBOT_WS_PATH/src/

echo -e "${GREEN}Sync Complete.${NC}"
echo -e "To build on robot, run:"
echo -e "ssh $ROBOT_USER@$ROBOT_IP 'cd $ROBOT_WS_PATH && colcon build --symlink-install && source install/setup.bash'"
