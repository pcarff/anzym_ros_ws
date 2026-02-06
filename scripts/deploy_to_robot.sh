#!/bin/bash

# Configuration
# Replace these with your actual robot details
ROBOT_USER="jetson"
ROBOT_IP="192.168.8.246"
ROBOT_WS_PATH="~/anzym_ros_ws"

# Colors for output
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN}Deploying to Robot ($ROBOT_USER@$ROBOT_IP)...${NC}"
# Enable SSH multiplexing to ask for password only once
SSH_CTL_DIR=~/.ssh/ctl
mkdir -p $SSH_CTL_DIR
SSH_CTL_SOCKET="$SSH_CTL_DIR/%r@%h:%p"
# Create a master connection in the background
ssh -fNM -S $SSH_CTL_SOCKET $ROBOT_USER@$ROBOT_IP
# Ensure we close it on exit
trap "ssh -O exit -S $SSH_CTL_SOCKET $ROBOT_USER@$ROBOT_IP 2>/dev/null" EXIT

# Check if IP is set
if [[ "$ROBOT_IP" == *"XXX"* ]]; then
    echo "Error: Please update the ROBOT_IP in scripts/deploy_to_robot.sh before running."
    exit 1
fi

# Sync src directory
rsync -avz --delete -e "ssh -S $SSH_CTL_SOCKET" \
    --exclude '.git' \
    --exclude '.vscode' \
    --exclude 'build' \
    --exclude 'install' \
    --exclude 'log' \
    --exclude '__pycache__' \
    src/ \
    $ROBOT_USER@$ROBOT_IP:$ROBOT_WS_PATH/src/

# Sync scripts directory
rsync -avz --delete -e "ssh -S $SSH_CTL_SOCKET" \
    scripts/ \
    $ROBOT_USER@$ROBOT_IP:$ROBOT_WS_PATH/scripts/

echo -e "${GREEN}Sync Complete.${NC}"
echo -e "To build on robot, run:"
echo -e "ssh -S $SSH_CTL_SOCKET $ROBOT_USER@$ROBOT_IP 'cd $ROBOT_WS_PATH && colcon build --symlink-install && source install/setup.bash'"
