#!/bin/bash
# Force NTP synchronization on the robot

TARGET=${1:-jetson@192.168.8.246}

echo "=== Forcing NTP Time Sync on Robot ==="
echo "This will enable NTP and force an immediate sync to internet time servers."

# Enable NTP and force immediate sync
ssh $TARGET "sudo timedatectl set-ntp true && sudo systemctl restart systemd-timesyncd && sleep 2 && timedatectl status"

echo ""
echo "=== Checking Time Difference ==="
ROBOT_EPOCH=$(ssh $TARGET "date +%s")
LOCAL_EPOCH=$(date +%s)
DIFF=$((LOCAL_EPOCH - ROBOT_EPOCH))

echo "Local Time (epoch): $LOCAL_EPOCH"
echo "Robot Time (epoch): $ROBOT_EPOCH"
echo "Difference: ${DIFF}s"

if [ ${DIFF#-} -gt 1 ]; then
    echo "[WARN] Time difference is greater than 1 second!"
    echo "Attempting direct sync from local machine..."
    ssh $TARGET "sudo timedatectl set-ntp false && sudo date -s @$LOCAL_EPOCH && sudo hwclock --systohc"
    echo "Done. New robot time:"
    ssh $TARGET "date"
else
    echo "[OK] Clocks are in sync."
fi
