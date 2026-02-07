#!/bin/bash

echo "Setting up Persistent USB Rules for Anzym Robot..."

# Define content for the rules file
# Using KERNELS (Physical Path) to distinguish identical CH340 devices.
# Rosmaster -> ttyUSB2 (Path: 1-2.2.2)
# Lidar -> ttyUSB0 (Path: 1-2.1.2) - Also uses unique Serial 0001 if available
# Unknown (Astra?) -> ttyUSB1 (Path: 1-2.1.1)

RULES_CONTENT='
# Rosmaster (CH340 on Hub Port 1-2.2.2)
KERNEL=="ttyUSB*", KERNELS=="1-2.2.2", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0777", SYMLINK+="rosmaster"

# YDLidar (CP210x on Hub Port 1-2.1.2)
KERNEL=="ttyUSB*", KERNELS=="1-2.1.2", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0777", SYMLINK+="ydlidar"

# Astra Serial (Optional/Unknown - CH340 on Hub Port 1-2.1.1)
KERNEL=="ttyUSB*", KERNELS=="1-2.1.1", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0777", SYMLINK+="astra_serial"
'

RULES_FILE="/etc/udev/rules.d/99-anzym.rules"

echo "Writing rules to $RULES_FILE requires sudo..."
echo "$RULES_CONTENT" | sudo tee $RULES_FILE

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Verifying symlinks..."
ls -l /dev/rosmaster /dev/ydlidar

echo "Done. Please disconnect and reconnect device if symlinks do not appear."
