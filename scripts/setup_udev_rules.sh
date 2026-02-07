#!/bin/bash
echo "Setting up UDEV rules for Anzym Robot..."

# Define the rules content
RULES_CONTENT='
# Lidar (CP210x) - Based on Silicon Labs Vendor ID
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="ydlidar"

# Motor Board (CH340) - Corrected Path (1-2.2.2)
# Found via manual diagnosis: ttyUSB2 was the working motor port.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-2.2.2", MODE="0666", SYMLINK+="rosmaster"

# Voice/Other Module (CH340) - Path (1-2.1.1)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="1-2.1.1", MODE="0666", SYMLINK+="voice_module"

# Alternative Motor Board Rule (if not on 1-2.1.1, try generic CH340 matching if unique)
# Currently commented out to avoid conflict with other CH340 devices.
# SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="rosmaster_candidate"
'

# Write to file
echo "$RULES_CONTENT" | sudo tee /etc/udev/rules.d/99-anzym.rules

# Reload rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done! Check devices with: ls -l /dev/rosmaster /dev/ydlidar"
