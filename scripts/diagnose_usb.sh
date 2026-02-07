#!/bin/bash
echo "Diagnosing USB Devices..."
for sysdevpath in $(udevadm info --query=path --name=/dev/ttyUSB0) $(udevadm info --query=path --name=/dev/ttyUSB1) $(udevadm info --query=path --name=/dev/ttyUSB2); do
    echo "------------------------------------------------"
    echo "Device Path: $sysdevpath"
    udevadm info -a -p "$sysdevpath" | grep -E "ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}|KERNEL" | head -n 10
done
echo "------------------------------------------------"
ls -l /dev/ttyUSB*
ls -l /dev/rosmaster /dev/ydlidar
