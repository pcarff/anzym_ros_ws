#!/usr/bin/env python3
import time
import sys
import os
import serial

def main():
    port = os.environ.get('ROSMASTER_PORT', '/dev/ttyUSB0')
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Opening {port} at 115200...")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    # Send Auto Report Enable Command manually
    # cmd = [0xFF, 0xFC, 0x05, 0x01, 0x01, 0x00, 0x07]
    cmd = bytearray([0xFF, 0xFC, 0x05, 0x01, 0x01, 0x00, 0x07])
    print(f"Sending Enable Auto Report: {list(cmd)}")
    ser.write(cmd)
    
    print("Reading raw data...")
    start_time = time.time()
    while time.time() - start_time < 10:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            hex_data = " ".join([f"{b:02X}" for b in data])
            print(f"Recv ({len(data)}): {hex_data}")
        time.sleep(0.05)
    
    ser.close()

if __name__ == '__main__':
    main()
