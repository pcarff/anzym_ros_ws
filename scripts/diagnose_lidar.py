
import serial
import time
import os

def check_lidar(port):
    print(f"Checking {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        # Try to reset: A5 40
        ser.write(b'\xA5\x40')
        time.sleep(0.1)
        resp = ser.read(10)
        print(f"  Response to Reset: {resp.hex()}")
        
        # Try to get info: A5 50
        ser.write(b'\xA5\x50')
        time.sleep(0.1)
        resp = ser.read(27)
        print(f"  Response to Info: {resp.hex()}")
        
        if len(resp) > 5 and resp[0] == 0xA5:
            print(f"  SUCCESS! Lidar found on {port}")
            return True
        ser.close()
    except Exception as e:
        print(f"  Failed: {e}")
    return False

def main():
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB2']
    found = False
    for p in ports:
        if os.path.exists(p):
            if check_lidar(p):
                found = True
                break
    
    if not found:
        print("No Lidar found on checked ports.")

if __name__ == '__main__':
    main()
