#!/usr/bin/env python3
import time
import os
import sys

# Add src to path just in case we need to import relative
script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(script_dir, "../src/anzym_core/anzym_core"))

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    # Try hardcoded path on robot
    sys.path.append("/home/jetson/anzym_ros_ws/src/anzym_core/anzym_core")
    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError:
        print("Cannot find Rosmaster_Lib.py")
        sys.exit(1)

def test_port(port_name):
    print(f"Testing {port_name}...", end='', flush=True)
    if not os.path.exists(port_name):
        print(" [NOT FOUND]")
        return None
    
    try:
        # Rosmaster class opens port in __init__
        # It defaults to 115200
        bot = Rosmaster(com=port_name, debug=False)
        
        # Give it a moment to initialize serial
        time.sleep(0.1)
        
        # Send Beep to confirm it's the right board (Lidar won't beep)
        # 50ms beep
        bot.set_beep(50)
        
        # Read version or valid packet? 
        # The class runs a receive thread usually, but let's just see if we crash.
        
        print(" [OPENED]")
        return bot
    except Exception as e:
        print(f" [FAILED: {e}]")
        return None

def main():
    print("=== Anzym Port Scanner ===")
    
    candidates = ['/dev/rosmaster'] + [f'/dev/ttyUSB{i}' for i in range(4)]
    
    success_bot = None
    success_port = ""
    
    for port in candidates:
        bot = test_port(port)
        if bot:
            # We found A device. Is it the motor board?
            # Let's try to spin a motor.
            print(f"  Attempts to spin Motor 1 on {port}...")
            bot.set_motor(30, 0, 0, 0)
            time.sleep(1)
            bot.set_motor(0, 0, 0, 0)
            
            answer = input(f"Did the robot Beep/Move on {port}? (y/n): ")
            if answer.lower() == 'y':
                success_bot = bot
                success_port = port
                break
            else:
                # Close/Delete and try next
                del bot

    if success_bot:
        print(f"\nSUCCESS! Robot found on {success_port}")
    else:
        print("\nCould not identify Robot Motor Board.")

if __name__ == '__main__':
    main()
