#!/usr/bin/env python3
import time
import sys
import os
import signal

# Trap SIGINT to ensure clean exit
def signal_handler(sig, frame):
    print('Exiting...')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Try importing Rosmaster_Lib
try:
    # Try direct import (if running from source or installed package)
    from anzym_core.Rosmaster_Lib import Rosmaster
except ImportError:
    # Try adding source path
    sys.path.append(os.path.join(os.path.dirname(__file__), '../src/anzym_core/anzym_core'))
    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError:
        print("Could not import Rosmaster_Lib.")
        sys.exit(1)

def main():
    # Detect port (default to ttyUSB0 or env)
    port = os.environ.get('ROSMASTER_PORT', '/dev/ttyUSB0')
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Connecting to Rosmaster on {port}...")
    try:
        # Debug=True creates a lot of noise, set to False first to see parsed data clearly
        # Set to True if we see nothing
        bot = Rosmaster(com=port, debug=True) 
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("Starting receive thread...")
    bot.create_receive_threading()
    
    print("Enabling Auto Report...")
    bot.set_auto_report_state(True)
    time.sleep(1)
    
    print("Reading data loop...")
    try:
        for i in range(100):
            # Access private vars using name mangling
            ax = getattr(bot, '_Rosmaster__ax', 0)
            ay = getattr(bot, '_Rosmaster__ay', 0)
            az = getattr(bot, '_Rosmaster__az', 0)
            gx = getattr(bot, '_Rosmaster__gx', 0)
            gy = getattr(bot, '_Rosmaster__gy', 0)
            gz = getattr(bot, '_Rosmaster__gz', 0)
            
            # Odometry
            vx = getattr(bot, '_Rosmaster__vx', 0)
            vy = getattr(bot, '_Rosmaster__vy', 0)
            vz = getattr(bot, '_Rosmaster__vz', 0)
            
            print(f"[{i}] A:({ax:.2f}, {ay:.2f}, {az:.2f}) G:({gx:.2f}, {gy:.2f}, {gz:.2f}) V:({vx:.2f}, {vy:.2f}, {vz:.2f})")
            
            time.sleep(0.2)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Closing...")
        del bot

if __name__ == '__main__':
    main()
