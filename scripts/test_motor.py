#!/usr/bin/env python3
import time
from anzym_core.Rosmaster_Lib import Rosmaster
import sys

# Try CarType 1 (X3) first, then change manually if needed
car_type = 1 
port = '/dev/ttyUSB2'

print(f"Testing Motor on {port} with CarType {car_type}")

try:
    bot = Rosmaster(car_type=car_type, com=port, debug=True)
    
    print("Sending BEEP (If you hear this, connection is GOOD)...")
    bot.set_beep(100)
    time.sleep(1)
    
    print("Sending Motor Command 100 (Max Power)...")
    # Try sending explicit integers
    bot.set_motor(100, 100, 100, 100)
    time.sleep(2)
    
    print("Stopping...")
    bot.set_motor(0, 0, 0, 0)
    time.sleep(1)
    
    del bot
    print("Test Complete.")
    
except Exception as e:
    print(f"Error: {e}")
