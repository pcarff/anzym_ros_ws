#!/usr/bin/env python3
import time
import sys
import os

# quick hack to import the lib from the src folder
sys.path.append(os.path.join(os.path.dirname(__file__), '../src/anzym_core/anzym_core'))
from Rosmaster_Lib import Rosmaster

def test_robot():
    print("=== Anzym Robot Diagnostic v3 (Scanner) ===")
    
def test_robot():
    print("=== Anzym Motor Mapper ===")
    
    port = '/dev/ttyUSB1' # We know this is the one
    bot = Rosmaster(car_type=1, com=port, debug=True)
    bot.create_receive_threading()
    time.sleep(1)
    
    bot.set_beep(50)
    
    # Stop all first
    bot.set_motor(0, 0, 0, 0)
    time.sleep(1)
    
    motors = ["M1", "M2", "M3", "M4"]
    
    # Test M1
    print("\n>>> SPINNING M1 (Speed 50) <<<")
    bot.set_motor(50, 0, 0, 0)
    time.sleep(2)
    bot.set_motor(0, 0, 0, 0)
    
    # Test M2
    print("\n>>> SPINNING M2 (Speed 50) <<<")
    bot.set_motor(0, 50, 0, 0)
    time.sleep(2)
    bot.set_motor(0, 0, 0, 0)

    # Test M3
    print("\n>>> SPINNING M3 (Speed 50) <<<")
    bot.set_motor(0, 0, 50, 0)
    time.sleep(2)
    bot.set_motor(0, 0, 0, 0)

    # Test M4
    print("\n>>> SPINNING M4 (Speed 50) <<<")
    bot.set_motor(0, 0, 0, 50)
    time.sleep(2)
    bot.set_motor(0, 0, 0, 0)
    
    print("\nDONE. Please report which wheel moved for M1, M2, M3, M4.")
    print("Also report direction (Fwd/Back) if possible.")

if __name__ == "__main__":
    test_robot()
