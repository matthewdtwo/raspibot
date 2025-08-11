#!/usr/bin/env python3

import sys
import time
sys.path.append('../src')

from robot import Robot

print("Testing robot signal handling...")
print("Robot will move forward, press Ctrl+C to interrupt")

with Robot() as robot:
    try:
        print("Moving forward...")
        robot.move_forward(200, debug=True)
        print("Movement complete")
        
        print("Sleeping for 10 seconds (press Ctrl+C to test interrupt)")
        time.sleep(10)
        
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt in main loop")
    
print("Test complete")
