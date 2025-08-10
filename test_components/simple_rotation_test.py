#!/usr/bin/env python3
"""Simple rotation test to verify anti-hunting improvements."""

import sys
import time
sys.path.append('../src')

from robot import Robot

def test_simple_rotation():
    robot = Robot()
    
    print("Testing 45-degree CCW rotation...")
    print("=" * 50)
    
    start_time = time.time()
    robot._rotate_ccw(45, debug=True)
    total_time = time.time() - start_time
    
    print("=" * 50)
    print(f"Rotation completed in {total_time:.2f} seconds")
    
    # Check final position
    try:
        final_heading = robot.imu.get_orientation()
        print(f"Final heading: {final_heading * 57.3:.1f}° (target was 45°)")
        print(f"Final error: {45 - final_heading * 57.3:.1f}°")
    except Exception as e:
        print(f"Could not read final heading: {e}")

if __name__ == "__main__":
    test_simple_rotation()
