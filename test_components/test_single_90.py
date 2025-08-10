#!/usr/bin/env python3
"""Quick 90-degree rotation test with adaptive parameters."""

import sys
import time
sys.path.append('../src')

from robot import Robot

def test_single_90():
    robot = Robot()
    
    print("Testing single 90-degree CCW rotation with adaptive parameters...")
    print("=" * 60)
    
    start_time = time.time()
    robot._rotate_ccw(90, debug=True)
    total_time = time.time() - start_time
    
    print("=" * 60)
    print(f"Rotation completed in {total_time:.2f} seconds")
    
    try:
        final_heading = robot.imu.get_orientation()
        final_error = 90 - final_heading * 57.3
        print(f"Final heading: {final_heading * 57.3:.1f}° (target was 90°)")
        print(f"Final error: {final_error:.1f}°")
    except Exception as e:
        print(f"Could not read final heading: {e}")

if __name__ == "__main__":
    test_single_90()
