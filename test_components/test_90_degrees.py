#!/usr/bin/env python3
"""Test 90-degree rotations to verify performance on larger angles."""

import sys
import time
sys.path.append('../src')

from robot import Robot

def test_90_degree_rotations():
    robot = Robot()
    
    # Test CCW 90 degrees
    print("Testing 90-degree CCW rotation...")
    print("=" * 60)
    
    start_time = time.time()
    robot._rotate_ccw(90, debug=True)
    total_time = time.time() - start_time
    
    print("=" * 60)
    print(f"CCW rotation completed in {total_time:.2f} seconds")
    
    try:
        final_heading = robot.imu.get_orientation()
        final_error = 90 - final_heading * 57.3
        print(f"Final heading: {final_heading * 57.3:.1f}° (target was 90°)")
        print(f"Final error: {final_error:.1f}°")
    except Exception as e:
        print(f"Could not read final heading: {e}")
    
    # Wait a moment between tests
    print("\nWaiting 3 seconds before next test...")
    time.sleep(3)
    
    # Test CW 90 degrees
    print("\nTesting 90-degree CW rotation...")
    print("=" * 60)
    
    start_time = time.time()
    robot._rotate_cw(90, debug=True)
    total_time = time.time() - start_time
    
    print("=" * 60)
    print(f"CW rotation completed in {total_time:.2f} seconds")
    
    try:
        final_heading = robot.imu.get_orientation()
        # For CW rotation, we expect negative heading change
        expected_heading = 90 - 90  # Should be near 0° after CW from 90°
        final_error = expected_heading - final_heading * 57.3
        print(f"Final heading: {final_heading * 57.3:.1f}° (target was ~0°)")
        print(f"Final error: {final_error:.1f}°")
    except Exception as e:
        print(f"Could not read final heading: {e}")

if __name__ == "__main__":
    test_90_degree_rotations()
