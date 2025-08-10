#!/usr/bin/env python3
"""Debug rotation hunting behavior with detailed output."""

import sys
import time
sys.path.append('../src')

from robot import Robot

def test_rotation_with_debug():
    robot = Robot()
    
    # Test small rotation that might show hunting
    print("Testing 30-degree CCW rotation with debug output...")
    print("=" * 60)
    
    start_time = time.time()
    robot._rotate_ccw(30, debug=True)
    total_time = time.time() - start_time
    
    print("=" * 60)
    print(f"Total rotation time: {total_time:.2f} seconds")
    
    # Test a smaller rotation that's more likely to hunt
    print("\nTesting 15-degree CW rotation...")
    print("=" * 60)
    
    start_time = time.time()
    robot._rotate_cw(15, debug=True)
    total_time = time.time() - start_time
    
    print("=" * 60)
    print(f"Total rotation time: {total_time:.2f} seconds")

if __name__ == "__main__":
    test_rotation_with_debug()
