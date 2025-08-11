#!/usr/bin/env python3
"""Encoder calibration test - measure actual pulses per rotation"""

import sys
import time
sys.path.append("../src")

from encoders import Encoders
from motor_controller import MotorController
from config import L_MTR, R_MTR, FWD, ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN, MIN_SPEED

def test_encoder_calibration():
    """Run motors for a fixed time and measure encoder pulses to calibrate"""
    
    motors = MotorController()
    encoders = Encoders()
    
    print("Encoder Calibration Test")
    print("========================")
    print("This will run the motors at low speed for 5 seconds")
    print("to measure encoder pulses. Make sure wheels can turn freely!")
    input("Press Enter to start...")
    
    # Reset encoders
    encoders.reset_counts()
    
    # Run both motors forward at minimum speed
    test_speed = MIN_SPEED + 20  # Slightly above minimum for more consistent speed
    run_time = 5.0  # seconds
    
    print(f"Running motors at speed {test_speed} for {run_time} seconds...")
    
    try:
        # Start motors
        motors.set_motor(L_MTR, FWD, test_speed)
        motors.set_motor(R_MTR, FWD, test_speed)
        
        start_time = time.time()
        
        # Monitor for the duration
        while time.time() - start_time < run_time:
            elapsed = time.time() - start_time
            raw_left, raw_right = encoders.get_counts()
            left_count = ENCODER_LEFT_SIGN * raw_left
            right_count = ENCODER_RIGHT_SIGN * raw_right
            
            print(f"\rTime: {elapsed:.1f}s, Left: {left_count:5d}, Right: {right_count:5d}", end="")
            time.sleep(0.1)
            
    finally:
        motors.stop_motors()
        print()  # New line after the progress display
    
    # Final measurements
    raw_left_final, raw_right_final = encoders.get_counts()
    left_final = ENCODER_LEFT_SIGN * raw_left_final
    right_final = ENCODER_RIGHT_SIGN * raw_right_final
    
    print(f"\nFinal Results:")
    print(f"Left wheel:  {left_final} pulses in {run_time}s")
    print(f"Right wheel: {right_final} pulses in {run_time}s")
    
    # Estimate pulses per rotation (very rough)
    # Assume robot moves ~20-40cm in 5 seconds at minimum speed
    # With 68mm wheel diameter, that's about 1-2 rotations
    estimated_rotations = 1.5  # This is a rough guess
    
    if left_final > 0 and right_final > 0:
        est_ppr_left = int(left_final / estimated_rotations)
        est_ppr_right = int(right_final / estimated_rotations)
        
        print(f"\nEstimated PPR (assuming ~{estimated_rotations} rotations):")
        print(f"Left wheel:  {est_ppr_left} pulses/rotation")
        print(f"Right wheel: {est_ppr_right} pulses/rotation")
        
        print(f"\nCurrent config values:")
        print(f"PULSES_PER_ROTATION_LEFT = {est_ppr_left}")
        print(f"PULSES_PER_ROTATION_RIGHT = {est_ppr_right}")
        
        # Calculate overshoot factor
        from config import PULSES_PER_ROTATION_LEFT, PULSES_PER_ROTATION_RIGHT
        overshoot_left = PULSES_PER_ROTATION_LEFT / est_ppr_left if est_ppr_left > 0 else 0
        overshoot_right = PULSES_PER_ROTATION_RIGHT / est_ppr_right if est_ppr_right > 0 else 0
        
        print(f"\nOvershoot factors:")
        print(f"Left:  {overshoot_left:.2f}x (current/actual)")
        print(f"Right: {overshoot_right:.2f}x (current/actual)")
    else:
        print("No encoder movement detected - check wiring or motor connections")

if __name__ == "__main__":
    test_encoder_calibration()
