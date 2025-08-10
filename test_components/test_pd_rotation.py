#!/usr/bin/env python3
"""Enhanced rotation function with PD control to reduce oscillations."""

import sys
import time
import math
sys.path.append('../src')

from robot import Robot

# Temporarily patch the robot class with improved rotation
def enhanced_rotate_by_deg(self, deg: float, debug: bool = False):
    """Rotate in place by a signed angle in degrees using enhanced PD control.

    Positive deg => CCW, Negative => CW.
    """
    # Zero heading to current orientation for a relative rotation
    try:
        self.imu.calibrate_zero(samples=15)
    except Exception:
        return

    target_rad = math.radians(deg)

    # Enhanced control parameters
    dt = 0.08  # Slower loop for stability
    tol_deg = 1.5  # Tighter tolerance
    deadband_deg = 0.8  # Smaller deadband
    hold_time = 0.4  # Longer hold time
    
    tol_rad = math.radians(tol_deg)
    deadband_rad = math.radians(deadband_deg)
    
    # PD Control parameters
    Kp = 0.8  # Proportional gain
    Kd = 0.3  # Derivative gain (damping)
    
    # Speed limits
    from config import TURN_MIN_SPEED, TURN_MAX_SPEED, MAX_SPEED, L_MTR, R_MTR, FWD, RWD
    min_speed = max(TURN_MIN_SPEED, 85)  # Slightly higher minimum
    max_speed = min(TURN_MAX_SPEED, 120)  # Lower maximum for control
    
    timeout_s = max(4.0, 0.08 * abs(deg) + 2.0)
    start = time.time()
    
    # Control state
    prev_error = 0.0
    in_tolerance_start = None
    loop_count = 0
    
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))
    
    try:
        while True:
            if time.time() - start > timeout_s:
                if debug:
                    print(f"Timeout after {timeout_s:.1f}s")
                break
                
            # Get current heading
            heading = self.imu.get_orientation()
            err = target_rad - heading
            err = (err + math.pi) % (2 * math.pi) - math.pi  # Wrap to [-pi, pi]
            
            # Calculate derivative (rate of error change)
            if loop_count > 0:
                derivative = (err - prev_error) / dt
            else:
                derivative = 0.0
            prev_error = err
            
            err_deg = math.degrees(err)
            
            # Check if we're in tolerance
            if abs(err) <= tol_rad:
                if in_tolerance_start is None:
                    in_tolerance_start = time.time()
                    if debug:
                        print(f"Entered tolerance: err={err_deg:.2f}°")
                elif time.time() - in_tolerance_start >= hold_time:
                    if debug:
                        print(f"Hold complete, stopping")
                    break
            else:
                in_tolerance_start = None
            
            # Apply deadband to prevent micro-movements
            if abs(err) <= deadband_rad:
                if debug and loop_count % 8 == 0:
                    print(f"In deadband: err={err_deg:.2f}°")
                time.sleep(dt)
                loop_count += 1
                continue
            
            # PD Control calculation
            proportional = Kp * err_deg
            derivative_term = Kd * math.degrees(derivative)
            
            # Combine P and D terms
            control_output = proportional + derivative_term
            
            # Convert to speed command
            speed_cmd = abs(control_output)
            
            # Apply speed limits and scaling
            if speed_cmd < 3.0:
                speed = min_speed
            elif speed_cmd > 30.0:
                speed = max_speed
            else:
                # Linear interpolation between min and max
                ratio = (speed_cmd - 3.0) / (30.0 - 3.0)
                speed = int(min_speed + ratio * (max_speed - min_speed))
            
            speed = clamp(speed, min_speed, max_speed)
            
            # Debug output
            if debug and loop_count % 4 == 0:
                elapsed = time.time() - start
                status = "TOL" if abs(err) <= tol_rad else "TURN"
                hold_info = f"hold:{time.time() - in_tolerance_start:.2f}s" if in_tolerance_start else "---"
                print(f"{elapsed:.2f}s {status} err:{err_deg:+6.2f}° d/dt:{math.degrees(derivative):+5.1f}°/s PD:{control_output:+5.1f} spd:{speed:3d} {hold_info}")
            
            # Determine motor directions and apply commands
            if err > 0:  # Need CCW rotation
                self.motors.set_motor(L_MTR, FWD, speed)
                self.motors.set_motor(R_MTR, RWD, speed)
            else:  # Need CW rotation
                self.motors.set_motor(L_MTR, RWD, speed)
                self.motors.set_motor(R_MTR, FWD, speed)
            
            time.sleep(dt)
            loop_count += 1
            
    finally:
        self.motors.stop_motors()
        if debug:
            final_heading = self.imu.get_orientation()
            final_err = math.degrees(target_rad - final_heading)
            print(f"Final error: {final_err:+.2f}°, loops: {loop_count}")

# Test the enhanced rotation
def test_enhanced_rotation():
    robot = Robot()
    
    # Patch the robot with enhanced rotation
    robot._rotate_by_deg = lambda deg, debug=False: enhanced_rotate_by_deg(robot, deg, debug)
    
    print("Testing Enhanced PD Rotation Control")
    print("=" * 50)
    
    # Test 45-degree rotation
    print("Testing 45° CCW rotation with PD control...")
    start_time = time.time()
    robot._rotate_ccw(45, debug=True)
    elapsed = time.time() - start_time
    
    try:
        final_heading = robot.imu.get_orientation()
        error = 45 - math.degrees(final_heading)
        print(f"\nResults:")
        print(f"  Time: {elapsed:.2f}s")
        print(f"  Final heading: {math.degrees(final_heading):.1f}°")
        print(f"  Error: {error:.2f}°")
    except:
        print("Could not read final heading")

if __name__ == "__main__":
    test_enhanced_rotation()
