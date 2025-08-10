#!/usr/bin/env python3
"""Test different anti-oscillation strategies."""

import sys
import time
import math
sys.path.append('../src')

from robot import Robot

def ultra_smooth_rotate(self, deg: float, debug: bool = False):
    """Ultra-smooth rotation with aggressive damping."""
    try:
        self.imu.calibrate_zero(samples=15)
    except Exception:
        return

    target_rad = math.radians(deg)
    
    # Very conservative parameters
    dt = 0.1  # Much slower loop
    tol_deg = 2.0
    deadband_deg = 1.2
    hold_time = 0.5
    
    tol_rad = math.radians(tol_deg)
    deadband_rad = math.radians(deadband_deg)
    
    # Import needed constants
    from config import L_MTR, R_MTR, FWD, RWD
    
    # Very conservative speeds
    min_speed = 88
    max_speed = 105
    
    timeout_s = max(6.0, 0.15 * abs(deg) + 3.0)
    start = time.time()
    
    # State tracking
    prev_error = 0.0
    error_history = []
    in_tolerance_start = None
    loop_count = 0
    consecutive_same_direction = 0
    last_direction = 0
    
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))
    
    def moving_average(values, window=3):
        if len(values) < window:
            return sum(values) / len(values) if values else 0
        return sum(values[-window:]) / window
    
    try:
        while True:
            if time.time() - start > timeout_s:
                if debug:
                    print(f"Timeout after {timeout_s:.1f}s")
                break
                
            # Get smoothed heading
            heading = self.imu.get_orientation(smooth=True)
            err = target_rad - heading
            err = (err + math.pi) % (2 * math.pi) - math.pi
            
            # Track error history for trend analysis
            error_history.append(err)
            if len(error_history) > 8:
                error_history.pop(0)
            
            err_deg = math.degrees(err)
            
            # Check for oscillation pattern
            oscillation_detected = False
            if len(error_history) >= 6:
                # Look for sign changes indicating oscillation
                signs = [1 if e > 0 else -1 for e in error_history[-4:]]
                if len(set(signs)) > 1:  # Mixed signs = oscillation
                    recent_range = max(error_history[-4:]) - min(error_history[-4:])
                    if recent_range > math.radians(3.0):  # Significant oscillation
                        oscillation_detected = True
            
            # Enhanced deadband when oscillating
            effective_deadband = deadband_rad
            if oscillation_detected:
                effective_deadband = deadband_rad * 2.0
                if debug and loop_count % 8 == 0:
                    print(f"Oscillation detected, using larger deadband")
            
            # Check tolerance
            if abs(err) <= tol_rad:
                if in_tolerance_start is None:
                    in_tolerance_start = time.time()
                    if debug:
                        print(f"Entered tolerance: err={err_deg:.2f}°")
                elif time.time() - in_tolerance_start >= hold_time:
                    if debug:
                        print(f"Hold complete")
                    break
            else:
                in_tolerance_start = None
            
            # Apply enhanced deadband
            if abs(err) <= effective_deadband:
                if debug and loop_count % 8 == 0:
                    status = "Enhanced deadband" if oscillation_detected else "Normal deadband"
                    print(f"{status}: err={err_deg:.2f}°")
                time.sleep(dt)
                loop_count += 1
                continue
            
            # Ultra-conservative speed control
            err_deg_abs = abs(err_deg)
            if err_deg_abs < 2.0:
                speed = min_speed
            elif err_deg_abs < 8.0:
                # Very gentle ramp
                ratio = (err_deg_abs - 2.0) / 6.0
                speed = int(min_speed + ratio * (max_speed - min_speed) * 0.5)
            else:
                speed = int(min_speed + (max_speed - min_speed) * 0.8)
            
            # Additional damping if we keep changing direction
            direction = 1 if err > 0 else -1
            if direction == last_direction:
                consecutive_same_direction += 1
            else:
                consecutive_same_direction = 0
                # Penalty for direction changes
                if loop_count > 5:
                    speed = max(min_speed, int(speed * 0.8))
            last_direction = direction
            
            speed = clamp(speed, min_speed, max_speed)
            
            # Debug
            if debug and loop_count % 6 == 0:
                elapsed = time.time() - start
                status = "TOL" if abs(err) <= tol_rad else "TURN"
                osc = "OSC" if oscillation_detected else "---"
                trend = moving_average([math.degrees(e) for e in error_history[-3:]])
                print(f"{elapsed:.2f}s {status} err:{err_deg:+6.2f}° trend:{trend:+5.1f}° spd:{speed:3d} {osc}")
            
            # Motor commands
            if err > 0:
                self.motors.set_motor(L_MTR, FWD, speed)
                self.motors.set_motor(R_MTR, RWD, speed)
            else:
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

def test_ultra_smooth():
    robot = Robot()
    
    # Patch with ultra-smooth rotation
    robot._rotate_by_deg = lambda deg, debug=False: ultra_smooth_rotate(robot, deg, debug)
    
    print("Testing Ultra-Smooth Anti-Oscillation Rotation")
    print("=" * 55)
    
    print("45° CCW with oscillation detection...")
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
        print(f"  Performance: {'EXCELLENT' if abs(error) < 2.0 else 'GOOD' if abs(error) < 4.0 else 'NEEDS TUNING'}")
    except:
        print("Could not read final heading")

if __name__ == "__main__":
    test_ultra_smooth()
