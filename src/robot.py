import time
import math
import signal
import atexit

from motor_controller import MotorController
from encoders import Encoders
from imu import IMU
from camera import Camera

from langchain_core.tools import tool

from config import (
    WHEEL_DIAMETER,
    MIN_SPEED,
    MAX_SPEED,
    TURN_MIN_SPEED,
    TURN_MAX_SPEED,
    L_MTR,
    R_MTR,
    FWD,
    RWD,
    PULSES_PER_ROTATION_LEFT,
    PULSES_PER_ROTATION_RIGHT,
    ENCODER_LEFT_SIGN,
    ENCODER_RIGHT_SIGN,
)


class Robot:
    def __init__(self, web_interface=None):
        self.motors = MotorController()
        self.encoders = Encoders()
        self.imu = IMU()
        self.web_interface = web_interface
        
        # Register cleanup handlers for safe shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        atexit.register(self.cleanup)

    def _signal_handler(self, signum, frame):
        """Handle interrupt signals by stopping motors and exiting"""
        print(f"\nReceived signal {signum}, stopping motors...")
        if self.web_interface:
            self.web_interface.update_status(active=False, current_action="Emergency stop")
        self.cleanup()
        exit(0)

    def cleanup(self):
        """Safely stop all motors"""
        try:
            self.motors.stop_motors()
            if self.web_interface:
                self.web_interface.update_status(active=False, current_action="Stopped")
            print("Motors stopped safely")
        except Exception as e:
            print(f"Error stopping motors: {e}")

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup"""
        self.cleanup()

    @tool
    def move_forward(self, mm: int, debug: bool = False):
        """Drive forward a distance in millimeters using a simple PID on encoder counts.
        Does not account for slippage if you get stuck on something, so move in small increments.
        Controls overall progress using average mm while keeping wheels aligned.
        Uses per-wheel encoder calibration and normalized signs.
        """
        if self.web_interface:
            self.web_interface.update_status(current_action=f"Moving forward {mm}mm")
            self.web_interface.log_debug(f"Starting forward movement: {mm}mm")
            
        # Conversion from mm to encoder pulses for each wheel
        wheel_circumference_mm = math.pi * WHEEL_DIAMETER
        pulses_per_mm_L = PULSES_PER_ROTATION_LEFT / wheel_circumference_mm
        pulses_per_mm_R = PULSES_PER_ROTATION_RIGHT / wheel_circumference_mm
        target_pulses_L = int(mm * pulses_per_mm_L)
        target_pulses_R = int(mm * pulses_per_mm_R)

        if debug:
            print(f"Move forward {mm}mm:")
            print(f"  Wheel circumference: {wheel_circumference_mm:.1f}mm")
            print(f"  Pulses/mm - Left: {pulses_per_mm_L:.2f}, Right: {pulses_per_mm_R:.2f}")
            print(f"  Target pulses - Left: {target_pulses_L}, Right: {target_pulses_R}")

        # PID gains (tune on hardware)
        Kp = 0.25
        Ki = 0.0
        Kd = 0.02

        # Steering gain
        K_steer = 0.035
        # Loop/tolerance
        dt = 0.05
        tolerance_mm = 3.0
        tolerance_pulses_L = max(2, int(tolerance_mm * pulses_per_mm_L))
        tolerance_pulses_R = max(2, int(tolerance_mm * pulses_per_mm_R))
        integral = 0.0
        prev_error_mm = mm
        integral_limit = 4 * MAX_SPEED

        # Timeout based on distance
        est_mm_per_s = 80.0
        timeout_s = max(2.0, mm / est_mm_per_s + 1.0)
        start_time = time.time()

        # Reset encoders
        self.encoders.reset_counts()

        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        try:
            loop_count = 0
            while True:
                loop_count += 1
                if time.time() - start_time > timeout_s:
                    if debug:
                        print("  Timeout reached!")
                    break

                # Read encoders and normalize sign so forward is positive
                raw_left_count, raw_right_count = self.encoders.get_counts()
                left_count = ENCODER_LEFT_SIGN * raw_left_count
                right_count = ENCODER_RIGHT_SIGN * raw_right_count

                # Per-wheel pulse errors
                err_L_pulses = target_pulses_L - left_count
                err_R_pulses = target_pulses_R - right_count

                # Overall error in mm (average of both wheels)
                avg_mm_done = 0.5 * (left_count / pulses_per_mm_L + right_count / pulses_per_mm_R)
                error_mm = mm - avg_mm_done

                if debug and loop_count % 10 == 0:  # Print every 0.5s (every 10th loop)
                    print(f"  Progress: {avg_mm_done:.1f}mm/{mm}mm, Left: {left_count}, Right: {right_count}, Error: {error_mm:.1f}mm")

                # Completion when both wheels within their tolerances
                if abs(err_L_pulses) <= tolerance_pulses_L and abs(err_R_pulses) <= tolerance_pulses_R:
                    if debug:
                        print(f"  Target reached! Final position: {avg_mm_done:.1f}mm")
                    break

                # Early exit if we've overshot significantly
                if avg_mm_done > mm + 20:  # 20mm overshoot tolerance
                    if debug:
                        print(f"  Stopping due to overshoot: {avg_mm_done:.1f}mm > {mm + 20}mm")
                    break

                # PID core
                integral += error_mm * dt
                integral = clamp(integral, -integral_limit, integral_limit)
                derivative = (error_mm - prev_error_mm) / dt
                prev_error_mm = error_mm
                pid_output = Kp * error_mm + Ki * integral + Kd * derivative

                # Base speed (forward only) - reduce as we approach target
                speed_cmd = int(abs(pid_output))
                
                # Slow down when close to target
                if error_mm < 20:  # Within 20mm of target
                    speed_cmd = int(speed_cmd * max(0.3, error_mm / 20.0))
                
                if speed_cmd < MIN_SPEED and error_mm > 5:  # Only apply minimum if we're still far from target
                    speed_cmd = MIN_SPEED
                speed_cmd = clamp(speed_cmd, 0, MAX_SPEED)

                # Steering correction in mm space
                left_mm = left_count / pulses_per_mm_L
                right_mm = right_count / pulses_per_mm_R
                diff_mm = left_mm - right_mm
                steer = int(K_steer * diff_mm)

                left_speed = clamp(speed_cmd - steer, 0, MAX_SPEED)
                right_speed = clamp(speed_cmd + steer, 0, MAX_SPEED)

                # Stop motors if we're very close to target to prevent overshoot
                if error_mm < 2:
                    left_speed = right_speed = 0

                self.motors.set_motor(L_MTR, FWD, int(left_speed))
                self.motors.set_motor(R_MTR, FWD, int(right_speed))

                time.sleep(dt)
        finally:
            self.motors.stop_motors()
            if self.web_interface:
                self.web_interface.update_status(current_action="Idle")
                self.web_interface.log_debug(f"Completed forward movement")
            
            # Calculate final position for return value
            raw_left_final, raw_right_final = self.encoders.get_counts()
            left_final = ENCODER_LEFT_SIGN * raw_left_final
            right_final = ENCODER_RIGHT_SIGN * raw_right_final
            final_mm = 0.5 * (left_final / pulses_per_mm_L + right_final / pulses_per_mm_R)
            error_mm = final_mm - mm
            
            # Final debug output
            if debug:
                print(f"  Final: {final_mm:.1f}mm (requested {mm}mm, error: {error_mm:+.1f}mm)")
                print(f"  Final pulses - Left: {left_final}, Right: {right_final}")
                
                # Calculate actual pulses per mm based on this run
                if final_mm > 0:
                    actual_ppm_L = left_final / final_mm
                    actual_ppm_R = right_final / final_mm
                    print(f"  Measured pulses/mm - Left: {actual_ppm_L:.2f}, Right: {actual_ppm_R:.2f}")
                    
                    # Suggested corrections
                    suggested_ppr_L = int(actual_ppm_L * wheel_circumference_mm)
                    suggested_ppr_R = int(actual_ppm_R * wheel_circumference_mm)
                    print(f"  Suggested PPR - Left: {suggested_ppr_L}, Right: {suggested_ppr_R}")

            # Return movement result
            result = f"Forward movement completed. Requested: {mm}mm, Actual: {final_mm:.1f}mm, Error: {error_mm:+.1f}mm"
            if self.web_interface:
                self.web_interface.log_debug(result)
            return result


    @tool
    def move_backward(self, mm: int, debug: bool = False):
        """Drive backward a distance in millimeters using a simple PID on encoder counts.
        Use sparingly, since you can't see behind you. Only move in small increments. Does not account for wheel slippage if you get stuck.

        Controls overall progress using average mm while keeping wheels aligned.
        Uses per-wheel encoder calibration and normalized signs.
        """
        if self.web_interface:
            self.web_interface.update_status(current_action=f"Moving backward {mm}mm")
            self.web_interface.log_debug(f"Starting backward movement: {mm}mm")
            
        # Conversion from mm to encoder pulses for each wheel
        wheel_circumference_mm = math.pi * WHEEL_DIAMETER
        pulses_per_mm_L = PULSES_PER_ROTATION_LEFT / wheel_circumference_mm
        pulses_per_mm_R = PULSES_PER_ROTATION_RIGHT / wheel_circumference_mm
        target_pulses_L = -int(mm * pulses_per_mm_L)  # Negative for backward
        target_pulses_R = -int(mm * pulses_per_mm_R)  # Negative for backward

        if debug:
            print(f"Move backward {mm}mm:")
            print(f"  Wheel circumference: {wheel_circumference_mm:.1f}mm")
            print(f"  Pulses/mm - Left: {pulses_per_mm_L:.2f}, Right: {pulses_per_mm_R:.2f}")
            print(f"  Target pulses - Left: {target_pulses_L}, Right: {target_pulses_R}")

        # PID gains (tune on hardware)
        Kp = 0.25
        Ki = 0.0
        Kd = 0.02

        # Steering gain
        K_steer = 0.035
        # Loop/tolerance
        dt = 0.05
        tolerance_mm = 3.0
        tolerance_pulses_L = max(2, int(tolerance_mm * pulses_per_mm_L))
        tolerance_pulses_R = max(2, int(tolerance_mm * pulses_per_mm_R))
        integral = 0.0
        prev_error_mm = mm
        integral_limit = 4 * MAX_SPEED

        # Timeout based on distance
        est_mm_per_s = 80.0
        timeout_s = max(2.0, mm / est_mm_per_s + 1.0)
        start_time = time.time()

        # Reset encoders
        self.encoders.reset_counts()

        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        try:
            loop_count = 0
            while True:
                loop_count += 1
                if time.time() - start_time > timeout_s:
                    if debug:
                        print("  Timeout reached!")
                    break

                # Read encoders and normalize sign so forward is positive, backward is negative
                raw_left_count, raw_right_count = self.encoders.get_counts()
                left_count = ENCODER_LEFT_SIGN * raw_left_count
                right_count = ENCODER_RIGHT_SIGN * raw_right_count

                # Per-wheel pulse errors (target is negative, current should be negative)
                err_L_pulses = target_pulses_L - left_count
                err_R_pulses = target_pulses_R - right_count

                # Overall error in mm (average of both wheels, use absolute value since we're going backward)
                avg_mm_done = 0.5 * (abs(left_count) / pulses_per_mm_L + abs(right_count) / pulses_per_mm_R)
                error_mm = mm - avg_mm_done

                if debug and loop_count % 10 == 0:  # Print every 0.5s (every 10th loop)
                    print(f"  Progress: {avg_mm_done:.1f}mm/{mm}mm, Left: {left_count}, Right: {right_count}, Error: {error_mm:.1f}mm")

                # Completion when both wheels within their tolerances
                if abs(err_L_pulses) <= tolerance_pulses_L and abs(err_R_pulses) <= tolerance_pulses_R:
                    if debug:
                        print(f"  Target reached! Final position: {avg_mm_done:.1f}mm")
                    break

                # Early exit if we've overshot significantly
                if avg_mm_done > mm + 20:  # 20mm overshoot tolerance
                    if debug:
                        print(f"  Stopping due to overshoot: {avg_mm_done:.1f}mm > {mm + 20}mm")
                    break

                # PID core
                integral += error_mm * dt
                integral = clamp(integral, -integral_limit, integral_limit)
                derivative = (error_mm - prev_error_mm) / dt
                prev_error_mm = error_mm
                pid_output = Kp * error_mm + Ki * integral + Kd * derivative

                # Base speed (backward only) - reduce as we approach target
                speed_cmd = int(abs(pid_output))
                
                # Slow down when close to target
                if error_mm < 20:  # Within 20mm of target
                    speed_cmd = int(speed_cmd * max(0.3, error_mm / 20.0))
                
                if speed_cmd < MIN_SPEED and error_mm > 5:  # Only apply minimum if we're still far from target
                    speed_cmd = MIN_SPEED
                speed_cmd = clamp(speed_cmd, 0, MAX_SPEED)

                # Steering correction in mm space (use absolute values since we're going backward)
                left_mm = abs(left_count) / pulses_per_mm_L
                right_mm = abs(right_count) / pulses_per_mm_R
                diff_mm = left_mm - right_mm  # Positive if left wheel has moved more
                steer = int(K_steer * diff_mm)

                left_speed = clamp(speed_cmd - steer, 0, MAX_SPEED)
                right_speed = clamp(speed_cmd + steer, 0, MAX_SPEED)

                # Stop motors if we're very close to target to prevent overshoot
                if error_mm < 2:
                    left_speed = right_speed = 0

                # Use RWD (reverse) direction for both motors
                self.motors.set_motor(L_MTR, RWD, int(left_speed))
                self.motors.set_motor(R_MTR, RWD, int(right_speed))

                time.sleep(dt)
        finally:
            self.motors.stop_motors()
            if self.web_interface:
                self.web_interface.update_status(current_action="Idle")
                self.web_interface.log_debug(f"Completed backward movement")
            
            # Calculate final position for return value
            raw_left_final, raw_right_final = self.encoders.get_counts()
            left_final = ENCODER_LEFT_SIGN * raw_left_final
            right_final = ENCODER_RIGHT_SIGN * raw_right_final
            final_mm = 0.5 * (abs(left_final) / pulses_per_mm_L + abs(right_final) / pulses_per_mm_R)
            error_mm = final_mm - mm
            
            # Final debug output
            if debug:
                print(f"  Final: {final_mm:.1f}mm (requested {mm}mm, error: {error_mm:+.1f}mm)")
                print(f"  Final pulses - Left: {left_final}, Right: {right_final}")
                
                # Calculate actual pulses per mm based on this run
                if final_mm > 0:
                    actual_ppm_L = abs(left_final) / final_mm
                    actual_ppm_R = abs(right_final) / final_mm
                    print(f"  Measured pulses/mm - Left: {actual_ppm_L:.2f}, Right: {actual_ppm_R:.2f}")
                    
                    # Suggested corrections
                    suggested_ppr_L = int(actual_ppm_L * wheel_circumference_mm)
                    suggested_ppr_R = int(actual_ppm_R * wheel_circumference_mm)
                    print(f"  Suggested PPR - Left: {suggested_ppr_L}, Right: {suggested_ppr_R}")

            # Return movement result
            result = f"Backward movement completed. Requested: {mm}mm, Actual: {final_mm:.1f}mm, Error: {error_mm:+.1f}mm"
            if self.web_interface:
                self.web_interface.log_debug(result)
            return result

    @tool
    def rotate_cw(self, deg: int, debug: bool = False):
        """Rotate clockwise by specified degrees.

        Args:
            deg: Degrees to rotate clockwise (positive value)
            debug: Print debug information if True
        """
        if self.web_interface:
            self.web_interface.update_status(current_action=f"Rotating clockwise {deg}°")
            self.web_interface.log_debug(f"Starting clockwise rotation: {deg}°")
        return self._rotate_by_deg(-abs(deg), debug=debug)

    @tool
    def rotate_ccw(self, deg: int, debug: bool = False):
        """Rotate counter-clockwise by specified degrees.
        
        Args:
            deg: Degrees to rotate counter-clockwise (positive value)
            debug: Print debug information if True
        """
        if debug:
            print(f"Starting CCW rotation of {deg}°")
        if self.web_interface:
            self.web_interface.update_status(current_action=f"Rotating counter-clockwise {deg}°")
            self.web_interface.log_debug(f"Starting counter-clockwise rotation: {deg}°")
        # Counter-clockwise is positive heading
        return self._rotate_by_deg(abs(deg), debug=debug)

    def _rotate_by_deg(self, deg: float, debug: bool = False):
        # Store the requested amount (absolute value) for error calculation
        requested_deg = abs(deg)
        
        def normalize_angle(angle):
            """Normalize angle to [-180, 180] range"""
            while angle > 180:
                angle -= 360
            while angle <= -180:
                angle += 360
            return angle
        
        def angle_diff(target, current):
            """Calculate shortest angular difference"""
            diff = normalize_angle(target - current)
            return diff
        
        # Get starting heading
        current_heading = self.imu.get_heading()
        target_heading = normalize_angle(current_heading + deg)
        
        if debug:
            print(f"Starting rotation: {deg}° (from {current_heading:.1f}° to {target_heading:.1f}°)")
        
        # Rotation parameters
        tolerance = 2.0  # degrees
        max_turn_time = abs(deg) / 45.0 + 2.0  # Estimate based on ~45°/s turn rate
        dt = 0.05  # 20Hz control loop
        
        # Turn direction and speed
        turn_direction = 1 if deg > 0 else -1  # 1=CCW, -1=CW
        base_speed = TURN_MIN_SPEED + int(0.3 * (TURN_MAX_SPEED - TURN_MIN_SPEED))
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_turn_time:
                current_heading = self.imu.get_heading()
                error = angle_diff(target_heading, current_heading)
                
                if debug:
                    print(f"Current: {current_heading:6.1f}°, Target: {target_heading:6.1f}°, Error: {error:5.1f}°")
                
                # Check if we've reached the target
                if abs(error) <= tolerance:
                    if debug:
                        print(f"Rotation complete! Final error: {error:.1f}°")
                    break
                
                # Simple proportional control for speed
                speed_factor = min(1.0, abs(error) / 20.0)  # Slow down as we approach target
                turn_speed = int(TURN_MIN_SPEED + speed_factor * (base_speed - TURN_MIN_SPEED))
                
                # Set motor directions based on turn direction
                if turn_direction > 0:  # CCW: left reverse, right forward
                    self.motors.set_motor(L_MTR, RWD, turn_speed)
                    self.motors.set_motor(R_MTR, FWD, turn_speed)
                else:  # CW: left forward, right reverse
                    self.motors.set_motor(L_MTR, FWD, turn_speed)
                    self.motors.set_motor(R_MTR, RWD, turn_speed)
                
                time.sleep(dt)
            
            # Timeout reached
            if time.time() - start_time >= max_turn_time:
                final_heading = self.imu.get_heading()
                final_error = angle_diff(target_heading, final_heading)
                if debug:
                    print(f"Rotation timeout! Final error: {final_error:.1f}°")
                    
        finally:
            self.motors.stop_motors()
            if self.web_interface:
                self.web_interface.update_status(current_action="Idle")
                self.web_interface.log_debug(f"Completed rotation")
            
            # Calculate actual rotation for return value
            final_heading = self.imu.get_heading()
            raw_rotation = angle_diff(final_heading, current_heading)
            actual_rotation = abs(raw_rotation)
            error_deg = actual_rotation - requested_deg
            
            if debug:
                print(f"Rotation debug: start={current_heading:.1f}°, end={final_heading:.1f}°, raw_diff={raw_rotation:.1f}°")
                print(f"Rotation summary: requested {requested_deg}°, actual {actual_rotation:.1f}°")

            # Return rotation result with debug info
            result = f"Rotation completed. Requested: {requested_deg:.1f}°, Actual: {actual_rotation:.1f}°, Error: {error_deg:+.1f}° (Start: {current_heading:.1f}°, End: {final_heading:.1f}°)"
            if self.web_interface:
                self.web_interface.log_debug(result)
            return result



