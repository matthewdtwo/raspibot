import time
import math
import signal
import atexit
from typing import Tuple, NamedTuple

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


class MovementParams(NamedTuple):
    """Parameters for movement calculations"""
    wheel_circumference_mm: float
    pulses_per_mm_L: float
    pulses_per_mm_R: float
    target_pulses_L: int
    target_pulses_R: int


class PIDState(NamedTuple):
    """State for PID controller"""
    Kp: float = 0.25
    Ki: float = 0.0
    Kd: float = 0.02
    K_steer: float = 0.035
    dt: float = 0.05
    tolerance_mm: float = 3.0
    integral_limit_factor: int = 4


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

    def _calculate_movement_params(self, mm: int, direction: int = 1) -> MovementParams:
        """Calculate movement parameters for a given distance.
        
        Args:
            mm: Distance in millimeters
            direction: 1 for forward, -1 for backward
            
        Returns:
            MovementParams with all calculated values
        """
        wheel_circumference_mm = math.pi * WHEEL_DIAMETER
        pulses_per_mm_L = PULSES_PER_ROTATION_LEFT / wheel_circumference_mm
        pulses_per_mm_R = PULSES_PER_ROTATION_RIGHT / wheel_circumference_mm
        target_pulses_L = int(direction * mm * pulses_per_mm_L)
        target_pulses_R = int(direction * mm * pulses_per_mm_R)
        
        return MovementParams(
            wheel_circumference_mm=wheel_circumference_mm,
            pulses_per_mm_L=pulses_per_mm_L,
            pulses_per_mm_R=pulses_per_mm_R,
            target_pulses_L=target_pulses_L,
            target_pulses_R=target_pulses_R
        )

    def _log_movement_start(self, direction: str, mm: int, movement_params: MovementParams, debug: bool):
        """Log the start of a movement operation"""
        if self.web_interface:
            self.web_interface.update_status(current_action=f"Moving {direction} {mm}mm")
            self.web_interface.log_debug(f"Starting {direction} movement: {mm}mm")
            
        if debug:
            print(f"Move {direction} {mm}mm:")
            print(f"  Wheel circumference: {movement_params.wheel_circumference_mm:.1f}mm")
            print(f"  Pulses/mm - Left: {movement_params.pulses_per_mm_L:.2f}, Right: {movement_params.pulses_per_mm_R:.2f}")
            print(f"  Target pulses - Left: {movement_params.target_pulses_L}, Right: {movement_params.target_pulses_R}")

    def _calculate_movement_progress(self, movement_params: MovementParams, is_backward: bool = False) -> Tuple[float, int, int, int, int]:
        """Calculate current movement progress and errors.
        
        Returns:
            Tuple of (avg_mm_done, left_count, right_count, err_L_pulses, err_R_pulses)
        """
        raw_left_count, raw_right_count = self.encoders.get_counts()
        left_count = ENCODER_LEFT_SIGN * raw_left_count
        right_count = ENCODER_RIGHT_SIGN * raw_right_count

        err_L_pulses = movement_params.target_pulses_L - left_count
        err_R_pulses = movement_params.target_pulses_R - right_count

        if is_backward:
            # For backward movement, use absolute values for progress calculation
            avg_mm_done = 0.5 * (abs(left_count) / movement_params.pulses_per_mm_L + 
                               abs(right_count) / movement_params.pulses_per_mm_R)
        else:
            # For forward movement
            avg_mm_done = 0.5 * (left_count / movement_params.pulses_per_mm_L + 
                               right_count / movement_params.pulses_per_mm_R)

        return avg_mm_done, left_count, right_count, err_L_pulses, err_R_pulses

    def _calculate_movement_speeds(self, error_mm: float, left_count: int, right_count: int, 
                                 movement_params: MovementParams, pid_state: PIDState, 
                                 integral: float, prev_error_mm: float, is_backward: bool = False) -> Tuple[int, int, float, float]:
        """Calculate motor speeds for movement control.
        
        Returns:
            Tuple of (left_speed, right_speed, new_integral, derivative)
        """
        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        # PID core
        new_integral = integral + error_mm * pid_state.dt
        new_integral = clamp(new_integral, -pid_state.integral_limit_factor * MAX_SPEED, 
                           pid_state.integral_limit_factor * MAX_SPEED)
        derivative = (error_mm - prev_error_mm) / pid_state.dt
        pid_output = pid_state.Kp * error_mm + pid_state.Ki * new_integral + pid_state.Kd * derivative

        # Base speed - reduce as we approach target
        speed_cmd = int(abs(pid_output))
        
        # Slow down when close to target
        if error_mm < 20:  # Within 20mm of target
            speed_cmd = int(speed_cmd * max(0.3, error_mm / 20.0))
        
        if speed_cmd < MIN_SPEED and error_mm > 5:  # Only apply minimum if we're still far from target
            speed_cmd = MIN_SPEED
        speed_cmd = clamp(speed_cmd, 0, MAX_SPEED)

        # Steering correction
        if is_backward:
            left_mm = abs(left_count) / movement_params.pulses_per_mm_L
            right_mm = abs(right_count) / movement_params.pulses_per_mm_R
        else:
            left_mm = left_count / movement_params.pulses_per_mm_L
            right_mm = right_count / movement_params.pulses_per_mm_R
            
        diff_mm = left_mm - right_mm
        steer = int(pid_state.K_steer * diff_mm)

        left_speed = clamp(speed_cmd - steer, 0, MAX_SPEED)
        right_speed = clamp(speed_cmd + steer, 0, MAX_SPEED)

        # Stop motors if we're very close to target to prevent overshoot
        if error_mm < 2:
            left_speed = right_speed = 0

        return left_speed, right_speed, new_integral, derivative

    def _log_movement_completion(self, direction: str, mm: int, movement_params: MovementParams, 
                               debug: bool, is_backward: bool = False) -> str:
        """Log movement completion and return result string"""
        if self.web_interface:
            self.web_interface.update_status(current_action="Idle")
            self.web_interface.log_debug(f"Completed {direction} movement")
        
        # Calculate final position for return value
        raw_left_final, raw_right_final = self.encoders.get_counts()
        left_final = ENCODER_LEFT_SIGN * raw_left_final
        right_final = ENCODER_RIGHT_SIGN * raw_right_final
        
        if is_backward:
            final_mm = 0.5 * (abs(left_final) / movement_params.pulses_per_mm_L + 
                            abs(right_final) / movement_params.pulses_per_mm_R)
        else:
            final_mm = 0.5 * (left_final / movement_params.pulses_per_mm_L + 
                            right_final / movement_params.pulses_per_mm_R)
            
        error_mm = final_mm - mm
        
        # Final debug output
        if debug:
            print(f"  Final: {final_mm:.1f}mm (requested {mm}mm, error: {error_mm:+.1f}mm)")
            print(f"  Final pulses - Left: {left_final}, Right: {right_final}")
            
            # Calculate actual pulses per mm based on this run
            if final_mm > 0:
                if is_backward:
                    actual_ppm_L = abs(left_final) / final_mm
                    actual_ppm_R = abs(right_final) / final_mm
                else:
                    actual_ppm_L = left_final / final_mm
                    actual_ppm_R = right_final / final_mm
                    
                print(f"  Measured pulses/mm - Left: {actual_ppm_L:.2f}, Right: {actual_ppm_R:.2f}")
                
                # Suggested corrections
                suggested_ppr_L = int(abs(actual_ppm_L) * movement_params.wheel_circumference_mm)
                suggested_ppr_R = int(abs(actual_ppm_R) * movement_params.wheel_circumference_mm)
                print(f"  Suggested PPR - Left: {suggested_ppr_L}, Right: {suggested_ppr_R}")

        # Return movement result
        result = f"{direction.capitalize()} movement completed. Requested: {mm}mm, Actual: {final_mm:.1f}mm, Error: {error_mm:+.1f}mm"
        if self.web_interface:
            self.web_interface.log_debug(result)
        return result

    @staticmethod
    def _clamp(val, lo, hi):
        """Utility function to clamp a value between bounds"""
        return max(lo, min(hi, val))

    @tool
    def move_forward(self, mm: int, debug: bool = False):
        """Drive forward a distance in millimeters using a simple PID on encoder counts.
        Does not account for slippage if you get stuck on something, so move in small increments.
        Controls overall progress using average mm while keeping wheels aligned.
        Uses per-wheel encoder calibration and normalized signs.
        """
        return self._execute_linear_movement(mm, "forward", FWD, debug=debug)

    @tool
    def move_backward(self, mm: int, debug: bool = False):
        """Drive backward a distance in millimeters using a simple PID on encoder counts.
        Use sparingly, since you can't see behind you. Only move in small increments. Does not account for wheel slippage if you get stuck.

        Controls overall progress using average mm while keeping wheels aligned.
        Uses per-wheel encoder calibration and normalized signs.
        """
        return self._execute_linear_movement(mm, "backward", RWD, debug=debug)

    def _execute_linear_movement(self, mm: int, direction_name: str, motor_direction: int, debug: bool = False) -> str:
        """Execute linear movement (forward or backward) with shared logic.
        
        Args:
            mm: Distance in millimeters
            direction_name: "forward" or "backward" for logging
            motor_direction: FWD or RWD motor direction constant
            debug: Enable debug output
            
        Returns:
            Movement result string
        """
        is_backward = motor_direction == RWD
        direction_multiplier = -1 if is_backward else 1
        
        # Calculate movement parameters
        movement_params = self._calculate_movement_params(mm, direction_multiplier)
        
        # Log movement start
        self._log_movement_start(direction_name, mm, movement_params, debug)

        # Initialize PID and control parameters
        pid_state = PIDState()
        tolerance_pulses_L = max(2, int(pid_state.tolerance_mm * movement_params.pulses_per_mm_L))
        tolerance_pulses_R = max(2, int(pid_state.tolerance_mm * movement_params.pulses_per_mm_R))
        integral = 0.0
        prev_error_mm = mm

        # Timeout based on distance
        est_mm_per_s = 80.0
        timeout_s = max(2.0, mm / est_mm_per_s + 1.0)
        start_time = time.time()

        # Reset encoders
        self.encoders.reset_counts()

        try:
            loop_count = 0
            while True:
                loop_count += 1
                if time.time() - start_time > timeout_s:
                    if debug:
                        print("  Timeout reached!")
                    break

                # Calculate current progress and errors
                avg_mm_done, left_count, right_count, err_L_pulses, err_R_pulses = \
                    self._calculate_movement_progress(movement_params, is_backward)

                # Overall error in mm
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

                # Calculate motor speeds
                left_speed, right_speed, integral, derivative = \
                    self._calculate_movement_speeds(error_mm, left_count, right_count, 
                                                  movement_params, pid_state, integral, prev_error_mm, is_backward)
                
                prev_error_mm = error_mm

                # Set motor speeds
                self.motors.set_motor(L_MTR, motor_direction, int(left_speed))
                self.motors.set_motor(R_MTR, motor_direction, int(right_speed))

                time.sleep(pid_state.dt)
        finally:
            self.motors.stop_motors()
            return self._log_movement_completion(direction_name, mm, movement_params, debug, is_backward)

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
        return self._execute_rotation(-abs(deg), debug=debug)

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
        return self._execute_rotation(abs(deg), debug=debug)

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-180, 180] range"""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle
    
    @staticmethod
    def _angle_diff(target, current):
        """Calculate shortest angular difference"""
        diff = Robot._normalize_angle(target - current)
        return diff

    def _execute_rotation(self, deg: float, debug: bool = False) -> str:
        """Execute rotation with shared logic for both directions.
        
        Args:
            deg: Degrees to rotate (positive=CCW, negative=CW)
            debug: Enable debug output
            
        Returns:
            Rotation result string
        """
        # Store the requested amount (absolute value) for error calculation
        requested_deg = abs(deg)
        
        # Get starting heading
        current_heading = self.imu.get_heading()
        target_heading = self._normalize_angle(current_heading + deg)
        
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
                error = self._angle_diff(target_heading, current_heading)
                
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
                final_error = self._angle_diff(target_heading, final_heading)
                if debug:
                    print(f"Rotation timeout! Final error: {final_error:.1f}°")
                    
        finally:
            self.motors.stop_motors()
            if self.web_interface:
                self.web_interface.update_status(current_action="Idle")
                self.web_interface.log_debug(f"Completed rotation")
            
            # Calculate actual rotation for return value
            final_heading = self.imu.get_heading()
            raw_rotation = self._angle_diff(final_heading, current_heading)
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

    @tool
    def take_snapshot_and_analyze(self) -> str:
        """Take a snapshot of the environment and return an analysis of what's visible"""
        try:
            # Check if camera is available
            if not hasattr(self, 'camera') or self.camera is None:
                return "Error: No camera available"
            
            if self.web_interface:
                self.web_interface.update_status(current_action="Taking snapshot")
            
            # Take snapshot
            image_path = self.camera.take_snapshot()
            
            # Analyze it (assuming there's a method for this)
            if hasattr(self, '_get_image_description'):
                description = self._get_image_description(image_path)
            else:
                description = "Snapshot taken (no analysis available)"
            
            # Log to web interface
            if self.web_interface:
                self.web_interface.log_snapshot(image_path, description)
                self.web_interface.update_status(current_action="Analyzing snapshot")
            
            return f"Snapshot taken and analyzed. Description: {description}"
        except Exception as e:
            error_msg = f"Error taking snapshot: {e}"
            if self.web_interface:
                self.web_interface.update_status(current_action="Idle")
            return error_msg


