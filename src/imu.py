import time
import sys
import math

try:
    import qwiic_icm20948
except Exception:  # Allow import to fail gracefully on non-target hosts
    qwiic_icm20948 = None

from config import (
    DECLINATION_DEG,
    MAG_OFFSET_X,
    MAG_OFFSET_Y,
    MAG_OFFSET_Z,
    MAG_SIGN_X,
    MAG_SIGN_Y,
    MAG_SIGN_Z,
)


def _wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


class IMU:
    def __init__(self, baseline_samples: int = 20, declination_deg: float = DECLINATION_DEG):
        self._connected = False
        self._declination = math.radians(declination_deg)
        self._zero_heading = 0.0
        self._imu = None
        # Enhanced smoothing for heading readings
        self._heading_history = []
        self._max_history = 8  # Increased from 5 for better smoothing
        # Error detection and recovery
        self._consecutive_errors = 0
        self._max_consecutive_errors = 5
        self._last_valid_heading = 0.0
        # Data validation
        self._mag_magnitude_range = (10.0, 1000.0)  # Valid magnetometer magnitude range
        self._accel_magnitude_range = (0.5, 2.0)    # Valid accelerometer magnitude range (in g's)

        try:
            if qwiic_icm20948 is None:
                return
            imu = qwiic_icm20948.QwiicIcm20948()
            if not getattr(imu, "connected", False):
                return
            imu.begin()
            time.sleep(0.25)
            
            # Configure IMU for better stability
            self._configure_imu(imu)
            
            self._imu = imu
            self._connected = True
        except Exception:
            self._connected = False
            self._imu = None
            return

        # Establish baseline heading with validation
        try:
            self._zero_heading = self._average_heading(baseline_samples)
        except Exception:
            self._zero_heading = 0.0

    def get_orientation(self, smooth: bool = True) -> float:
        """Return current heading relative to startup heading, in radians [-pi, pi]."""
        if not self._connected:
            return 0.0
        
        try:
            current = self._compute_heading()
            if current is None:  # Invalid reading
                if self._consecutive_errors < self._max_consecutive_errors:
                    return self._last_valid_heading  # Return last known good value
                else:
                    return 0.0  # Too many errors, return safe default
            
            # Reset error counter on successful reading
            self._consecutive_errors = 0
            rel = _wrap_pi(current - self._zero_heading)
            self._last_valid_heading = rel
            
        except Exception:
            self._consecutive_errors += 1
            if self._consecutive_errors < self._max_consecutive_errors:
                return self._last_valid_heading
            else:
                return 0.0
        
        if smooth and rel is not None:
            # Enhanced smoothing with outlier rejection
            rel = self._apply_smoothing(rel)
        
        return rel

    def calibrate_zero(self, samples: int = 20) -> None:
        if not self._connected:
            self._zero_heading = 0.0
            return
        # Clear history when recalibrating
        self._heading_history.clear()
        self._consecutive_errors = 0  # Reset error counter
        try:
            self._zero_heading = self._average_heading(samples)
        except Exception:
            self._zero_heading = 0.0  # Fallback to safe default

    def get_imu_health(self) -> dict:
        """Return IMU health status and diagnostics."""
        if not self._connected:
            return {"connected": False, "status": "disconnected"}
            
        try:
            axes_data = self._read_axes()
            if axes_data is None:
                return {
                    "connected": True,
                    "status": "error",
                    "consecutive_errors": self._consecutive_errors,
                    "data_valid": False
                }
                
            ax, ay, az, mx, my, mz = axes_data
            accel_mag = math.sqrt(ax*ax + ay*ay + az*az) / 16384.0  # Convert to g's
            mag_mag = math.sqrt(mx*mx + my*my + mz*mz)
            
            return {
                "connected": True,
                "status": "healthy" if self._consecutive_errors == 0 else "degraded",
                "consecutive_errors": self._consecutive_errors,
                "data_valid": True,
                "accel_magnitude_g": round(accel_mag, 3),
                "mag_magnitude": round(mag_mag, 1),
                "history_length": len(self._heading_history),
                "last_valid_heading_deg": round(math.degrees(self._last_valid_heading), 1)
            }
        except Exception as e:
            return {
                "connected": True,
                "status": "error",
                "error": str(e),
                "consecutive_errors": self._consecutive_errors
            }

    # ---------------- Internals ----------------
    def _configure_imu(self, imu) -> None:
        """Configure IMU for optimal performance and stability."""
        try:
            # Set magnetometer to higher resolution mode if available
            if hasattr(imu, 'setMagFS'):
                imu.setMagFS(0)  # ±4900 μT range for better resolution
            
            # Set accelerometer range for better resolution
            if hasattr(imu, 'setAccelFS'):
                imu.setAccelFS(0)  # ±2g range for better resolution
                
            # Enable data ready interrupt if available
            if hasattr(imu, 'enableDataReadyInterrupt'):
                imu.enableDataReadyInterrupt()
                
        except Exception:
            pass  # Gracefully handle unsupported features

    def _apply_smoothing(self, new_heading: float) -> float:
        """Apply enhanced smoothing with outlier rejection."""
        # Check for outliers before adding to history
        if len(self._heading_history) >= 2:
            # Calculate angular difference from recent history
            recent_avg = self._circular_mean(self._heading_history[-2:])
            angular_diff = abs(_wrap_pi(new_heading - recent_avg))
            
            # Reject obvious outliers (>45° sudden change)
            if angular_diff > math.radians(45):
                return self._last_valid_heading if self._last_valid_heading else 0.0
        
        # Add to history
        self._heading_history.append(new_heading)
        if len(self._heading_history) > self._max_history:
            self._heading_history.pop(0)
        
        # Apply circular averaging
        if len(self._heading_history) >= 3:
            return self._circular_mean(self._heading_history)
        else:
            return new_heading

    def _circular_mean(self, angles: list) -> float:
        """Compute circular mean of angles in radians."""
        if not angles:
            return 0.0
        s_x = sum(math.cos(h) for h in angles)
        s_y = sum(math.sin(h) for h in angles)
        return math.atan2(s_y, s_x)

    def _validate_sensor_data(self, ax: float, ay: float, az: float, 
                            mx: float, my: float, mz: float) -> bool:
        """Validate sensor readings for sanity."""
        # Check accelerometer magnitude (should be ~1g when stationary)
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        accel_g = accel_mag / 16384.0  # Convert to g's (assuming ±2g scale)
        
        if not (self._accel_magnitude_range[0] <= accel_g <= self._accel_magnitude_range[1]):
            return False
        
        # Check magnetometer magnitude
        mag_mag = math.sqrt(mx*mx + my*my + mz*mz)
        if not (self._mag_magnitude_range[0] <= mag_mag <= self._mag_magnitude_range[1]):
            return False
            
        # Check for NaN or infinite values
        values = [ax, ay, az, mx, my, mz]
        if any(not math.isfinite(v) for v in values):
            return False
            
        return True
    def _update(self) -> None:
        """Update sensor readings with retry logic."""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                if hasattr(self._imu, "dataReady") and self._imu.dataReady():
                    self._imu.getAgmt()
                    return
                elif hasattr(self._imu, "getAgmt"):
                    self._imu.getAgmt()
                    return
            except Exception:
                if attempt < max_retries - 1:
                    time.sleep(0.001)  # Brief pause before retry
                    continue
                else:
                    raise  # Re-raise on final attempt

    def _read_axes(self):
        """Read and validate sensor axes with error handling."""
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self._update()

                def _gx(primary: str, fallback: str):
                    return getattr(self._imu, primary, getattr(self._imu, fallback, 0.0))

                ax = _gx("aX", "axRaw")
                ay = _gx("aY", "ayRaw")
                az = _gx("aZ", "azRaw")
                mx = _gx("mX", "mxRaw")
                my = _gx("mY", "myRaw")
                mz = _gx("mZ", "mzRaw")
                
                # Apply hard-iron offsets and axis signs
                mx = (mx - MAG_OFFSET_X) * MAG_SIGN_X
                my = (my - MAG_OFFSET_Y) * MAG_SIGN_Y
                mz = (mz - MAG_OFFSET_Z) * MAG_SIGN_Z
                
                # Validate the readings
                if self._validate_sensor_data(ax, ay, az, mx, my, mz):
                    return ax, ay, az, mx, my, mz
                else:
                    if attempt < max_attempts - 1:
                        time.sleep(0.005)  # Brief pause before retry
                        continue
                    else:
                        return None  # Return None for invalid data
                        
            except Exception:
                if attempt < max_attempts - 1:
                    time.sleep(0.005)
                    continue
                else:
                    return None

    def _compute_heading(self) -> float:
        """Compute heading with enhanced error handling."""
        axes_data = self._read_axes()
        if axes_data is None:
            return None  # Signal invalid reading
            
        ax, ay, az, mx, my, mz = axes_data

        # Compute roll and pitch from accelerometer with bounds checking
        try:
            # Normalize accelerometer data
            accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
            if accel_norm < 1e-6:  # Avoid division by zero
                return None
                
            ax_norm = ax / accel_norm
            ay_norm = ay / accel_norm  
            az_norm = az / accel_norm
            
            # Clamp values to valid range for atan2
            ay_norm = max(-1.0, min(1.0, ay_norm))
            ax_norm = max(-1.0, min(1.0, ax_norm))
            
            roll = math.atan2(ay_norm, az_norm)
            pitch = math.atan2(-ax_norm, math.sqrt(ay_norm * ay_norm + az_norm * az_norm))
            
        except Exception:
            return None

        # Tilt compensation for magnetometer with bounds checking
        try:
            cos_pitch = math.cos(pitch)
            sin_pitch = math.sin(pitch)
            cos_roll = math.cos(roll)
            sin_roll = math.sin(roll)
            
            Xh = mx * cos_pitch + mz * sin_pitch
            Yh = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch

            # Check for valid magnetometer components
            if abs(Xh) < 1e-6 and abs(Yh) < 1e-6:
                return None  # No valid magnetic field
                
            # Use atan2(-Yh, Xh) to align with common NED/board orientation so CCW is positive
            heading = math.atan2(-Yh, Xh) + self._declination
            heading = _wrap_pi(heading)
            return heading
            
        except Exception:
            return None

    def _average_heading(self, samples: int) -> float:
        """Compute average heading with enhanced error handling."""
        s_x = 0.0
        s_y = 0.0
        valid_samples = 0
        n = max(1, int(samples))
        
        for i in range(n):
            try:
                h = self._compute_heading()
                if h is not None:  # Only include valid readings
                    s_x += math.cos(h)
                    s_y += math.sin(h)
                    valid_samples += 1
                time.sleep(0.02)
            except Exception:
                continue  # Skip invalid readings
                
        if valid_samples < n // 2:  # Need at least half valid samples
            raise Exception(f"Insufficient valid samples: {valid_samples}/{n}")
            
        return math.atan2(s_y, s_x)


if __name__ == "__main__":
    imu = IMU()
    print(math.degrees(imu.get_orientation()))
    time.sleep(5)
    print(math.degrees(imu.get_orientation()))