#!/usr/bin/env python3
"""Test enhanced IMU reliability and diagnostics."""

import sys
import time
sys.path.append('../src')

from imu import IMU
import math

def test_imu_reliability():
    print("Testing Enhanced IMU Reliability")
    print("=" * 50)
    
    # Initialize IMU
    imu = IMU()
    
    # Check initial health
    health = imu.get_imu_health()
    print(f"Initial IMU Health: {health}")
    
    if not health.get("connected", False):
        print("IMU not connected - cannot run reliability tests")
        return
    
    print("\nTesting reading stability over 30 samples...")
    readings = []
    errors = 0
    
    for i in range(30):
        try:
            heading = imu.get_orientation(smooth=True)
            readings.append(heading)
            print(f"Sample {i+1:2d}: {math.degrees(heading):6.1f}° ", end="")
            
            # Show health every 10 samples
            if (i + 1) % 10 == 0:
                health = imu.get_imu_health()
                print(f"[Health: {health['status']}, Errors: {health['consecutive_errors']}]")
            else:
                print()
                
        except Exception as e:
            errors += 1
            print(f"Sample {i+1:2d}: ERROR - {e}")
        
        time.sleep(0.1)
    
    # Analyze stability
    if len(readings) >= 2:
        # Calculate standard deviation
        mean_heading = sum(readings) / len(readings)
        variance = sum((r - mean_heading)**2 for r in readings) / len(readings)
        std_dev = math.sqrt(variance)
        
        print(f"\nReliability Analysis:")
        print(f"  Valid readings: {len(readings)}/30")
        print(f"  Error rate: {errors/30*100:.1f}%")
        print(f"  Mean heading: {math.degrees(mean_heading):.1f}°")
        print(f"  Standard deviation: {math.degrees(std_dev):.2f}°")
        print(f"  Max deviation: {math.degrees(max(readings) - min(readings)):.1f}°")
        
        # Final health check
        final_health = imu.get_imu_health()
        print(f"\nFinal IMU Health: {final_health}")
        
        # Recommendation
        if std_dev < math.radians(2.0) and errors == 0:
            print("✅ IMU performance: EXCELLENT")
        elif std_dev < math.radians(5.0) and errors < 3:
            print("✅ IMU performance: GOOD")
        elif std_dev < math.radians(10.0) and errors < 10:
            print("⚠️  IMU performance: ACCEPTABLE")
        else:
            print("❌ IMU performance: POOR - consider recalibration")
    
    # Test outlier rejection
    print(f"\nTesting outlier rejection...")
    print("Readings with and without smoothing:")
    
    for i in range(5):
        raw = imu.get_orientation(smooth=False)
        smooth = imu.get_orientation(smooth=True)
        print(f"  Raw: {math.degrees(raw):6.1f}°  Smooth: {math.degrees(smooth):6.1f}°  Diff: {math.degrees(abs(raw-smooth)):4.1f}°")
        time.sleep(0.2)

if __name__ == "__main__":
    test_imu_reliability()
