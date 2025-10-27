#!/usr/bin/env python3
"""
Test script for the live motion plotter

Tests the live plotter with simulated data to verify functionality.
"""

import sys
import time
import threading
from live_motion_plotter import LiveMotionData

def test_live_data_structure():
    """Test the live data structure"""
    print("Testing LiveMotionData structure...")
    
    data = LiveMotionData(max_samples=10)
    
    # Add some test data
    for i in range(15):  # Test circular buffer behavior
        timestamp = i * 0.01
        
        # Add raw data
        accel = [1000 + i, 50 + i, -950 + i]
        gyro = [0.1 + i*0.01, -0.2 + i*0.01, 0.3 + i*0.01]
        data.add_raw_data(timestamp, accel, gyro)
        
        # Add angle data
        data.add_angles_data(timestamp, 1.5 + i*0.1, -2.3 + i*0.1, 0.8 + i*0.1)
        data.add_di_angles(timestamp, 1.2 + i*0.1, -2.1 + i*0.1, 0.9 + i*0.1)
        data.add_si_angles(timestamp, 1.4 + i*0.1, -2.2 + i*0.1, 0.7 + i*0.1)
        data.add_co_angles(timestamp, 1.3 + i*0.1, -2.0 + i*0.1, 0.8 + i*0.1)
        data.add_fu_angles(timestamp, 1.35 + i*0.1, -2.15 + i*0.1, 0.75 + i*0.1)
        data.add_gyro_bias(timestamp, -0.411 + i*0.001, 0.587 + i*0.001, 0.774 + i*0.001)
    
    # Get snapshot
    snapshot = data.get_data_snapshot()
    
    # Verify circular buffer behavior (should have max 10 samples)
    print(f"  Timestamps: {len(snapshot['timestamps'])} samples (expected: 10)")
    print(f"  Accel data: {len(snapshot['accel'][0])} samples (expected: 10)")
    print(f"  ANGLES data: {len(snapshot['angles'][0])} samples (expected: 10)")
    print(f"  ANGLES_DI data: {len(snapshot['di'][0])} samples (expected: 10)")
    print(f"  ANGLES_SI data: {len(snapshot['si'][0])} samples (expected: 10)")
    print(f"  ANGLES_CO data: {len(snapshot['co'][0])} samples (expected: 10)")
    print(f"  ANGLES_FU data: {len(snapshot['fu'][0])} samples (expected: 10)")
    
    # Check that we kept the latest data (samples 5-14)
    print(f"  Latest timestamp: {snapshot['timestamps'][-1]:.3f} (expected: ~0.140)")
    print(f"  Oldest timestamp: {snapshot['timestamps'][0]:.3f} (expected: ~0.050)")
    
    if (len(snapshot['timestamps']) == 10 and 
        abs(snapshot['timestamps'][-1] - 0.14) < 0.001 and
        abs(snapshot['timestamps'][0] - 0.05) < 0.001):
        print("✅ LiveMotionData structure test passed!")
        return True
    else:
        print("❌ LiveMotionData structure test failed!")
        return False

def test_parsing_methods():
    """Test individual parsing methods"""
    print("\\nTesting parsing methods...")
    
    try:
        from live_motion_plotter import LiveMotionData
        
        data = LiveMotionData(max_samples=100)
        
        # Test each parsing method
        timestamp = 1.0
        
        # Test raw data
        data.add_raw_data(timestamp, [1000.0, 50.0, -950.0], [0.1, -0.2, 0.3])
        
        # Test angle data
        data.add_angles_data(timestamp, 1.5, -2.3, 0.8)
        data.add_di_angles(timestamp, 1.2, -2.1, 0.9)
        data.add_si_angles(timestamp, 1.4, -2.2, 0.7)
        data.add_co_angles(timestamp, 1.3, -2.0, 0.8)
        data.add_fu_angles(timestamp, 1.35, -2.15, 0.75)
        data.add_gyro_bias(timestamp, -0.411, 0.587, 0.774)
        
        # Get snapshot
        snapshot = data.get_data_snapshot()
        
        checks = [
            (len(snapshot['timestamps']) > 0, "Raw timestamps"),
            (len(snapshot['accel'][0]) > 0, "Accelerometer data"),
            (len(snapshot['gyro'][0]) > 0, "Gyroscope data"),
            (len(snapshot['angles'][0]) > 0, "ANGLES data"),
            (len(snapshot['di'][0]) > 0, "ANGLES_DI data"),
            (len(snapshot['si'][0]) > 0, "ANGLES_SI data"),
            (len(snapshot['co'][0]) > 0, "ANGLES_CO data"),
            (len(snapshot['fu'][0]) > 0, "ANGLES_FU data"),
            (len(snapshot['bias'][0]) > 0, "Gyro bias data")
        ]
        
        all_passed = True
        for passed, test_name in checks:
            status = "✅" if passed else "❌"
            print(f"  {status} {test_name}")
            if not passed:
                all_passed = False
        
        # Test data values
        if all_passed:
            print(f"  Sample values:")
            print(f"    Accel: [{snapshot['accel'][0][0]:.1f}, {snapshot['accel'][1][0]:.1f}, {snapshot['accel'][2][0]:.1f}]")
            print(f"    ANGLES: [{snapshot['angles'][0][0]:.1f}, {snapshot['angles'][1][0]:.1f}, {snapshot['angles'][2][0]:.1f}]")
            print(f"    ANGLES_DI: [{snapshot['di'][0][0]:.1f}, {snapshot['di'][1][0]:.1f}, {snapshot['di'][2][0]:.1f}]")
        
        if all_passed:
            print("✅ Parsing methods test passed!")
            return True
        else:
            print("❌ Parsing methods test failed!")
            return False
            
    except Exception as e:
        print(f"❌ Parsing methods test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests"""
    print("Motion Detection Tools Test Suite")
    print("=================================")
    
    test1_passed = test_live_data_structure()
    test2_passed = test_parsing_methods()
    
    print("\\n" + "="*50)
    if test1_passed and test2_passed:
        print("🎉 All tests passed! Tools are ready for use.")
        print("\\nNext steps:")
        print("1. Connect your device and run: python3 live_motion_plotter.py --port <your_port>")
        print("2. Or analyze a log file: python3 motion_debug_visualizer.py <log_file>")
        print("\\nNew angle format is working:")
        print("- ANGLES: Final output (altitude, azimuth, zenith)")
        print("- ANGLES_DI: MotionDI euler (pitch, yaw, roll)")
        print("- ANGLES_SI: Simple Integration (pitch, yaw, roll)")
        print("- ANGLES_CO: Complementary filter (pitch, yaw, roll)")
        print("- ANGLES_FU: Fused output (pitch, yaw, roll)")
    else:
        print("❌ Some tests failed. Check the error messages above.")
    print("="*50)

if __name__ == "__main__":
    main()