# Motion Detection Debugging Tools

This directory contains tools to help debug and analyze motion detection issues in the embedded system.

## Problem Analysis

Based on the code analysis, the current issue is:

**ANGLES output shows `0,0,0` instead of actual angle values**

### Root Cause
The problem is in the `_calculateAnglesToReference()` function in `motion_detection.cpp` (lines 800-820). The function is applying `fabsf()` (absolute value) to all angles:

```cpp
// Apply range clamping for motion detection
// We only care about magnitude, so clamp to positive ranges
yaw = fabsf(yaw);      // ❌ This loses directional information
pitch = fabsf(pitch);  // ❌ This loses directional information  
roll = fabsf(roll);    // ❌ This loses directional information
```

### What Should Happen vs What's Actually Happening

**Before (Working):**
- ANGLES showed: `1.802,-118.835,3.832`
- MotionEstimator showed: `Yaw=-118.837 deg, Pitch=8.848 deg, Roll=6.975 deg`
- Reference angles were: `Yaw=-166.455 deg, Pitch=6.680 deg, Roll=122.228 deg`

**After (Broken):**
- ANGLES shows: `0,0,0` 
- MotionEstimator shows: `Yaw=0.069 deg, Pitch=0.731 deg, Roll=10.938 deg`
- Reference angles are: `Yaw=0 deg, Pitch=0 deg, Roll=0 deg`

## Available Tools

### 1. Enhanced Motion Debug Visualizer (`motion_debug_visualizer.py`)

An enhanced tool that creates comprehensive plots of:
- Raw sensor data (accelerometer and gyroscope)
- ANGLES - Final output angles (altitude, azimuth, zenith)
- ANGLES_DI - MotionDI euler angles (pitch, yaw, roll)
- ANGLES_SI - Simple Integration filter (pitch, yaw, roll)
- ANGLES_CO - Complementary filter (pitch, yaw, roll)
- ANGLES_FU - Fused angles (pitch, yaw, roll)
- Gyro bias over time
- Comprehensive angle comparison
- Enhanced analysis summary

**Usage:**
```bash
# Static file analysis
python3 motion_debug_visualizer.py <log_file>
python3 motion_debug_visualizer.py motion_detection.log --output debug_plot.png

# Live data visualization
python3 enhanced_motion_visualizer.py --live --port COM15
python3 enhanced_motion_visualizer.py --live --port /dev/ttyUSB0 --baudrate 115200
```

### 1b. Live Motion Plotter (`live_motion_plotter.py`)

A dedicated live visualization tool for real-time monitoring:
- Real-time plotting of all angle sources
- Automatic serial connection management
- Optimized for continuous monitoring
- Side-by-side comparison of all filters

**Usage:**
```bash
python3 live_motion_plotter.py --port COM15
python3 live_motion_plotter.py --port /dev/ttyUSB0 --baudrate 115200 --samples 1000
```

### 2. Advanced Motion Parser (`advanced_motion_parser.py`)

A comprehensive tool that:
- Parses multi-line MotionEstimator debug sections
- Extracts MotionEstimator fusion data
- Provides detailed analysis
- Exports data to CSV for further analysis

**Usage:**
```bash
# Basic analysis
python3 advanced_motion_parser.py motion_detection.log

# Export to CSV files
python3 advanced_motion_parser.py motion_detection.log --csv

# Export with custom filename
python3 advanced_motion_parser.py motion_detection.log --csv --output my_analysis
```

## How to Use These Tools

### Option A: Live Real-Time Analysis (Recommended)

#### Step 1: Connect Device and Start Live Plotter
```bash
cd app/src/motion_detection/testing

# For Windows
python3 live_motion_plotter.py --port COM15

# For Linux
python3 live_motion_plotter.py --port /dev/ttyUSB0
```

#### Step 2: Enable Motion Detection Debug Output
The live plotter automatically enables debug output, but you can manually control it:
```
# In the device console
setdebug 127    # Enable all debug outputs including PRINT_ESTIMATOR
printraw 1      # Enable raw sensor data
```

### Option B: File-Based Analysis

#### Step 1: Collect Log Data
Run your motion detection system and capture the log output to a file:
```bash
./your_motion_detection_program > motion_detection.log 2>&1
```

#### Step 2: Analyze with Enhanced Visualizer
```bash
cd app/src/motion_detection/testing
python3 motion_debug_visualizer.py motion_detection.log
```

#### Step 3: Deep Analysis with Advanced Tool
```bash
python3 advanced_motion_parser.py motion_detection.log --csv
```

### Step 4: Review the Analysis
The enhanced tools will show you:
- **ANGLES**: Final output angles (altitude, azimuth, zenith)
- **ANGLES_DI**: MotionDI euler angles (pitch, yaw, roll)
- **ANGLES_SI**: Simple Integration filter output (pitch, yaw, roll)
- **ANGLES_CO**: Complementary filter output (pitch, yaw, roll)
- **ANGLES_FU**: Fused angles combining MDI + MotionEstimator (pitch, yaw, roll)
- Whether angles are all zero (indicating the fabsf() issue)
- Raw sensor data patterns and quality
- Filter comparison and performance
- Comprehensive recommendations for debugging

## Expected Output

### When Working Correctly:
- **ANGLES** should show varying values like: `1.802,-118.835,3.832`
- **ANGLES_DI** should show MotionDI euler angles: `pitch,yaw,roll`
- **ANGLES_SI** should show simple filter: `pitch,yaw,roll`
- **ANGLES_CO** should show complementary filter: `pitch,yaw,roll`
- **ANGLES_FU** should show fused output: `pitch,yaw,roll`
- Raw sensor data should show realistic accelerometer/gyroscope readings
- All angle sources should be consistent and non-zero

### When Broken (Current State):
- **ANGLES** shows: `0,0,0` for all samples
- **ANGLES_DI** may show normal MotionDI calculations
- **ANGLES_SI/CO/FU** may show filter outputs but final result is zero
- Raw sensor data may look normal
- The issue is in the coordinate conversion/fabsf() application

## Fixing the Issue

The fix involves modifying the `_calculateAnglesToReference()` function in `motion_detection.cpp`:

```cpp
// BEFORE (Broken):
yaw = fabsf(yaw);      // ❌ Loses direction
pitch = fabsf(pitch);  // ❌ Loses direction
roll = fabsf(roll);    // ❌ Loses direction

// AFTER (Fixed):
// Keep original signs for proper motion detection
// Only apply clamping if absolutely necessary for your application
if (yaw > 180.0f) yaw = 180.0f;
if (yaw < -180.0f) yaw = -180.0f;
// ... similar for pitch and roll
```

## Dependencies

The tools require Python 3 with these packages:
```bash
pip3 install numpy matplotlib pyserial
```

### For Live Mode:
- Ensure `console_reader.py` is available in `motion_detection_ideas/`
- Serial connection to the motion detection device
- Device should support debug commands (`setdebug`, `printraw`)

## Troubleshooting

### Common Issues:
1. **No data parsed**: Check if log file format matches expected patterns
2. **Import errors**: Install required Python packages
3. **Empty plots**: Verify log file contains the expected data types

### Getting Help:
1. Run the basic visualizer first to see if data is being parsed
2. Use the advanced parser with `--csv` to export data for manual inspection
3. Check the analysis report for specific recommendations

## Next Steps

1. Use these tools to confirm the fabsf() issue in your logs
2. Fix the `_calculateAnglesToReference()` function
3. Re-run the tools to verify the fix
4. Test with real motion to ensure angles are now updating correctly