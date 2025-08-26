# MotionEstimator

A hybrid motion detection system that combines simple filtering for yaw (azimuth) with complementary filtering for all axes to achieve optimal drift resistance and motion detection.

## Overview

The MotionEstimator implements the approach that showed excellent results in testing:
- **Simple filter for yaw (azimuth)**: Direct gyro integration with periodic resets to prevent drift accumulation
- **Complementary filter for all axes**: Including yaw, pitch, and roll for drift detection and validation

## Key Features

- **Hybrid filtering approach**: Combines the best of both filtering methods
- **Drift detection**: Uses complementary filter yaw stability to validate simple filter readings
- **Configurable thresholds**: Set separate thresholds for altitude and azimuth changes
- **Calibration support**: Automatic gyro bias estimation during initialization
- **Reset capabilities**: Can reset filters to prevent drift accumulation
- **Event detection**: Returns angle changes when thresholds are exceeded

## Usage

### Initialization

```cpp
#include "motion_estimator.h"

// Use default configuration
MotionEstimator_Initialize(nullptr);

// Or customize configuration
MotionEstimatorConfig config = MOTION_ESTIMATOR_DEFAULT_CONFIG;
config.threshold_altitude = 3.0f;      // 3 degrees for altitude
config.threshold_azimuth = 8.0f;       // 8 degrees for azimuth
config.yaw_stability_threshold = 1.5f; // 1.5 degrees for yaw stability
config.calibration_samples = 100;      // 100 samples for calibration

MotionEstimator_Initialize(&config);
```

### Processing Data

```cpp
float acc_mg[3] = {0.0f, 0.0f, 1000.0f};  // Accelerometer data in mg
float gyro_dps[3] = {0.0f, 0.0f, 0.0f};   // Gyroscope data in deg/s
uint64_t timestamp_us = 1000000;            // Timestamp in microseconds

MotionEvent event;
bool has_motion = MotionEstimator_ProcessData(acc_mg, gyro_dps, timestamp_us, &event);

if (has_motion) {
    if (event.yaw_event) {
        printf("Yaw change: %.2f degrees\n", event.yaw_angle);
    }
    if (event.altitude_event) {
        printf("Altitude change - Pitch: %.2f, Roll: %.2f degrees\n", 
               event.pitch_angle, event.roll_angle);
    }
}
```

### Getting Current Angles

```cpp
float yaw, pitch, roll;
MotionEstimator_GetCurrentAngles(&yaw, &pitch, &roll);
printf("Current: Yaw=%.2f, Pitch=%.2f, Roll=%.2f degrees\n", yaw, pitch, roll);
```

### Resetting Filters

```cpp
// Reset complementary filter to current accelerometer readings
MotionEstimator_ResetComplementaryFilter();

// Reset simple yaw filter to current complementary filter value
MotionEstimator_ResetSimpleYawFilter();
```

### Recalibration

```cpp
float gyro_sample[3] = {0.1f, -0.05f, 0.02f}; // New calibration sample
MotionEstimator_Recalibrate(gyro_sample);
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sample_rate_hz` | 104.0f | IMU sample rate in Hz |
| `alpha` | 0.98f | Complementary filter weight (0.0 to 1.0) |
| `threshold_altitude` | 5.0f | Altitude change threshold in degrees |
| `threshold_azimuth` | 10.0f | Azimuth change threshold in degrees |
| `yaw_stability_threshold` | 2.0f | Yaw stability threshold for drift detection |
| `calibration_samples` | 100 | Number of samples for gyro bias calibration |
| `reset_threshold_deg` | 15.0f | Threshold for yaw reset in simple filter |

## How It Works

### 1. Calibration Phase
- Collects initial gyro samples to estimate bias
- No motion events are generated during calibration

### 2. Hybrid Filtering
- **Simple Filter (Yaw)**: Direct integration of gyro Z-axis with reset mechanism
- **Complementary Filter (All Axes)**: Fuses gyro integration with accelerometer tilt angles

### 3. Event Detection
- **Yaw Events**: Only generated when complementary filter yaw is stable (no drift)
- **Altitude Events**: Generated when pitch or roll changes exceed threshold

### 4. Drift Prevention
- Uses complementary filter yaw stability as a "drift detector"
- If complementary filter yaw is stable, trusts simple filter readings
- If complementary filter yaw is drifting, doesn't send yaw data

## Integration with Existing Code

The MotionEstimator can be used alongside the existing MotionDetector:

```cpp
#include "motion_estimator.h"
#include "MotionDetector.hpp"

// Initialize both systems
MotionEstimatorConfig config = MOTION_ESTIMATOR_DEFAULT_CONFIG;
MotionEstimator_Initialize(&config);

MotionDetector::Config detector_config;
detector_config.sampleRateHz = 104.0f;
MotionDetector detector(detector_config);

// Process data with both systems
MotionEvent event;
bool has_event = MotionEstimator_ProcessData(acc_mg, gyro_dps, timestamp_us, &event);
bool detector_event = detector.processData(acc_mg, gyro_dps, timestamp_us);
```

## Building

The MotionEstimator is implemented in C++ with C-style interface for easy integration. Simply include the header and link the implementation file in your build system.

## Testing

A test program is provided in `test_motion_estimator.cpp` that demonstrates basic functionality and can be used to verify the implementation.