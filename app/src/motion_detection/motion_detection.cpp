#include "motion_estimator.h"
#include "MotionDetector.hpp"
#include <cstdint>

// Example usage of both MotionDI and MotionEstimator
// This file demonstrates how to integrate both motion detection systems

namespace motion_detection {

// Example function showing how to initialize both systems
void initializeMotionDetection() {
    // Initialize MotionEstimator with custom configuration
    MotionEstimatorConfig config = MOTION_ESTIMATOR_DEFAULT_CONFIG;
    config.threshold_altitude = 3.0f;  // 3 degrees for altitude
    config.threshold_azimuth = 8.0f;   // 8 degrees for azimuth
    config.yaw_stability_threshold = 1.5f; // 1.5 degrees for yaw stability
    
    MotionEstimator_Initialize(&config);
    
    // Initialize MotionDetector (existing system)
    MotionDetector::Config detector_config;
    detector_config.sampleRateHz = 104.0f;
    detector_config.alpha = 0.98f;
    detector_config.thresholdDeg = 5.0f;
    detector_config.windowSeconds = 2.0f;
    
    // Note: MotionDetector would be instantiated as a class object
    // MotionDetector detector(detector_config);
}

// Example function showing how to process data with both systems
bool processMotionData(const std::array<float,3>& acc_mg,
                       const std::array<float,3>& gyro_dps,
                       uint64_t timestamp_us) {
    
    // Process with MotionEstimator
    MotionEvent event;
    bool has_motion_event = MotionEstimator_ProcessData(
        acc_mg.data(), gyro_dps.data(), timestamp_us, &event);
    
    if (has_motion_event) {
        if (event.yaw_event) {
            // Handle yaw motion event
            // event.yaw_angle contains the detected yaw change
        }
        
        if (event.altitude_event) {
            // Handle altitude motion event
            // event.pitch_angle and event.roll_angle contain the changes
        }
    }
    
    // Process with MotionDetector (existing system)
    // bool detector_event = detector.processData(acc_mg, gyro_dps, timestamp_us);
    
    return has_motion_event;
}

// Example function showing how to get current angles
void getCurrentMotionState(float* yaw, float* pitch, float* roll) {
    MotionEstimator_GetCurrentAngles(yaw, pitch, roll);
}

} // namespace motion_detection