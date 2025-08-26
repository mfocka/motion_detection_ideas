#include "src/motion_detection/motion_estimator.h"
#include <iostream>
#include <cmath>

// Simple test program for MotionEstimator
int main() {
    std::cout << "MotionEstimator Test Program" << std::endl;
    
    // Initialize with default configuration
    MotionEstimatorConfig config = MOTION_ESTIMATOR_DEFAULT_CONFIG;
    config.threshold_altitude = 3.0f;
    config.threshold_azimuth = 8.0f;
    config.yaw_stability_threshold = 1.5f;
    
    MotionEstimator_Initialize(&config);
    std::cout << "MotionEstimator initialized" << std::endl;
    
    // Simulate some IMU data
    float acc_mg[3] = {0.0f, 0.0f, 1000.0f};  // 1g in Z direction
    float gyro_dps[3] = {0.0f, 0.0f, 0.0f};   // No rotation
    
    // Process a few samples for calibration
    std::cout << "Calibrating..." << std::endl;
    for (int i = 0; i < 100; i++) {
        MotionEvent event;
        MotionEstimator_ProcessData(acc_mg, gyro_dps, i * 10000, &event);
    }
    std::cout << "Calibration complete" << std::endl;
    
    // Simulate a yaw motion event
    std::cout << "Simulating yaw motion..." << std::endl;
    gyro_dps[2] = 45.0f;  // 45 deg/s yaw rotation
    
    for (int i = 0; i < 10; i++) {
        MotionEvent event;
        bool has_event = MotionEstimator_ProcessData(acc_mg, gyro_dps, (100 + i) * 10000, &event);
        
        if (has_event) {
            std::cout << "Motion event detected!" << std::endl;
            if (event.yaw_event) {
                std::cout << "  Yaw change: " << event.yaw_angle << " degrees" << std::endl;
            }
            if (event.altitude_event) {
                std::cout << "  Altitude change - Pitch: " << event.pitch_angle 
                         << ", Roll: " << event.roll_angle << " degrees" << std::endl;
            }
        }
    }
    
    // Get current angles
    float yaw, pitch, roll;
    MotionEstimator_GetCurrentAngles(&yaw, &pitch, &roll);
    std::cout << "Current angles - Yaw: " << yaw << ", Pitch: " << pitch 
              << ", Roll: " << roll << " degrees" << std::endl;
    
    std::cout << "Test complete" << std::endl;
    return 0;
}