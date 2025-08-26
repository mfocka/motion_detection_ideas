#pragma once

#include <cstdint>
#include <cstdbool>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration structure for MotionEstimator
typedef struct {
    float sample_rate_hz;        // IMU sample rate in Hz
    float alpha;                 // Complementary filter weight (0.0 to 1.0)
    float threshold_altitude;    // Altitude change threshold in degrees
    float threshold_azimuth;     // Azimuth change threshold in degrees
    float yaw_stability_threshold; // Yaw stability threshold for drift detection
    int calibration_samples;     // Number of samples to use for gyro bias calibration
    float reset_threshold_deg;   // Threshold for yaw reset in simple filter
} MotionEstimatorConfig;

typedef struct {
    float euler_angles_simple[3];
    float euler_angles_complementary[3];
    float euler_angles_fused[3];
} ME_output_t;

// Default configuration values
#define MOTION_ESTIMATOR_DEFAULT_CONFIG { \
    .sample_rate_hz = 104.0f, \
    .alpha = 0.98f, \
    .threshold_altitude = 5.0f, \
    .threshold_azimuth = 10.0f, \
    .yaw_stability_threshold = 2.0f, \
    .calibration_samples = 100, \
    .reset_threshold_deg = 15.0f \
}

// Motion event structure
typedef struct {
    float yaw_angle;            // Yaw angle change in degrees (0 if no significant change)
    float pitch_angle;          // Pitch angle change in degrees (0 if no significant change)
    float roll_angle;           // Roll angle change in degrees (0 if no significant change)
    bool yaw_event;             // True if significant yaw change detected
    bool altitude_event;        // True if significant altitude change detected
} MotionEvent;

// Initialize the MotionEstimator with configuration
void MotionEstimator_Initialize(const MotionEstimatorConfig* config);

// Process new IMU data and return motion events
// Returns true if any significant motion is detected
bool MotionEstimator_ProcessData(const float acc_mg[3], const float gyro_dps[3], 
                                uint64_t timestamp_us, MotionEvent* event);

// Get current orientation estimates
void MotionEstimator_GetCurrentAngles(float* yaw_deg, float* pitch_deg, float* roll_deg);

// Reset the complementary filter to current accelerometer readings
void MotionEstimator_ResetComplementaryFilter(void);

// Reset the simple yaw filter to current value
void MotionEstimator_ResetSimpleYawFilter(void);

// Recalibrate gyro bias using new calibration samples
void MotionEstimator_Recalibrate(const float gyro_dps[3]);

// Update configuration parameters
void MotionEstimator_UpdateConfig(const MotionEstimatorConfig* config);

#ifdef __cplusplus
}
#endif