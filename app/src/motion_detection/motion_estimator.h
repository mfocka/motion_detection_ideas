/*
 ********************************************************************************
 *  ADB Safegate Belgium BV
 *
 *  Motion Estimator - Combined Filter Implementation
 *
 ********************************************************************************
 */

#ifndef MOTION_ESTIMATOR_H
#define MOTION_ESTIMATOR_H

#include <cstdint>
#include <cmath>
#include <cstring>
#include "adb_types.h"
#include "sensor_data.h"

/**
 * @brief Motion Estimator using combined simple and complementary filters
 * 
 * Uses simple filter for azimuth (yaw) and complementary filter for tilt (pitch/roll)
 * Implements smart logic to combine both filters for optimal results
 */
class MotionEstimator
{
public:
    /**
     * @brief Configuration structure for the motion estimator
     */
    struct Config
    {
        float sample_rate_hz = 104.0f;           // Sampling frequency in Hz
        float alpha = 0.98f;                     // Complementary filter gain
        float azimuth_threshold_deg = 10.0f;     // Azimuth change threshold in degrees
        float altitude_threshold_deg = 5.0f;     // Altitude change threshold in degrees
        uint32_t calibration_samples = 500;      // Number of samples for calibration
        float gyro_noise_threshold_dps = 0.1f;  // Gyro noise threshold for bias calculation
    };

    /**
     * @brief Calibration data structure
     */
    struct CalibrationData
    {
        float gyro_bias[3];                     // X, Y, Z gyro bias values
        float acc_bias[3];                      // X, Y, Z accelerometer bias values
        bool is_calibrated = false;
        uint32_t sample_count = 0;
    };

    /**
     * @brief Output structure for motion estimation
     */
    struct Output
    {
        float yaw_deg;                          // Yaw angle in degrees (azimuth)
        float pitch_deg;                        // Pitch angle in degrees (altitude)
        float roll_deg;                         // Roll angle in degrees
        bool has_large_change;                  // True if large angle change detected
        uint64_t timestamp_us;                 // Timestamp of the measurement
    };

    MotionEstimator();
    ~MotionEstimator() = default;

    /**
     * @brief Initialize the motion estimator with configuration
     * @param config Configuration parameters
     * @return true if initialization successful
     */
    bool initialize(const Config& config);

    /**
     * @brief Add calibration sample for bias calculation
     * @param accel Accelerometer data in mg
     * @param gyro Gyroscope data in dps
     * @return true if calibration is complete
     */
    bool addCalibrationSample(const float accel[3], const float gyro[3]);

    /**
     * @brief Update motion estimation with new sensor data
     * @param accel Accelerometer data in mg
     * @param gyro Gyroscope data in dps
     * @param timestamp_us Timestamp in microseconds
     * @return Motion estimation output
     */
    Output update(const float accel[3], const float gyro[3], uint64_t timestamp_us);

    /**
     * @brief Get current calibration status
     * @return Calibration data
     */
    const CalibrationData& getCalibrationData() const { return _calibration; }

    /**
     * @brief Reset calibration data
     */
    void resetCalibration();

    /**
     * @brief Check if estimator is ready (calibrated)
     * @return true if ready
     */
    bool isReady() const { return _calibration.is_calibrated; }


    void updateCompleteEulerAngles(const float other_euler_angles[3], const float trust[3], float resulting_euler_angles[3]);

private:
    // Configuration
    Config _config;
    CalibrationData _calibration;
    
    // Filter state
    float _dt;                                  // Time step in seconds
    float _prev_yaw_deg;                        // Previous yaw angle
    float _prev_pitch_deg;                      // Previous pitch angle
    float _prev_roll_deg;                       // Previous roll angle
    
    // Simple filter state (for yaw/azimuth)
    float _simple_yaw_deg;                      // Simple filter yaw estimate
    float _simple_pitch_deg;                    // Simple filter pitch estimate
    float _simple_roll_deg;                     // Simple filter roll estimate
    
    // Complementary filter state (for tilt)
    float _comp_yaw_deg;                        // Complementary filter yaw estimate
    float _comp_pitch_deg;                      // Complementary filter pitch estimate
    float _comp_roll_deg;                       // Complementary filter roll estimate
    
    // Reference angles for change detection
    float _ref_yaw_deg;                         // Reference yaw angle
    float _ref_pitch_deg;                       // Reference pitch angle
    float _ref_roll_deg;                        // Reference roll angle
    bool _has_reference;                         // True if reference angles are set
    
    // Internal methods
    void _updateSimpleFilter(const float gyro[3]);
    void _updateComplementaryFilter(const float accel[3], const float gyro[3]);
    void _combineFilters(Output& output);
    bool _detectLargeChange(const Output& output);
    void _setReferenceAngles();
    float _tiltAngleFromAccel(const float accel[3], int axis);
    float _normalizeAngle(float angle_deg);

};

#endif // MOTION_ESTIMATOR_H