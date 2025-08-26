/*
 ********************************************************************************
 *  ADB Safegate Belgium BV
 *
 *  Motion Estimator - Combined Filter Implementation
 *
 ********************************************************************************
 */

#include "motion_estimator.h"
#include <cstring>
#include <cmath>

// Constants
static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float G_TO_MS2 = 9.80665f;
static constexpr float MG_TO_MS2 = G_TO_MS2 / 1000.0f;

MotionEstimator::MotionEstimator()
    : _dt(0.0f)
    , _prev_yaw_deg(0.0f)
    , _prev_pitch_deg(0.0f)
    , _prev_roll_deg(0.0f)
    , _simple_yaw_deg(0.0f)
    , _simple_pitch_deg(0.0f)
    , _simple_roll_deg(0.0f)
    , _comp_yaw_deg(0.0f)
    , _comp_pitch_deg(0.0f)
    , _comp_roll_deg(0.0f)
    , _ref_yaw_deg(0.0f)
    , _ref_pitch_deg(0.0f)
    , _ref_roll_deg(0.0f)
    , _has_reference(false)
{
    // Initialize calibration data
    std::memset(&_calibration, 0, sizeof(_calibration));
}

bool MotionEstimator::initialize(const Config& config)
{
    _config = config;
    _dt = 1.0f / _config.sample_rate_hz;
    
    // Reset all state variables
    _prev_yaw_deg = _prev_pitch_deg = _prev_roll_deg = 0.0f;
    _simple_yaw_deg = _simple_pitch_deg = _simple_roll_deg = 0.0f;
    _comp_yaw_deg = _comp_pitch_deg = _comp_roll_deg = 0.0f;
    _ref_yaw_deg = _ref_pitch_deg = _ref_roll_deg = 0.0f;
    _has_reference = false;
    
    // Reset calibration
    resetCalibration();
    
    return true;
}

bool MotionEstimator::addCalibrationSample(const float accel[3], const float gyro[3])
{
    if (_calibration.is_calibrated) {
        return true; // Already calibrated
    }
    
    // Add to running sums
    for (int i = 0; i < 3; i++) {
        _calibration.acc_bias[i] += accel[i];
        _calibration.gyro_bias[i] += gyro[i];
    }
    _calibration.sample_count++;
    
    // Check if we have enough samples
    if (_calibration.sample_count >= _config.calibration_samples) {
        // Calculate averages
        for (int i = 0; i < 3; i++) {
            _calibration.acc_bias[i] /= _calibration.sample_count;
            _calibration.gyro_bias[i] /= _calibration.sample_count;
        }
        
        // Check if gyro bias is reasonable (not too noisy)
        float gyro_magnitude = std::sqrt(
            _calibration.gyro_bias[0] * _calibration.gyro_bias[0] +
            _calibration.gyro_bias[1] * _calibration.gyro_bias[1] +
            _calibration.gyro_bias[2] * _calibration.gyro_bias[2]
        );
        
        if (gyro_magnitude < _config.gyro_noise_threshold_dps) {
            _calibration.is_calibrated = true;
            return true;
        } else {
            // Reset and try again
            resetCalibration();
            return false;
        }
    }
    
    return false;
}

MotionEstimator::Output MotionEstimator::update(const float accel[3], const float gyro[3], uint64_t timestamp_us)
{
    Output output;
    output.timestamp_us = timestamp_us;
    
    if (!_calibration.is_calibrated) {
        // Return zero angles if not calibrated
        output.yaw_deg = output.pitch_deg = output.roll_deg = 0.0f;
        output.has_large_change = false;
        return output;
    }
    
    // Apply bias correction
    float accel_corrected[3], gyro_corrected[3];
    for (int i = 0; i < 3; i++) {
        accel_corrected[i] = accel[i] - _calibration.acc_bias[i];
        gyro_corrected[i] = gyro[i] - _calibration.gyro_bias[i];
    }
    
    // Update both filters
    _updateSimpleFilter(gyro_corrected);
    _updateComplementaryFilter(accel_corrected, gyro_corrected);
    
    // Combine filters using smart logic
    _combineFilters(output);
    
    // Detect large changes
    output.has_large_change = _detectLargeChange(output);
    
    // Set reference angles if not set yet
    if (!_has_reference) {
        _setReferenceAngles();
    }
    
    // Update previous angles
    _prev_yaw_deg = output.yaw_deg;
    _prev_pitch_deg = output.pitch_deg;
    _prev_roll_deg = output.roll_deg;
    
    return output;
}

void MotionEstimator::resetCalibration()
{
    std::memset(&_calibration, 0, sizeof(_calibration));
    _calibration.is_calibrated = false;
    _calibration.sample_count = 0;
}

void MotionEstimator::_updateSimpleFilter(const float gyro[3])
{
    // Simple integration filter for all axes
    // Yaw: integrate around Z-axis (gyro[2])
    _simple_yaw_deg = _prev_yaw_deg + gyro[2] * _dt;
    
    // Pitch: integrate around Y-axis (gyro[1]) 
    _simple_pitch_deg = _prev_pitch_deg + gyro[1] * _dt;
    
    // Roll: integrate around X-axis (gyro[0])
    _simple_roll_deg = _prev_roll_deg + gyro[0] * _dt;
    
    // Normalize angles to [-180, 180] range
    _simple_yaw_deg = _normalizeAngle(_simple_yaw_deg);
    _simple_pitch_deg = _normalizeAngle(_simple_pitch_deg);
    _simple_roll_deg = _normalizeAngle(_simple_roll_deg);
}

void MotionEstimator::_updateComplementaryFilter(const float accel[3], const float gyro[3])
{
    // Get tilt angles from accelerometer
    float acc_pitch_deg = _tiltAngleFromAccel(accel, 1); // Pitch from Y-axis
    float acc_roll_deg = _tiltAngleFromAccel(accel, 0);  // Roll from X-axis
    
    // Gyro integration
    float gyro_pitch_deg = _prev_pitch_deg + gyro[1] * _dt;
    float gyro_roll_deg = _prev_roll_deg + gyro[0] * _dt;
    float gyro_yaw_deg = _prev_yaw_deg + gyro[2] * _dt;
    
    // Complementary filter fusion
    _comp_pitch_deg = _config.alpha * gyro_pitch_deg + (1.0f - _config.alpha) * acc_pitch_deg;
    _comp_roll_deg = _config.alpha * gyro_roll_deg + (1.0f - _config.alpha) * acc_roll_deg;
    _comp_yaw_deg = gyro_yaw_deg; // Yaw only from gyro (no accelerometer reference)
    
    // Normalize angles
    _comp_yaw_deg = _normalizeAngle(_comp_yaw_deg);
    _comp_pitch_deg = _normalizeAngle(_comp_pitch_deg);
    _comp_roll_deg = _normalizeAngle(_comp_roll_deg);
}

void MotionEstimator::_combineFilters(Output& output)
{
    // Smart combination logic based on your testing results:
    // - Azimuth (yaw): Simple filter works better but can be noisy
    // - Tilt (pitch/roll): Complementary filter has less drift
    
    // Yaw: Use simple filter as primary, but validate with complementary filter
    output.yaw_deg = _simple_yaw_deg;
    
    // Check if complementary filter shows significant change
    float yaw_diff = std::abs(_comp_yaw_deg - _prev_yaw_deg);
    if (yaw_diff < 1.0f) { // Small change threshold
        // If complementary filter shows small change, trust simple filter
        output.yaw_deg = _simple_yaw_deg;
    } else {
        // If complementary filter shows large change, use weighted average
        output.yaw_deg = 0.7f * _simple_yaw_deg + 0.3f * _comp_yaw_deg;
    }
    
    // Pitch and Roll: Use complementary filter as primary (less drift)
    output.pitch_deg = _comp_pitch_deg;
    output.roll_deg = _comp_roll_deg;
    
    // Normalize final angles
    output.yaw_deg = _normalizeAngle(output.yaw_deg);
    output.pitch_deg = _normalizeAngle(output.pitch_deg);
    output.roll_deg = _normalizeAngle(output.roll_deg);
}

bool MotionEstimator::_detectLargeChange(const Output& output)
{
    if (!_has_reference) {
        return false;
    }
    
    // Check if any angle exceeds threshold
    float yaw_change = std::abs(output.yaw_deg - _ref_yaw_deg);
    float pitch_change = std::abs(output.pitch_deg - _ref_pitch_deg);
    float roll_change = std::abs(output.roll_deg - _ref_roll_deg);
    
    return (yaw_change > _config.azimuth_threshold_deg ||
            pitch_change > _config.altitude_threshold_deg ||
            roll_change > _config.altitude_threshold_deg);
}

void MotionEstimator::_setReferenceAngles()
{
    _ref_yaw_deg = _prev_yaw_deg;
    _ref_pitch_deg = _prev_pitch_deg;
    _ref_roll_deg = _prev_roll_deg;
    _has_reference = true;
}

void MotionEstimator::_convertWDStoENU(const float wds[3], float enu[3])
{
    // Transformation matrix WDS -> ENU
    // WDS: X-West, Y-Down, Z-South
    // ENU: X-East, Y-North, Z-Up
    enu[0] = -wds[0];  // East = -West
    enu[1] = -wds[2];  // North = -South
    enu[2] = -wds[1];  // Up = -Down
}

float MotionEstimator::_tiltAngleFromAccel(const float accel[3], int axis)
{
    // Convert mg to m/s²
    float ax = accel[0] * MG_TO_MS2;
    float ay = accel[1] * MG_TO_MS2;
    float az = accel[2] * MG_TO_MS2;
    
    float angle_rad;
    if (axis == 0) { // Roll (around X-axis)
        angle_rad = std::atan2(ay, az);
    } else if (axis == 1) { // Pitch (around Y-axis)
        angle_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    } else { // Invalid axis
        return 0.0f;
    }
    
    return angle_rad * RAD_TO_DEG;
}

float MotionEstimator::_normalizeAngle(float angle_deg)
{
    // Normalize angle to [-180, 180] range
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}