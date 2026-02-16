/*
 ********************************************************************************
 *  ADB Safegate Belgium BV
 *
 *  Motion Detection Tools - Common Utilities
 *
 ********************************************************************************
 */

#ifndef TOOLS_H
#define TOOLS_H

#include <cmath>

// Mathematical constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

// Conversion constants
static constexpr float DEG_TO_RAD = M_PI / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / M_PI;
static constexpr float G_TO_MS2 = 9.80665f;
static constexpr float MG_TO_MS2 = G_TO_MS2 / 1000.0f;

/**
 * @brief Motion detection coordinate system tools
 */
namespace MotionTools {

    /**
     * @brief Coordinate system conversion from WDS to ENU
     * @param wds Input in WDS coordinates (X-West, Y-Down, Z-South)  
     * @param enu Output in ENU coordinates (X-East, Y-North, Z-Up)
     */
    void convertWDStoENU(const float wds[3], float enu[3]);

    /**
     * @brief Coordinate system conversion from ENU to WDS
     * @param enu Input in ENU coordinates (X-East, Y-North, Z-Up)
     * @param wds Output in WDS coordinates (X-West, Y-Down, Z-South)
     */
    void convertENUtoWDS(const float enu[3], float wds[3]);

    /**
     * @brief Convert Euler angles to horizontal coordinates
     * @param euler_angles Input [yaw, pitch, roll] in degrees
     * @param horizontal_coords Output [azimuth, altitude, zenith] in degrees
     */
    void eulerToHorizontal(const float euler_angles[3], float horizontal_coords[3]);

    /**
     * @brief Convert horizontal coordinates to Euler angles
     * @param horizontal_coords Input [azimuth, altitude, zenith] in degrees
     * @param euler_angles Output [yaw, pitch, roll] in degrees
     */
    void horizontalToEuler(const float horizontal_coords[3], float euler_angles[3]);

    /**
     * @brief Normalize angle to [-180, 180] range
     * @param angle_deg Angle in degrees
     * @return Normalized angle in degrees
     */
    float normalizeAngle180(float angle_deg);

    /**
     * @brief Normalize angle to [0, 360] range
     * @param angle_deg Angle in degrees
     * @return Normalized angle in degrees
     */
    float normalizeAngle360(float angle_deg);

    /**
     * @brief Get absolute angle (for motion detection)
     * @param angle_deg Angle in degrees
     * @return Absolute angle in degrees
     */
    float absoluteAngle(float angle_deg);

    /**
     * @brief Clamp angle to specified range
     * @param angle_deg Angle in degrees
     * @param min_deg Minimum angle in degrees
     * @param max_deg Maximum angle in degrees
     * @return Clamped angle in degrees
     */
    float clampAngle(float angle_deg, float min_deg, float max_deg);

    /**
     * @brief Calculate angular difference between two angles
     * @param angle1_deg First angle in degrees
     * @param angle2_deg Second angle in degrees
     * @return Shortest angular difference in degrees
     */
    float angleDifference(float angle1_deg, float angle2_deg);

    /**
     * @brief Convert accelerometer data from mg to g
     * @param accel_mg Input acceleration in mg
     * @param accel_g Output acceleration in g
     */
    void mgToG(const float accel_mg[3], float accel_g[3]);

    /**
     * @brief Convert accelerometer data from g to mg
     * @param accel_g Input acceleration in g
     * @param accel_mg Output acceleration in mg
     */
    void gToMg(const float accel_g[3], float accel_mg[3]);

    /**
     * @brief Check if sensor data is valid (no NaN, within reasonable bounds)
     * @param accel_mg Accelerometer data in mg
     * @param gyro_dps Gyroscope data in dps
     * @param max_accel_mg Maximum valid acceleration in mg (default: 2000mg = 2g)
     * @param max_gyro_dps Maximum valid gyro rate in dps (default: 500 dps)
     * @return true if data is valid
     */
    bool isValidSensorData(const float accel_mg[3], const float gyro_dps[3], 
                          float max_accel_mg = 2000.0f, float max_gyro_dps = 500.0f);

    /**
     * @brief Calculate vector magnitude
     * @param vector 3D vector
     * @return Magnitude of the vector
     */
    float vectorMagnitude(const float vector[3]);

    /**
     * @brief Normalize a 3D vector
     * @param vector Input/output 3D vector to normalize
     * @return true if normalization successful (non-zero vector)
     */
    bool normalizeVector(float vector[3]);

    /**
     * @brief Linear interpolation between two values
     * @param a First value
     * @param b Second value
     * @param t Interpolation factor [0.0, 1.0]
     * @return Interpolated value
     */
    float lerp(float a, float b, float t);

    /**
     * @brief Apply low-pass filter (simple exponential moving average)
     * @param current_value Current measurement
     * @param previous_filtered Previous filtered value
     * @param alpha Filter coefficient [0.0, 1.0] (higher = less filtering)
     * @return Filtered value
     */
    float lowPassFilter(float current_value, float previous_filtered, float alpha);

} // namespace MotionTools

#endif // TOOLS_H