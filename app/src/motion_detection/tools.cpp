/*
 ********************************************************************************
 *  ADB Safegate Belgium BV
 *
 *  Motion Detection Tools - Common Utilities Implementation
 *
 ********************************************************************************
 */

#include "tools.h"
#include <cstring>
#include <cmath>

namespace MotionTools {

void convertWDStoENU(const float wds[3], float enu[3])
{
    // Transformation matrix WDS -> ENU
    // WDS: X-West, Y-Down, Z-South
    // ENU: X-East, Y-North, Z-Up
    enu[0] = -wds[0];  // East = -West
    enu[1] = -wds[2];  // North = -South  
    enu[2] = -wds[1];  // Up = -Down
}

void convertENUtoWDS(const float enu[3], float wds[3])
{
    // Transformation matrix ENU -> WDS (inverse of WDS->ENU)
    // ENU: X-East, Y-North, Z-Up
    // WDS: X-West, Y-Down, Z-South
    wds[0] = -enu[0];  // West = -East
    wds[1] = -enu[2];  // Down = -Up
    wds[2] = -enu[1];  // South = -North
}

void eulerToHorizontal(const float euler_angles[3], float horizontal_coords[3])
{
    // Map Euler angles to horizontal coordinates
    // EulerAngles: YAW=0, PITCH=1, ROLL=2
    // HorizontalCoordinatesMapping: AZIMUTH=0, ALTITUDE=1, ZENITH=2
    horizontal_coords[0] = euler_angles[0]; // AZIMUTH = YAW
    horizontal_coords[1] = euler_angles[1]; // ALTITUDE = PITCH  
    horizontal_coords[2] = euler_angles[2]; // ZENITH = ROLL
}

void horizontalToEuler(const float horizontal_coords[3], float euler_angles[3])
{
    // Map horizontal coordinates to Euler angles (inverse mapping)
    euler_angles[0] = horizontal_coords[0]; // YAW = AZIMUTH
    euler_angles[1] = horizontal_coords[1]; // PITCH = ALTITUDE
    euler_angles[2] = horizontal_coords[2]; // ROLL = ZENITH
}

float normalizeAngle180(float angle_deg)
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

float normalizeAngle360(float angle_deg)
{
    // Normalize angle to [0, 360] range
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float absoluteAngle(float angle_deg)
{
    // For motion detection, use absolute values to avoid wrap-around jumps
    // This prevents -180/+180 discontinuities that cause false motion events
    return fabsf(angle_deg);
}

float clampAngle(float angle_deg, float min_deg, float max_deg)
{
    if (angle_deg < min_deg) return min_deg;
    if (angle_deg > max_deg) return max_deg;
    return angle_deg;
}

float angleDifference(float angle1_deg, float angle2_deg)
{
    // Calculate shortest angular difference between two angles
    float diff = angle1_deg - angle2_deg;
    return normalizeAngle180(diff);
}

void mgToG(const float accel_mg[3], float accel_g[3])
{
    accel_g[0] = accel_mg[0] / 1000.0f;
    accel_g[1] = accel_mg[1] / 1000.0f;
    accel_g[2] = accel_mg[2] / 1000.0f;
}

void gToMg(const float accel_g[3], float accel_mg[3])
{
    accel_mg[0] = accel_g[0] * 1000.0f;
    accel_mg[1] = accel_g[1] * 1000.0f;
    accel_mg[2] = accel_g[2] * 1000.0f;
}

bool isValidSensorData(const float accel_mg[3], const float gyro_dps[3], 
                      float max_accel_mg, float max_gyro_dps)
{
    // Check for NaN values
    for (int i = 0; i < 3; i++) {
        if (isnan(accel_mg[i]) || isnan(gyro_dps[i])) {
            return false;
        }
        
        // Check bounds
        if (fabsf(accel_mg[i]) > max_accel_mg || fabsf(gyro_dps[i]) > max_gyro_dps) {
            return false;
        }
    }
    
    return true;
}

float vectorMagnitude(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

bool normalizeVector(float vector[3])
{
    float magnitude = vectorMagnitude(vector);
    
    // Check for zero vector
    if (magnitude < 1e-6f) {
        return false;
    }
    
    vector[0] /= magnitude;
    vector[1] /= magnitude;
    vector[2] /= magnitude;
    
    return true;
}

float lerp(float a, float b, float t)
{
    // Clamp t to [0, 1] range
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    return a + t * (b - a);
}

float lowPassFilter(float current_value, float previous_filtered, float alpha)
{
    // Clamp alpha to [0, 1] range
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    return alpha * current_value + (1.0f - alpha) * previous_filtered;
}

} // namespace MotionTools