#include "motion_estimator.h"
#include <cmath>
#include <cstring>

// Constants
static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
static constexpr float G_TO_MS2 = 9.80665f;
static constexpr float MG_TO_MS2 = G_TO_MS2 / 1000.0f;

// Internal state structure
struct MotionEstimatorState {
    MotionEstimatorConfig config;
    float dt;                    // Time step in seconds
    
    // Current orientation estimates
    float yaw_simple;            // Simple filter yaw (degrees)
    float yaw_complementary;     // Complementary filter yaw (degrees)
    float pitch;                 // Pitch angle (degrees)
    float roll;                  // Roll angle (degrees)
    
    // Previous values for change detection
    float prev_yaw_simple;
    float prev_yaw_complementary;
    float prev_pitch;
    float prev_roll;
    
    // Gyro bias estimates
    float gyro_bias[3];         // Gyro bias in deg/s
    
    // Calibration state
    int calibration_count;
    float calibration_sum[3];    // Sum of gyro readings during calibration
    bool is_calibrated;
    
    // Event detection state
    bool yaw_event_active;
    bool altitude_event_active;
    int event_duration_count;
    
    // Reset state
    bool needs_yaw_reset;
    float reset_yaw_value;
};

// Global instance
static MotionEstimatorState g_state;

// Helper functions
static void computeTiltAngles(const float acc_mg[3], float& pitch_deg, float& roll_deg) {
    // Convert mg to g
    const float ax = acc_mg[0] / 1000.0f;
    const float ay = acc_mg[1] / 1000.0f;
    const float az = acc_mg[2] / 1000.0f;
    
    pitch_deg = RAD_TO_DEG * std::atan2(-ax, std::sqrt(ay*ay + az*az));
    roll_deg = RAD_TO_DEG * std::atan2(ay, az);
}

static float normalizeAngle(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static float angleDifference(float angle1_deg, float angle2_deg) {
    float diff = normalizeAngle(angle1_deg - angle2_deg);
    return std::abs(diff);
}

void MotionEstimator_Initialize(const MotionEstimatorConfig* config) {
    if (config) {
        g_state.config = *config;
    } else {
        g_state.config = MOTION_ESTIMATOR_DEFAULT_CONFIG;
    }
    
    g_state.dt = 1.0f / g_state.config.sample_rate_hz;
    
    // Initialize orientation to zero
    g_state.yaw_simple = 0.0f;
    g_state.yaw_complementary = 0.0f;
    g_state.pitch = 0.0f;
    g_state.roll = 0.0f;
    
    g_state.prev_yaw_simple = 0.0f;
    g_state.prev_yaw_complementary = 0.0f;
    g_state.prev_pitch = 0.0f;
    g_state.prev_roll = 0.0f;
    
    // Initialize gyro bias
    std::memset(g_state.gyro_bias, 0, sizeof(g_state.gyro_bias));
    
    // Initialize calibration state
    g_state.calibration_count = 0;
    std::memset(g_state.calibration_sum, 0, sizeof(g_state.calibration_sum));
    g_state.is_calibrated = false;
    
    // Initialize event detection state
    g_state.yaw_event_active = false;
    g_state.altitude_event_active = false;
    g_state.event_duration_count = 0;
    
    // Initialize reset state
    g_state.needs_yaw_reset = false;
    g_state.reset_yaw_value = 0.0f;
}

bool MotionEstimator_ProcessData(const float acc_mg[3], const float gyro_dps[3], 
                                uint64_t timestamp_us, MotionEvent* event) {
    if (!event) return false;
    
    // Initialize event
    std::memset(event, 0, sizeof(MotionEvent));
    
    // Handle calibration if not yet calibrated
    if (!g_state.is_calibrated) {
        if (g_state.calibration_count < g_state.config.calibration_samples) {
            for (int i = 0; i < 3; i++) {
                g_state.calibration_sum[i] += gyro_dps[i];
            }
            g_state.calibration_count++;
            
            if (g_state.calibration_count == g_state.config.calibration_samples) {
                // Calculate bias
                for (int i = 0; i < 3; i++) {
                    g_state.gyro_bias[i] = g_state.calibration_sum[i] / g_state.config.calibration_samples;
                }
                g_state.is_calibrated = true;
            }
            return false; // No events during calibration
        }
    }
    
    // Remove gyro bias
    float gyro_corrected[3];
    for (int i = 0; i < 3; i++) {
        gyro_corrected[i] = gyro_dps[i] - g_state.gyro_bias[i];
    }
    
    // Store previous values
    g_state.prev_yaw_simple = g_state.yaw_simple;
    g_state.prev_yaw_complementary = g_state.yaw_complementary;
    g_state.prev_pitch = g_state.pitch;
    g_state.prev_roll = g_state.roll;
    
    // 1. Simple filter for yaw (direct integration with reset)
    float yaw_rate = gyro_corrected[2]; // Z-axis for yaw
    g_state.yaw_simple += yaw_rate * g_state.dt;
    g_state.yaw_simple = normalizeAngle(g_state.yaw_simple);
    
    // Check if yaw reset is needed
    if (std::abs(yaw_rate) > g_state.config.reset_threshold_deg) {
        g_state.needs_yaw_reset = true;
        g_state.reset_yaw_value = g_state.yaw_simple;
    }
    
    // 2. Complementary filter for all axes
    float pitch_acc, roll_acc;
    computeTiltAngles(acc_mg, pitch_acc, roll_acc);
    
    // Integrate gyro
    float yaw_gyro = g_state.yaw_complementary + gyro_corrected[2] * g_state.dt;
    float pitch_gyro = g_state.pitch + gyro_corrected[1] * g_state.dt;
    float roll_gyro = g_state.roll + gyro_corrected[0] * g_state.dt;
    
    // Fuse with complementary filter
    const float alpha = g_state.config.alpha;
    g_state.yaw_complementary = alpha * yaw_gyro + (1.0f - alpha) * g_state.yaw_complementary;
    g_state.pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    g_state.roll = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    
    // Normalize angles
    g_state.yaw_complementary = normalizeAngle(g_state.yaw_complementary);
    g_state.pitch = normalizeAngle(g_state.pitch);
    g_state.roll = normalizeAngle(g_state.roll);
    
    // 3. Event detection
    bool has_event = false;
    
    // Check yaw stability in complementary filter
    float yaw_complementary_change = angleDifference(g_state.yaw_complementary, g_state.prev_yaw_complementary);
    bool yaw_stable = yaw_complementary_change < g_state.config.yaw_stability_threshold;
    
    // Yaw event detection
    if (yaw_stable) {
        float yaw_simple_change = angleDifference(g_state.yaw_simple, g_state.prev_yaw_simple);
        if (yaw_simple_change > g_state.config.threshold_azimuth) {
            event->yaw_event = true;
            event->yaw_angle = yaw_simple_change;
            has_event = true;
        }
    } else {
        // Yaw is drifting, don't trust simple filter
        event->yaw_angle = 0.0f;
    }
    
    // Altitude event detection (pitch and roll)
    float pitch_change = std::abs(g_state.pitch - g_state.prev_pitch);
    float roll_change = std::abs(g_state.roll - g_state.prev_roll);
    
    if (pitch_change > g_state.config.threshold_altitude || 
        roll_change > g_state.config.threshold_altitude) {
        event->altitude_event = true;
        event->pitch_angle = pitch_change;
        event->roll_angle = roll_change;
        has_event = true;
    }
    
    return has_event;
}

void MotionEstimator_GetCurrentAngles(float* yaw_deg, float* pitch_deg, float* roll_deg) {
    if (yaw_deg) *yaw_deg = g_state.yaw_simple; // Use simple filter for yaw
    if (pitch_deg) *pitch_deg = g_state.pitch;
    if (roll_deg) *roll_deg = g_state.roll;
}

void MotionEstimator_ResetComplementaryFilter(void) {
    // Reset complementary filter to current accelerometer readings
    // This will be done on next ProcessData call
    g_state.needs_yaw_reset = true;
}

void MotionEstimator_ResetSimpleYawFilter(void) {
    // Reset simple yaw filter to current complementary filter value
    g_state.yaw_simple = g_state.yaw_complementary;
    g_state.prev_yaw_simple = g_state.yaw_simple;
}

void MotionEstimator_Recalibrate(const float gyro_dps[3]) {
    // Reset calibration state
    g_state.calibration_count = 0;
    std::memset(g_state.calibration_sum, 0, sizeof(g_state.calibration_sum));
    g_state.is_calibrated = false;
    
    // Start new calibration
    for (int i = 0; i < 3; i++) {
        g_state.calibration_sum[i] += gyro_dps[i];
    }
    g_state.calibration_count = 1;
}

void MotionEstimator_UpdateConfig(const MotionEstimatorConfig* config) {
    if (config) {
        g_state.config = *config;
        g_state.dt = 1.0f / g_state.config.sample_rate_hz;
    }
}