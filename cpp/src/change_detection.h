#pragma once
#include <vector>
#include <cstddef>
#include "orientation_filter.h"

struct OrientationEventC {
    std::size_t index;
    double timestamp_s;
    double delta_roll_deg;
    double delta_pitch_deg;
    double delta_yaw_deg;
    bool persistent;
};

std::vector<OrientationEventC> detect_changes(
    const std::vector<EulerRPY>& euler,
    double az_thresh_deg,
    double alt_thresh_deg,
    double T_detection_s,
    double T_validation_s,
    double fs_hz,
    double smooth_window_s
);

