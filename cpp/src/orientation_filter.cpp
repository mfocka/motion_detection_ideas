#include "orientation_filter.h"
#include <cmath>

static inline double clamp(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }

ComplementaryFilter::ComplementaryFilter(double gain, double fs_hz)
    : gain_(gain), dt_(1.0/fs_hz) {}

void ComplementaryFilter::reset(){ euler_ = {0.0,0.0,0.0}; }

EulerRPY ComplementaryFilter::update(const std::array<double,3>& acc, const std::array<double,3>& gyro){
    // Integrate gyro
    euler_.roll  += gyro[0] * dt_;
    euler_.pitch += gyro[1] * dt_;
    euler_.yaw   += gyro[2] * dt_;

    // Accelerometer inclination (roll,pitch) from gravity vector
    double ax = acc[0], ay = acc[1], az = acc[2];
    double pitch_acc = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    double roll_acc  = std::atan2( ay, az );

    // Complementary blend for roll/pitch
    euler_.roll  = (1.0 - gain_) * euler_.roll  + gain_ * roll_acc;
    euler_.pitch = (1.0 - gain_) * euler_.pitch + gain_ * pitch_acc;
    // yaw left as integrated gyro; without magnetometer it drifts

    return euler_;
}

