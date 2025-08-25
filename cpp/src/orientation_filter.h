#pragma once
#include <array>
#include <vector>

struct EulerRPY {
    double roll;
    double pitch;
    double yaw;
};

class ComplementaryFilter {
public:
    ComplementaryFilter(double gain, double fs_hz);
    void reset();
    // acc: m/s^2, gyro: rad/s
    EulerRPY update(const std::array<double,3>& acc, const std::array<double,3>& gyro);

private:
    double gain_;
    double dt_;
    EulerRPY euler_ {0.0,0.0,0.0};
};

