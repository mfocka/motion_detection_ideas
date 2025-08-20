#pragma once
#include <array>
#include <deque>
#include <cstdint>

namespace motion_detection {

struct Config {
    float sampleRateHz      = 104.0f;   // IMU ODR
    float alpha             = 0.98f;    // complementary filter weight
    float thresholdDeg      = 5.0f;     // detection threshold
    float windowSeconds     = 2.0f;     // baseline window
};

class MotionDetector {
public:
    MotionDetector() = default;
    explicit MotionDetector(const Config& cfg) { init(cfg); }

    // Re-initialise internal state (called once at sensor thread start).
    void init(const Config& cfg);

    // Feed a new sample.  Returns true when an abrupt orientation change is detected.
    //   acc_mg   – accelerometer reading in milli-g (mg)
    //   gyro_dps – gyroscope reading in deg/s
    //   timestamp_us – monotonic timestamp of the sample (microseconds)
    bool processData(const std::array<float,3>& acc_mg,
                     const std::array<float,3>& gyro_dps,
                     std::uint64_t timestamp_us);

    // Accessors
    float pitch() const { return m_pitch; }
    float roll()  const { return m_roll;  }

private:
    Config m_cfg{};
    float  m_dt        = 1.0f/104.0f;

    // Orientation state
    float m_pitch = 0.f;
    float m_roll  = 0.f;

    // Sliding window buffers for baseline
    std::deque<float> m_pitchBuf;
    std::deque<float> m_rollBuf;
    float m_pitchMean = 0.f;
    float m_rollMean  = 0.f;

    // Helper computations
    static void computeTiltAngles(const std::array<float,3>& acc_mg, float& pitchDeg, float& rollDeg);
};

} // namespace motion_detection
