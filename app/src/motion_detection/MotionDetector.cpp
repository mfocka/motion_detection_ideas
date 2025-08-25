#include "MotionDetector.hpp"
#include <cmath>

namespace motion_detection {

static constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
static constexpr float RAD2DEG = 180.0f / 3.14159265358979323846f;

void MotionDetector::init(const Config& cfg)
{
    m_cfg = cfg;
    m_dt  = 1.0f / m_cfg.sampleRateHz;

    m_pitch = m_roll = 0.f;
    m_pitchBuf.clear();
    m_rollBuf.clear();
    m_pitchMean = m_rollMean = 0.f;

    // Pre-size buffers
    std::size_t winSamples = static_cast<std::size_t>(std::round(m_cfg.windowSeconds * m_cfg.sampleRateHz));
    m_pitchBuf = std::deque<float>();
    m_rollBuf  = std::deque<float>();
    m_pitchBuf.resize(0);
    m_rollBuf.resize(0);
    m_pitchBuf.shrink_to_fit();
    m_rollBuf.shrink_to_fit();
}

void MotionDetector::computeTiltAngles(const std::array<float,3>& acc_mg, float& pitchDeg, float& rollDeg)
{
    // Convert mg to g
    const float ax = acc_mg[0] / 1000.0f;
    const float ay = acc_mg[1] / 1000.0f;
    const float az = acc_mg[2] / 1000.0f;

    pitchDeg = RAD2DEG * std::atan2(-ax, std::sqrt(ay*ay + az*az));
    rollDeg  = RAD2DEG * std::atan2( ay, az );
}

bool MotionDetector::processData(const std::array<float,3>& acc_mg,
                                 const std::array<float,3>& gyro_dps,
                                 std::uint64_t /*timestamp_us*/)
{
    // 1. Estimate orientation via complementary filter
    float pitchAcc, rollAcc;
    computeTiltAngles(acc_mg, pitchAcc, rollAcc);

    // Integrate gyro (deg)
    float rollGyro  = m_roll  + gyro_dps[0] * m_dt;
    float pitchGyro = m_pitch + gyro_dps[1] * m_dt;

    // Fuse
    const float alpha = m_cfg.alpha;
    m_roll  = alpha * rollGyro  + (1.0f - alpha) * rollAcc;
    m_pitch = alpha * pitchGyro + (1.0f - alpha) * pitchAcc;

    // 2. Update baseline buffers
    std::size_t winSamples = static_cast<std::size_t>(std::round(m_cfg.windowSeconds * m_cfg.sampleRateHz));

    auto updateBuf = [winSamples](std::deque<float>& buf, float& mean, float value)
    {
        if (buf.size() == winSamples)
        {
            // remove oldest from mean
            mean -= buf.front();
            buf.pop_front();
        }
        buf.push_back(value);
        mean += value;
    };

    updateBuf(m_pitchBuf, m_pitchMean, m_pitch);
    updateBuf(m_rollBuf,  m_rollMean,  m_roll);

    bool event = false;
    if (m_pitchBuf.size() == winSamples)
    {
        float meanPitch = m_pitchMean / static_cast<float>(winSamples);
        float meanRoll  = m_rollMean  / static_cast<float>(winSamples);
        if (std::fabs(m_pitch - meanPitch) > m_cfg.thresholdDeg ||
            std::fabs(m_roll  - meanRoll ) > m_cfg.thresholdDeg)
        {
            event = true;
        }
    }
    return event;
}

} // namespace motion_detection
