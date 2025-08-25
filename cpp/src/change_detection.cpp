#include "change_detection.h"
#include <cmath>
#include <algorithm>

static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

std::vector<OrientationEventC> detect_changes(
    const std::vector<EulerRPY>& euler,
    double az_thresh_deg,
    double alt_thresh_deg,
    double T_detection_s,
    double T_validation_s,
    double fs_hz,
    double smooth_window_s
){
    const std::size_t N = euler.size();
    if(N < 2) return {};

    // Smooth via simple moving average
    const std::size_t win = std::max<std::size_t>(1, std::lround(smooth_window_s * fs_hz));
    auto smooth = [&](auto accessor){
        std::vector<double> out(N,0.0);
        double sum=0.0; std::size_t w=0;
        for(std::size_t i=0;i<N;i++){
            sum += accessor(i);
            if(i>=win){ sum -= accessor(i-win); w=win; } else { w=i+1; }
            out[i] = sum / static_cast<double>(w);
        }
        return out;
    };

    auto roll  = smooth([&](std::size_t i){ return rad2deg(euler[i].roll);  });
    auto pitch = smooth([&](std::size_t i){ return rad2deg(euler[i].pitch); });
    auto yaw   = smooth([&](std::size_t i){ return rad2deg(euler[i].yaw);   });

    std::size_t det_n = std::max<std::size_t>(1, std::lround(T_detection_s * fs_hz));
    std::size_t val_n = std::max<std::size_t>(1, std::lround(T_validation_s * fs_hz));

    std::vector<OrientationEventC> events;
    for(std::size_t t=det_n; t<N; ++t){
        double droll  = roll[t]  - roll[t-det_n];
        double dpitch = pitch[t] - pitch[t-det_n];
        double dyaw   = yaw[t]   - yaw[t-det_n];
        bool az_ok = std::abs(dyaw)   >= az_thresh_deg;
        bool alt_ok= std::abs(dpitch) >= alt_thresh_deg;
        if(!(az_ok || alt_ok)) continue;

        std::size_t end_idx = std::min(N-1, t+val_n);
        double new_r = roll[t],  new_p = pitch[t], new_y = yaw[t];
        bool persistent = true;
        for(std::size_t k=t; k<=end_idx; ++k){
            if(std::abs(roll[k]-new_r)  > std::abs(droll)*0.5 + 1.0) { persistent=false; break; }
            if(std::abs(pitch[k]-new_p) > std::abs(dpitch)*0.5 + 1.0){ persistent=false; break; }
            if(std::abs(yaw[k]-new_y)   > std::abs(dyaw)*0.5 + 1.0)  { persistent=false; break; }
        }
        events.push_back({t, static_cast<double>(t)/fs_hz, droll, dpitch, dyaw, persistent});
    }

    return events;
}

