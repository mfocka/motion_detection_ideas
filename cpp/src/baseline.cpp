#include "baseline.h"
#include <fstream>
#include <numeric>

bool save_baseline(const std::vector<EulerRPY>& window, const std::string& path){
    if(window.empty()) return false;
    double mr=0, mp=0, my=0;
    for(const auto& e : window){ mr+=e.roll; mp+=e.pitch; my+=e.yaw; }
    mr/=window.size(); mp/=window.size(); my/=window.size();
    std::ofstream f(path);
    if(!f) return false;
    f << mr << "," << mp << "," << my;
    return true;
}

std::array<double,3> load_baseline(const std::string& path){
    std::ifstream f(path);
    std::array<double,3> r{0.0,0.0,0.0};
    if(!f) return r;
    char comma;
    f >> r[0] >> comma >> r[1] >> comma >> r[2];
    return r;
}

