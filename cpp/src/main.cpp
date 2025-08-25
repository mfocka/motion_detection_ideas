#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <string>
#include "orientation_filter.h"
#include "change_detection.h"
#include "baseline.h"

struct Sample { double ax_mg, ay_mg, az_mg; double gx_mdps, gy_mdps, gz_mdps; };

static bool read_csv(const std::string& path, std::vector<Sample>& out){
    std::ifstream f(path);
    if(!f) return false;
    std::string line; std::getline(f,line); // header
    while(std::getline(f,line)){
        std::stringstream ss(line);
        std::string item; Sample s{};
        // timestamp_ms
        std::getline(ss,item,',');
        // ax,ay,az,gx,gy,gz
        std::getline(ss,item,','); s.ax_mg = std::stod(item);
        std::getline(ss,item,','); s.ay_mg = std::stod(item);
        std::getline(ss,item,','); s.az_mg = std::stod(item);
        std::getline(ss,item,','); s.gx_mdps = std::stod(item);
        std::getline(ss,item,','); s.gy_mdps = std::stod(item);
        std::getline(ss,item,','); s.gz_mdps = std::stod(item);
        out.push_back(s);
    }
    return true;
}

int main(int argc, char** argv){
    std::string path = argc>1? argv[1] : "../data/imu1.csv";
    double fs = 104.0;
    std::vector<Sample> samples;
    if(!read_csv(path, samples)){
        std::cerr << "Failed to read " << path << "\n";
        return 1;
    }
    const double G_TO_MS2 = 9.80665; const double MG_TO_MS2 = G_TO_MS2/1000.0;
    const double DEG_TO_RAD = 3.141592653589793/180.0;

    ComplementaryFilter cf(0.1, fs);
    std::vector<EulerRPY> eulers; eulers.reserve(samples.size());
    for(const auto& s: samples){
        std::array<double,3> acc{ s.ax_mg*MG_TO_MS2, s.ay_mg*MG_TO_MS2, s.az_mg*MG_TO_MS2 };
        std::array<double,3> gyr{ s.gx_mdps*DEG_TO_RAD*1000.0, s.gy_mdps*DEG_TO_RAD*1000.0, s.gz_mdps*DEG_TO_RAD*1000.0 };
        eulers.push_back(cf.update(acc,gyr));
    }

    auto events = detect_changes(eulers, /*az*/5.0, /*alt*/5.0, /*det*/2.0, /*val*/10.0, fs, /*smooth*/0.5);
    std::cout << "events: " << events.size() << "\n";
    for(const auto& ev: events){
        std::cout << ev.index << ", t=" << ev.timestamp_s << ", dR=" << ev.delta_roll_deg
                  << ", dP=" << ev.delta_pitch_deg << ", dY=" << ev.delta_yaw_deg
                  << ", persist=" << ev.persistent << "\n";
    }

    save_baseline(eulers, "baseline_cpp.txt");
    auto bl = load_baseline("baseline_cpp.txt");
    std::cout << "baseline rpy(rad): " << bl[0] << ", " << bl[1] << ", " << bl[2] << "\n";
    return 0;
}

