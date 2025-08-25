#pragma once
#include <array>
#include <string>
#include <vector>
#include "orientation_filter.h"

bool save_baseline(const std::vector<EulerRPY>& window, const std::string& path);
std::array<double,3> load_baseline(const std::string& path);

