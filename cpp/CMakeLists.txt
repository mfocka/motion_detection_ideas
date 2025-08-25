cmake_minimum_required(VERSION 3.10)
project(wds_orientation CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(wds_orient
    src/main.cpp
    src/orientation_filter.cpp
    src/change_detection.cpp
    src/baseline.cpp
)

target_include_directories(wds_orient PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
