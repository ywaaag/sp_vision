cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

target_link_libraries(tools yaml-cpp)