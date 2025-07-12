cmake_minimum_required(VERSION 3.16)

find_package(Ceres REQUIRED)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    ransac_sine_fitter.cpp
)
target_link_libraries(tools)
    pid.cpp
    crc.cpp
)
