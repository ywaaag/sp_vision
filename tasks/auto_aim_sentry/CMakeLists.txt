cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2023.3.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_library(auto_aim_sentry OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    yolov8.cpp
    voter.cpp
    ekfpnp.cpp
)

target_link_libraries(auto_aim_sentry openvino::runtime)