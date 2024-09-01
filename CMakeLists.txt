cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)

set(OpenVINO_DIR "/opt/intel/openvino_2023.3.0/runtime/cmake/")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)

add_executable(camera_test examples/camera_test.cpp)
add_executable(cboard_test examples/cboard_test.cpp)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools)

add_executable(auto_aim_test examples/auto_aim_test.cpp)
target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)
add_executable(armor_detector_video examples/armor_detector_video.cpp)
target_link_libraries(armor_detector_video ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools)

add_executable(auto_buff_test examples/auto_buff_test.cpp)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} openvino::runtime fmt::fmt yaml-cpp auto_buff tools)

add_executable(capture examples/capture.cpp)
add_executable(calibrate_camera examples/calibrate_camera.cpp)
add_executable(calibrate_handeye examples/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye examples/calibrate_robotworld_handeye.cpp)
target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

add_executable(split examples/split_vedio.cpp)
target_link_libraries(split ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

add_executable(auto_aim_debug src/auto_aim_debug.cpp)
target_link_libraries(auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(auto_buff_debug src/auto_buff_debug.cpp)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} openvino::runtime fmt::fmt yaml-cpp auto_buff tools io)

add_executable(standard src/standard.cpp)
target_link_libraries(standard ${OpenCV_LIBS} openvino::runtime fmt::fmt yaml-cpp auto_aim auto_buff tools io)
