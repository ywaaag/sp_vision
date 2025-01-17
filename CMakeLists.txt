cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim_sentry)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(sentry src/sentry.cpp)
add_executable(standard src/standard.cpp)
add_executable(auto_aim_debug src/auto_aim_debug.cpp)
add_executable(multithread src/multithread_debug.cpp)

target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim_sentry omniperception tools io)
target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim_sentry tools io)
target_link_libraries(auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim_sentry omniperception tools io)
target_link_libraries(multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim_sentry omniperception tools io)

##################examples################
add_executable(capture examples/capture.cpp)
add_executable(calibrate_camera examples/calibrate_camera.cpp)
add_executable(calibrate_handeye examples/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye examples/calibrate_robotworld_handeye.cpp)
add_executable(split_video examples/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(armor_detect_video_test tests/armor_detect_video_test.cpp)
add_executable(armor_detect_test tests/armor_detect_test.cpp)
add_executable(auto_aim_sentry_test tests/auto_aim_sentry_test.cpp)
add_executable(auto_shoot_test tests/auto_shoot_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(pnp_test tests/pnp_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(ekfpnp_test tests/ekfpnp_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(publish_test tests/publish_test.cpp)

target_link_libraries(armor_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim_sentry)
target_link_libraries(auto_aim_sentry_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim_sentry)
target_link_libraries(armor_detect_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim_sentry)
target_link_libraries(auto_shoot_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim_sentry io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(pnp_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim_sentry io)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(ekfpnp_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim_sentry)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim_sentry)

