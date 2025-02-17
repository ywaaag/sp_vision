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
add_subdirectory(tasks/auto_aim)

##################tests##################
add_executable(yolo11_test tests/yolo11_test.cpp)

target_link_libraries(yolo11_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)

