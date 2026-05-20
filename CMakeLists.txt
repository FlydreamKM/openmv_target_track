cmake_minimum_required(VERSION 3.10)
project(openmv_target_track VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_EXAMPLES "Build example applications" OFF)
option(USE_GSTREAMER "Use GStreamer for camera capture" OFF)

# Find OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/target_detector.cpp
    src/pose_estimator.cpp
    src/serial_output.cpp
)

# Library
add_library(${PROJECT_NAME}_core STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME}_core
    ${OpenCV_LIBS}
)

# Main executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_core
    ${OpenCV_LIBS}
)

# Examples
if(BUILD_EXAMPLES)
    add_executable(calibration_tool examples/calibration_tool.cpp)
    target_link_libraries(calibration_tool ${OpenCV_LIBS})
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config/camera_calib.yaml DESTINATION etc/openmv_target_track)
