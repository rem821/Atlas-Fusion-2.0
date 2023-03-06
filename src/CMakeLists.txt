cmake_minimum_required(VERSION 3.22.1)
project(atlas_fusion)

# Use C++ 20
SET(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

# Turn on all possible warnings during build
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Setup Debug and Release flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Find package paths
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

find_package(atlas_fusion_interfaces REQUIRED)

find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(Yaml-cpp)

add_executable(${PROJECT_NAME} src/main.cpp)
target_precompile_headers(${PROJECT_NAME} PRIVATE include/precompiled_headers/PCH.h)
file(GLOB_RECURSE DATA_LOADER_SRCS src/data_loaders/*.cpp)
file(GLOB_RECURSE DATA_MODEL_SRCS src/data_models/*.cpp)
file(GLOB_RECURSE UTIL_SRCS src/util/*.cpp)
file(GLOB_RECURSE LOGGING_SRCS src/logging/*.cpp)

target_sources(${PROJECT_NAME} PRIVATE ${DATA_LOADER_SRCS} ${DATA_MODEL_SRCS} ${UTIL_SRCS} ${LOGGING_SRCS})

add_subdirectory(libs/robotic-template-library)
add_subdirectory(libs/spdlog)
target_precompile_headers(${PROJECT_NAME} PRIVATE include/precompiled_headers/PCH.h)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${camera_info_manager_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Qt5_INCLUDE_DIRS}
        libs/robotic-template-library/include
        )

target_link_directories(${PROJECT_NAME} PRIVATE ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs rviz_2d_overlay_msgs atlas_fusion_interfaces image_transport camera_info_manager tf2_ros)
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        Qt5::Core
        stdc++fs
        yaml-cpp
        fmt
        )

ament_package()