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

find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(Yaml-cpp)

include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs rviz_2d_overlay_msgs image_transport camera_info_manager tf2_ros)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} Qt5::Core stdc++fs yaml-cpp)

ament_package()