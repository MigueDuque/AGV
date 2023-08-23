cmake_minimum_required(VERSION 3.5)
project(agv_control)

# Add C++14 standard
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/lib)

add_library(serialib STATIC lib/serialib.cpp)

add_executable(agv_controller src/agv_controller.cpp)

target_sources(agv_controller PRIVATE
  src/agv_controller.cpp
)

ament_target_dependencies(agv_controller
  rclcpp
  geometry_msgs
)

# Enlaza la biblioteca serialib al ejecutable agv_controller
target_link_libraries(agv_controller
  serialib
)

install(TARGETS 
  agv_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()