cmake_minimum_required(VERSION 3.8)
project(air_bms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES
    src/bms_node.cpp
    src/bms_uart.cpp
)
add_executable(bms_status_node  ${SOURCES})

ament_target_dependencies(bms_status_node 
  rclcpp
  sensor_msgs
)
install(TARGETS bms_status_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()