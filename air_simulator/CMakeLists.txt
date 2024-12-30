cmake_minimum_required(VERSION 3.8)
project(air_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/scanMerger.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC include)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  urdf
  rviz
  meshes
  maps
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
