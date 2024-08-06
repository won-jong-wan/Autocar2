cmake_minimum_required(VERSION 3.10)
project(jac_gric)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pop_interfaces REQUIRED)

#build
add_executable(autocar2 src/gric_autocar2_node.cpp)
#target_include_directories(autocar2 PRIVATE include)
#target_link_libraries(autocar2 OpenXLSX::OpenXLSX)
ament_target_dependencies(autocar2 rclcpp std_msgs geometry_msgs sensor_msgs pop_interfaces)

add_executable(joy2teleop src/joy2teleop.cpp)
#target_include_directories(joy2tel PRIVATE include)
ament_target_dependencies(joy2teleop rclcpp std_msgs sensor_msgs pop_interfaces)

install(TARGETS
  autocar2
  joy2teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
