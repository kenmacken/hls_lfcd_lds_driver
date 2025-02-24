################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(hls_lfcd_lds_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_executable(hlds_laser_publisher	src/hlds_laser_publisher.cpp)
add_executable(lidar_tf_broadcaster	src/lidar_tf_broadcaster.cpp)
target_link_libraries(hlds_laser_publisher ${Boost_LIBRARIES})
target_link_libraries(lidar_tf_broadcaster ${Boost_LIBRARIES})
ament_target_dependencies(hlds_laser_publisher
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(lidar_tf_broadcaster
  rclcpp
  tf2_ros
)


################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS hlds_laser_publisher lidar_tf_broadcaster
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)
ament_export_include_directories(include)
ament_package()
