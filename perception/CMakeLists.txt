cmake_minimum_required(VERSION 3.8)
project(perception_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(perception_system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(perception_system_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_cascade_lifecycle
  rclpy
  behaviortree_cpp_v3
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  tf2
  rclcpp_action
  perception_system
  OpenCV
  perception_system_interfaces
  sensor_msgs
  std_srvs
)

include_directories(include)

set(plugin_sources
  src/perception/IsDetected.cpp
  src/perception/FilterEntity.cpp
  src/perception/IsPointing.cpp
  src/perception/IdentifyPerson.cpp
  src/perception/Identify.cpp
  src/perception/GetAngle.cpp
  src/perception/IsInFront.cpp
)

set(plugin_libs "")

foreach(src ${plugin_sources})
  get_filename_component(plugin ${src} NAME_WE)
  add_library(${plugin}_bt_node SHARED ${src})
  ament_target_dependencies(${plugin}_bt_node ${dependencies})
  target_compile_definitions(${plugin}_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${plugin}_bt_node)
endforeach()

add_executable(perception_test src/main_perception.cpp)
ament_target_dependencies(perception_test ${dependencies})

install(TARGETS
  perception_test
  ${plugin_libs}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch config bt_xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
