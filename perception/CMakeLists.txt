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
find_package(yolov8_msgs REQUIRED)
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
  yolov8_msgs
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  tf2
  rclcpp_action
  behaviortree_cpp_v3
  perception_system
  OpenCV
  perception_system_interfaces
  sensor_msgs
  std_srvs
)

include_directories(include)

add_library(is_detected_bt_node SHARED src/perception/IsDetected.cpp)
add_library(filter_entity_bt_node SHARED src/perception/FilterEntity.cpp)
add_library(is_pointing_bt_node SHARED src/perception/IsPointing.cpp)
add_library(identify_person_bt_node SHARED src/perception/IdentifyPerson.cpp)
add_library(get_angle_bt_node SHARED src/perception/GetAngle.cpp)
add_library(is_in_front_bt_node SHARED src/perception/IsInFront.cpp)

list(APPEND plugin_libs
  is_detected_bt_node
  filter_entity_bt_node
  is_pointing_bt_node
  identify_person_bt_node
  get_angle_bt_node
  is_in_front_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(perception_test src/main_perception.cpp)
ament_target_dependencies(perception_test ${dependencies})
target_link_libraries(perception_test ${ZMQ_LIBRARIES})

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

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

foreach(bt_plugin ${plugin_libs})
  ament_export_libraries(${bt_plugin})
endforeach()

ament_package()
