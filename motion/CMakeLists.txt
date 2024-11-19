cmake_minimum_required(VERSION 3.8)
project(motion_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
# find_package(nav2_costmap_2d REQUIRED)
# find_package(pluginlib REQUIRED)
find_package(navigation_system_interfaces REQUIRED)
# find_package(slam_toolbox REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
# find_package(lifecycle_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  # nav2_costmap_2d
  # pluginlib
  navigation_system_interfaces
  # slam_toolbox
  ament_index_cpp
  trajectory_msgs
  # lifecycle_msgs
  sensor_msgs
)

include_directories(include)

add_library(navigate_to_bt_node SHARED src/motion/navigation/NavigateTo.cpp)
add_library(navigate_through_bt_node SHARED src/motion/navigation/NavigateThrough.cpp)
add_library(rotate_bt_node SHARED src/motion/base/Rotate.cpp)
add_library(pan_bt_node SHARED src/motion/head/Pan.cpp)

list(APPEND
  plugin_libs
  navigate_to_bt_node
  navigate_through_bt_node
  rotate_bt_node
  pan_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nav_test src/motion/main_nav.cpp)
ament_target_dependencies(nav_test ${dependencies})
target_link_libraries(nav_test ${ZMQ_LIBRARIES})

install(TARGETS
  nav_test
  ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch config bt_xml
  DESTINATION share/${PROJECT_NAME}
)

# install(FILES clear_people_layer.xml
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
# ament_export_libraries(clear_people_layer)
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# pluginlib_export_plugin_description_file(nav2_costmap_2d clear_people_layer.xml)

ament_package()
