cmake_minimum_required(VERSION 3.8)
project(attention_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(attention_system REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_cascade_lifecycle
  attention_system
  behaviortree_cpp_v3
)

include_directories(include)

set(plugin_sources
  src/attention/activate_attention.cpp
  src/attention/deactivate_attention.cpp
)

set(plugin_libs "")

foreach(src ${plugin_sources})
  get_filename_component(plugin ${src} NAME_WE)
  add_library(${plugin}_bt_node SHARED ${src})
  ament_target_dependencies(${plugin}_bt_node ${dependencies})
  target_compile_definitions(${plugin}_bt_node PRIVATE BT_PLUGIN_EXPORT)
  list(APPEND plugin_libs ${plugin}_bt_node)
endforeach()

add_executable(attention_test src/main_attention.cpp)
ament_target_dependencies(attention_test ${dependencies})

install(TARGETS
  attention_test
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
