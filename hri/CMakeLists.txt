cmake_minimum_required(VERSION 3.8)
project(hri_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(llama_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  ament_index_cpp
  rclcpp_cascade_lifecycle
  rclcpp_action
  behaviortree_cpp_v3
  audio_common_msgs
  whisper_msgs
  llama_msgs
  std_msgs
  sensor_msgs
  std_srvs
  audio_common_msgs
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(listen_bt_node SHARED src/hri/dialog/listen.cpp)
add_library(speak_bt_node SHARED src/hri/dialog/speak.cpp)
add_library(dialog_confirmation_bt_node SHARED src/hri/dialog/dialog_confirmation.cpp)
add_library(query_bt_node SHARED src/hri/dialog/query.cpp)

list(APPEND plugin_libs
  listen_bt_node
  speak_bt_node
  dialog_confirmation_bt_node
  query_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(hri_test src/main_hri.cpp)
ament_target_dependencies(hri_test ${dependencies})
target_link_libraries(hri_test ${ZMQ_LIBRARIES})

install(TARGETS
  hri_test
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

foreach(bt_plugin ${plugin_libs})
  ament_export_libraries(${bt_plugin})
endforeach()
  
ament_package()
