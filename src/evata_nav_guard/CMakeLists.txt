cmake_minimum_required(VERSION 3.8)
project(evata_nav_guard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# Humble/Iron normally use behaviortree_cpp_v3. Newer distros may expose behaviortree_cpp.
find_package(behaviortree_cpp_v3 QUIET)
if(behaviortree_cpp_v3_FOUND)
  set(EVATA_BT_DEP behaviortree_cpp_v3)
  add_compile_definitions(EVATA_BTCPP_V3)
else()
  find_package(behaviortree_cpp REQUIRED)
  set(EVATA_BT_DEP behaviortree_cpp)
endif()

add_library(evata_safety_pause_bt_node SHARED
  src/safety_pause.cpp
)

target_include_directories(evata_safety_pause_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(evata_safety_pause_bt_node
  rclcpp
  std_msgs
  nav2_behavior_tree
  ${EVATA_BT_DEP}
)

target_compile_definitions(evata_safety_pause_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS evata_safety_pause_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/evata_safety_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
