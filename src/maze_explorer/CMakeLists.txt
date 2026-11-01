cmake_minimum_required(VERSION 3.8)
project(maze_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(explorer_node src/explorer_node.cpp)
target_compile_features(explorer_node PUBLIC cxx_std_17)
target_include_directories(explorer_node PRIVATE include)
ament_target_dependencies(explorer_node
  rclcpp rclcpp_action nav_msgs sensor_msgs nav2_msgs geometry_msgs tf2 tf2_ros lifecycle_msgs)

install(TARGETS explorer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/record_mapping_diagnostics.py scripts/replay_mapping.py
  scripts/evaluate_mapping_recording.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/mapping_analysis.py scripts/mapping_events.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frontier_search test/test_frontier_search.cpp)
  target_compile_features(test_frontier_search PUBLIC cxx_std_17)
  target_include_directories(test_frontier_search PRIVATE include)
  ament_target_dependencies(test_frontier_search nav_msgs)
  ament_add_gtest(test_operation_tracker test/test_operation_tracker.cpp)
  target_compile_features(test_operation_tracker PUBLIC cxx_std_17)
  target_include_directories(test_operation_tracker PRIVATE include)
  ament_add_test(test_explorer_actions
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_explorer_actions.py
      --executable $<TARGET_FILE:explorer_node>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
  ament_add_test(test_mapping_replay
    COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mapping_replay.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
endif()

ament_package()
