cmake_minimum_required(VERSION 3.8)
project(maze_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

add_executable(maze_robot src/maze_generator.cpp)

install(TARGETS maze_robot 
        DESTINATION  lib/${PROJECT_NAME}

)

install(DIRECTORY 
      launch
      worlds
      urdf
      scripts
      config 
      DESTINATION share/${PROJECT_NAME} 
)

#launch 文件使用的运行时输出目录 (share/maze_worlds/generated)
install(CODE "file(MAKE_DIRECTORY  \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/generated\")")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
