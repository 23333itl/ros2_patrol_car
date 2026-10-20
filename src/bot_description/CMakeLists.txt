cmake_minimum_required(VERSION 3.8)
project(bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 模型、launch、世界文件等静态资源
install(
  DIRECTORY bot
            launch
            models
            maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/joint_state_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
