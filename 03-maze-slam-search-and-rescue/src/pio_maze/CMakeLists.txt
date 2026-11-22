cmake_minimum_required(VERSION 3.8)
project(pio_maze)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---- C++ behavior-tree executor ----
add_executable(rescue_bt src/rescue_bt.cpp)
target_include_directories(rescue_bt PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(rescue_bt
  rclcpp rclcpp_action behaviortree_cpp nav2_msgs geometry_msgs ament_index_cpp)
install(TARGETS rescue_bt DESTINATION lib/${PROJECT_NAME})

# ---- shared resources ----
install(
  DIRECTORY urdf worlds config launch rviz behavior_trees models maps
  DESTINATION share/${PROJECT_NAME}
)

# ---- python nodes ----
install(
  PROGRAMS
    scripts/frontier_explorer.py
    scripts/align_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
