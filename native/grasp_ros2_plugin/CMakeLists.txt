cmake_minimum_required(VERSION 3.16)
project(manipulation_grasp_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(mujoco_vendor REQUIRED)

file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/src/observation.cpp" OBSERVATION_SHA256)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/src/ordered_fifo.hpp" FIFO_SHA256)
add_library(mujoco_ros2_control_grasp_observer SHARED src/observation.cpp)
target_compile_features(mujoco_ros2_control_grasp_observer PUBLIC cxx_std_17)
target_compile_options(mujoco_ros2_control_grasp_observer PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_compile_definitions(mujoco_ros2_control_grasp_observer PRIVATE
  MANIPULATION_GRASP_PLUGIN_SHA256="${OBSERVATION_SHA256}"
  MANIPULATION_GRASP_FIFO_SHA256="${FIFO_SHA256}")
ament_target_dependencies(mujoco_ros2_control_grasp_observer rclcpp std_msgs pluginlib)
target_link_libraries(mujoco_ros2_control_grasp_observer
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins mujoco::mujoco)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugin.xml)
install(TARGETS mujoco_ros2_control_grasp_observer LIBRARY DESTINATION lib)
ament_export_dependencies(rclcpp std_msgs pluginlib mujoco_ros2_control_plugins mujoco_vendor)
ament_package()
