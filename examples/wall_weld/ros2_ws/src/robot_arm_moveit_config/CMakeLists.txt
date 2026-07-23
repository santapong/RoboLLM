cmake_minimum_required(VERSION 3.8)
project(robot_arm_moveit_config)
find_package(ament_cmake REQUIRED)
# rviz/ must ship too: wallweld.launch.py falls back to
# <share>/rviz/wallweld.rviz when the /ros2_ws/src mount is absent.
install(DIRECTORY config launch scripts rviz DESTINATION share/${PROJECT_NAME})
# hand_follow.py imports arm_ik from the workspace-level tools/ dir; install it
# next to the scripts so the package is self-contained after a colcon build
# (ws layout is <ws>/src/<this pkg> with tools at <ws>/tools in both the dev
# workspace and the vendored RoboLLM example).
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/arm_ik.py)
  install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/arm_ik.py
          DESTINATION share/${PROJECT_NAME}/scripts)
endif()
ament_package()
