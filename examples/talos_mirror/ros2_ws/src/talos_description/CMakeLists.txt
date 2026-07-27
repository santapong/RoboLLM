cmake_minimum_required(VERSION 3.8)
project(talos_description)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/10_robot_state_publisher.yaml
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

#############
## Testing ##
#############

#if(BUILD_TESTING)
#    ament_auto_find_test_dependencies()
#    ament_lint_auto_find_test_dependencies()
  
#    # calibration_parameters_node_test
#    ament_auto_add_gtest(calibration_parameters_node_test
#        test/calibration_parameters_node_test.cpp
#    )

#    ament_target_dependencies(calibration_parameters_node_test
#      launch_testing_ament_cmake
#    )
  
#    install(
#     TARGETS calibration_parameters_node_test
#     DESTINATION lib/${PROJECT_NAME}
#  )

#    add_launch_test(
#             test/calibration_parameters_node_test.launch.py
#            )
#endif()


ament_auto_package(INSTALL_TO_SHARE config gazebo launch meshes robots ros2_control srdf urdf)
