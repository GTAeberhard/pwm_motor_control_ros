if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rospy REQUIRED)

  add_rostest(test_launch/pwm_motor_control_component.test)
  add_rostest(test_launch/joy_motor_control_integration_basic.test)
  add_rostest(test_launch/joy_motor_control_integration_xbox_controller.test)

  catkin_install_python(PROGRAMS
                        src/test_joy_motor_control_initialization.py
                        src/test_joy_motor_control_integration_basic.py
                        src/test_joy_motor_control_integration_xbox_controller.py
                        src/test_pwm_motor_control_node.py
                        src/test_pwm_motor_control_node_initialization.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY test_launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_launch)
endif()
