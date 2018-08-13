if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pwm_motor_control_component.test)
  add_rostest(joy_motor_control_integration.test)

  catkin_install_python(PROGRAMS
                        test_joy_motor_control_initialization.py
                        test_joy_motor_control_integration.py
                        test_pwm_motor_control_node.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
