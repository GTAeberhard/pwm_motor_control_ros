if(CATKIN_ENABLE_TESTING)
  add_subdirectory(joy_motor_control_node)
  add_subdirectory(pwm_motor_control_node)

  add_rostest(test_launch/pwm_motor_control_component.test)
  add_rostest(test_launch/joy_motor_control_integration_basic.test)
  add_rostest(test_launch/joy_motor_control_integration_xbox_controller.test)

  install(DIRECTORY test_launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_launch)
endif()
