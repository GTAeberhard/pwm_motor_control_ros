if(CATKIN_ENABLE_TESTING)
  catkin_install_python(PROGRAMS
                        test_pwm_motor_control_node.py
                        test_pwm_motor_control_node_initialization.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
