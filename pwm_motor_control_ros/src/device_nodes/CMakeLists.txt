add_executable(pwm_motor_control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/pwm_motor_control_ros.cpp)

target_link_libraries(pwm_motor_control_node
  ${catkin_LIBRARIES}
  pwm_motor_control_node_lib
  gpio_stub_ros)

install(TARGETS pwm_motor_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(pwm_servo_control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/pwm_servo_control_ros.cpp)

target_link_libraries(pwm_servo_control_node
  ${catkin_LIBRARIES}
  pwm_servo_control_node_lib
  gpio_stub_ros)

install(TARGETS pwm_servo_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)