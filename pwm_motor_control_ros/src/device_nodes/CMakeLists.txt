add_executable(pwm_motor_control_node_ros
  ${CMAKE_CURRENT_SOURCE_DIR}/pwm_motor_control_ros.cpp)

target_link_libraries(pwm_motor_control_node_ros
  ${catkin_LIBRARIES}
  pwm_motor_control_node)

install(TARGETS pwm_motor_control_node_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)