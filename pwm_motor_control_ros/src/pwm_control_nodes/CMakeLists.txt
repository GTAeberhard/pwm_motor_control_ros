add_library(pwm_motor_control_node STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pwm_motor_control_node.cpp)

target_include_directories(pwm_motor_control_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${pwm_motor_control_INCLUDE_DIRS})

target_link_libraries(pwm_motor_control_node
  ${catkin_LIBRARIES}
  motor_control
  gpio_stub_ros)

install(TARGETS pwm_motor_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)