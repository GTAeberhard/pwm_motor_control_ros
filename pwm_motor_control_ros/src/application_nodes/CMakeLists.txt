add_executable(joy_motor_control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/src/joy_motor_control_node.cpp)

target_link_libraries(joy_motor_control_node
  ${catkin_LIBRARIES})

install(TARGETS joy_motor_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)