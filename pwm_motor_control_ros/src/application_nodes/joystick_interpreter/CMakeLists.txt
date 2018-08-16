add_library (joystick_interpreter STATIC
             ${CMAKE_CURRENT_SOURCE_DIR}/src/joystick_interpreter.cpp)

target_include_directories(joystick_interpreter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CATKIN_PACKAGE_INCLUDE_DESTINATION}>
)

install(TARGETS joystick_interpreter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(joy_motor_control_node
  ${CMAKE_CURRENT_SOURCE_DIR}/joy_motor_control_node.cpp)

target_link_libraries(joy_motor_control_node
  ${catkin_LIBRARIES})

install(TARGETS joy_motor_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)