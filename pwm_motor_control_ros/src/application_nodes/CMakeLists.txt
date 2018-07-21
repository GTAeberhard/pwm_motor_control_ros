add_executable(joy_motor_control_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/joy_motor_control_node.cpp)
target_link_libraries(joy_motor_control_node
    ${catkin_LIBRARIES})