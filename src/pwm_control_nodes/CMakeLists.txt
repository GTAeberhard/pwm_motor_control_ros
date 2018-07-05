add_executable(pwm_motor_control_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pwm_motor_control_node.cpp)
target_include_directories(pwm_motor_control_node
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pwm_motor_control_node
    ${catkin_LIBRARIES}
    motor_control
    gpio_stub_ros)

add_executable(joy_motor_control_node
    ${CMAKE_CURRENT_SOURCE_DIR}/src/joy_motor_control_node.cpp)
target_link_libraries(joy_motor_control_node
    ${catkin_LIBRARIES})