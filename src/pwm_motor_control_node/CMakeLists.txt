add_executable(pwm_motor_control_node
    ${CMAKE_CURRENT_SOURCE_DIR}/pwm_motor_control_node.cpp)
target_link_libraries(pwm_motor_control_node
    ${catkin_LIBRARIES}
    motor_control
    gpio_stub_ros)