add_library(gpio_stub_ros
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_pwm_ros_stub.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_ros_stub.cpp)
target_include_directories(gpio_stub_ros PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gpio_stub_ros
    ${catkin_LIBRARIES}
    gpio)