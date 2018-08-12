add_library(gpio_stub_ros STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_pwm_ros_stub.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_ros_stub.cpp)

target_include_directories(gpio_stub_ros PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${pwm_motor_control_INCLUDE_DIRS})

target_link_libraries(gpio_stub_ros
  ${catkin_LIBRARIES}
  gpio)

install(TARGETS gpio_stub_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)