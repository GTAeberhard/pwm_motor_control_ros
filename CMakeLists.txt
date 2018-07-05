cmake_minimum_required(VERSION 3.8)

project(pwm_motor_control_ros)

add_compile_options(-std=c++14)

add_subdirectory(pwm_motor_control)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

find_package(pwm_motor_control)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS pwm_motor_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/pwm_control_nodes)
add_subdirectory(src/gpio_stub)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/pwm_motor_control_node.test)
  add_rostest(test/joy_motor_control.test)
endif()