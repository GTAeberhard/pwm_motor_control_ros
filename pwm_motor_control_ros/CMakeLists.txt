cmake_minimum_required(VERSION 3.8)

project(pwm_motor_control_ros)

add_compile_options(-std=c++14)

find_package(pwm_motor_control REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS pwm_motor_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${pwm_motor_control_INCLUDE_DIRS}
)

add_subdirectory(src/device_nodes)
add_subdirectory(src/pwm_control_nodes)
add_subdirectory(src/ros_stubs)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/pwm_motor_control_component.test)
  add_rostest(test/joy_motor_control_integration.test)
endif()

