cmake_minimum_required(VERSION 3.8)

project(pwm_motor_control_ros)

add_compile_options(-std=c++14)

find_package(pwm_motor_control REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS pwm_motor_control
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rospy REQUIRED)
  add_subdirectory(test)
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})