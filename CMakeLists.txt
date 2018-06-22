cmake_minimum_required(VERSION 3.8)

project(pwm_motor_control_ros)

add_compile_options(-std=c++14)

add_subdirectory(pwm_motor_control)

find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(pwm_motor_control)

catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS pwm_motor_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pwm_motor_control_node src/pwm_motor_control_node.cpp)
target_link_libraries(pwm_motor_control_node ${catkin_LIBRARIES})
