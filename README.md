# PWM Controller Nodes

This package wraps the [PWM Motor Control](https://github.com/GTAeberhard/pwm_motor_control) implementation into ROS node, allowing for the control basic PWM DC motors and servo motors using various hardware.

## DC Motor Control

Currently only a stub is implemented in order to test the ROS implementation.  The stub simply outputs the same data that the hardware would output on a pin as a ROS topic.  As input, a motor speed from 0-100 can be published to the topic /motor_speed using the std_nsgs/Int message.

## Servo Motor Control

Not yet implemented.

## Motor Control with a Joystick

To be implemented.