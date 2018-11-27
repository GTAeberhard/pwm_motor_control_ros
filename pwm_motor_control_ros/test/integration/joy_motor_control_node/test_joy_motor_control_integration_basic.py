#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest
from sensor_msgs.msg import Joy
from pwm_test_support.base_pwm_test_fixture import BasePwmTestFixture

PKG = 'pwm_motor_control_ros'

topic_name_joy = '/joy'

class TestJoyMotorControl_JoyTriggerInputForPositiveMotorSpeed(BasePwmTestFixture):
    def __init__(self, *args):
        super(TestJoyMotorControl_JoyTriggerInputForPositiveMotorSpeed, self).__init__(*args)
        self.pub_joy = rospy.Publisher(topic_name_joy, Joy, queue_size=1)

    def test_1_full_motor_speed(self):
        joy_input = Joy()
        joy_input.axes.append(1.0)

        self.pub_joy.publish(joy_input)

        self.wait_and_assert_pwm_value(255)
        self.assert_positive_motor_direction()

    def test_2_half_motor_speed(self):
        joy_input = Joy()
        joy_input.axes.append(0.5)

        self.pub_joy.publish(joy_input)

        self.wait_and_assert_pwm_value(128)
        self.assert_positive_motor_direction()


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_joy_motor_control_integration', __name__, sys.argv)
