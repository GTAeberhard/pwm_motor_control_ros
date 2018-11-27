#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest
from sensor_msgs.msg import Joy
from pwm_test_support.base_pwm_test_fixture import BasePwmTestFixture
from pwm_test_support.xbox_controller_joy_stub import XboxControllerJoyStub

PKG = 'pwm_motor_control_ros'

topic_name_joy = '/joy'

class TestXboxJoyMotorControl_RightTriggerForPositiveMotorSpeed(BasePwmTestFixture):
    def __init__(self, *args):
        super(TestXboxJoyMotorControl_RightTriggerForPositiveMotorSpeed, self).__init__(*args)
        self.pub_joy = rospy.Publisher(topic_name_joy, Joy, queue_size=1)

    def test_1_full_motor_speed_positive(self):
        self.pub_joy.publish(XboxControllerJoyStub.right_trigger_depressed())

        self.wait_and_assert_pwm_value(255)
        self.assert_positive_motor_direction()

    def test_2_idle_motor_speed_positive(self):
        self.pub_joy.publish(XboxControllerJoyStub.idle_controller())

        self.wait_and_assert_pwm_value(0)
        self.assert_positive_motor_direction()

    def test_3_half_motor_speed_positive(self):
        self.pub_joy.publish(XboxControllerJoyStub.right_trigger_half_depressed())

        self.wait_and_assert_pwm_value(128)
        self.assert_positive_motor_direction()


class TestXboxJoyMotorControl_LeftTriggerForNegativeMotorSpeed(BasePwmTestFixture):
    def __init__(self, *args):
        super(TestXboxJoyMotorControl_LeftTriggerForNegativeMotorSpeed, self).__init__(*args)
        self.pub_joy = rospy.Publisher(topic_name_joy, Joy, queue_size=1)

    def test_1_full_motor_speed_negative(self):
        self.pub_joy.publish(XboxControllerJoyStub.left_trigger_depressed())

        self.wait_and_assert_pwm_value(255)
        self.assert_negative_motor_direction()

    def test_2_half_motor_speed_negative(self):
        self.pub_joy.publish(XboxControllerJoyStub.left_trigger_half_depressed())

        self.wait_and_assert_pwm_value(128)
        self.assert_negative_motor_direction()


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_joy_motor_control_integration_xbox', __name__, sys.argv)
