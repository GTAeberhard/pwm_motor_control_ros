#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest
from std_msgs.msg import Int8
from pwm_test_support.base_pwm_test_fixture import BasePwmTestFixture

PKG = 'pwm_motor_control_ros'

topic_name_motor_speed = '/motor_speed'


class TestPwmMotorControlNode_MotorSpeeds(BasePwmTestFixture):
    pwm_receiver = None

    def __init__(self, *args):
        super(TestPwmMotorControlNode_MotorSpeeds, self).__init__(*args)
        self.pub_motor_speed = rospy.Publisher(topic_name_motor_speed, Int8, queue_size=1)

    def test_1_when_no_input_received(self):
        self.wait_and_assert_pwm_value(0)
        self.assert_positive_motor_direction()

    def test_2_1_input_50_percent(self):
        self.pub_motor_speed.publish(Int8(50))

        self.wait_and_assert_pwm_value(128)
        self.assert_positive_motor_direction()

    def test_2_2_input_50_percent(self):
        self.pub_motor_speed.publish(Int8(-50))

        self.wait_and_assert_pwm_value(128)
        self.assert_negative_motor_direction()

    def test_3_reset_motor_speed_after_publish_silence(self):
        self.pub_motor_speed.publish(Int8(50))

        self.wait_and_assert_pwm_value(128)
        self.assert_positive_motor_direction()

        # wait for motor speed timeout so that reset to 0 motor speed occurs
        time.sleep(1.1)

        self.wait_and_assert_pwm_value(0)
        self.assert_positive_motor_direction()


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_motorc_ontrol_node', __name__, sys.argv)
