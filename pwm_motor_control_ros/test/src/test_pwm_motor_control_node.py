#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16
from pwm_test_support.pwm_receiver_node import PwmReceiverNode

TIMEOUT = 5.0

PKG = 'pwm_motor_control_ros'

topic_name_motor_speed = '/motor_speed'


class TestPwmMotorControlNode_MotorSpeeds(unittest.TestCase):
    pwm_receiver = None

    def __init__(self, *args):
        super(TestPwmMotorControlNode_MotorSpeeds, self).__init__(*args)
        self.pwm_receiver = PwmReceiverNode()
        self.pub_motor_speed = rospy.Publisher(topic_name_motor_speed, Int8, queue_size=1)

    def setUp(self):
        time.sleep(1.0)
        self.pwm_receiver.reset_data_received()

    def test_1_when_no_input_received(self):
        self.wait_and_assert_speed(0)
        self.assert_positive_motor_direction()

    def test_2_1_input_50_percent(self):
        self.pub_motor_speed.publish(Int8(50))

        self.wait_and_assert_speed(128)
        self.assert_positive_motor_direction()

    def test_2_2_input_50_percent(self):
        self.pub_motor_speed.publish(Int8(-50))

        self.wait_and_assert_speed(128)
        self.assert_negative_motor_direction()

    def test_3_reset_motor_speed_after_publish_silence(self):
        self.pub_motor_speed.publish(Int8(50))

        self.wait_and_assert_speed(128)
        self.assert_positive_motor_direction()

        # wait for motor speed timeout so that reset to 0 motor speed occurs
        time.sleep(1.1)

        self.wait_and_assert_speed(0)
        self.assert_positive_motor_direction()

    def wait_and_assert_speed(self, speed_value):
        self.pwm_receiver.wait_for_data_received()
        self.assertEqual(self.pwm_receiver.pwm_out_received, speed_value)

    def assert_positive_motor_direction(self):
        self.assertEqual(self.pwm_receiver.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_receiver.pwm_direction_2_received, 0)

    def assert_negative_motor_direction(self):
        self.assertEqual(self.pwm_receiver.pwm_direction_1_received, 0)
        self.assertEqual(self.pwm_receiver.pwm_direction_2_received, 255)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_control_node', __name__, sys.argv)
