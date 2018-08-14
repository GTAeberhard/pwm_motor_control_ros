#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16
from sensor_msgs.msg import Joy
from pwm_test_support.pwm_receiver_node import PwmReceiverNode

TIMEOUT = 5.0

PKG = 'pwm_motor_control_ros'

pwm_motor_control_node_name = '/pwm_motor_control'
joy_motor_control_node_name = '/joy_motor_control'

topic_name_motor_speed = '/motor_speed'
topic_name_joy = '/joy'

class TestJoyMotorControl_JoyTriggerInputForPositiveMotorSpeed(unittest.TestCase):
    pwm_receiver = None

    def __init__(self, *args):
        super(TestJoyMotorControl_JoyTriggerInputForPositiveMotorSpeed, self).__init__(*args)
        self.pwm_receiver = PwmReceiverNode()
        self.pub_joy = rospy.Publisher(topic_name_joy, Joy, queue_size=1)

    def test_1_full_motor_speed(self):
        # Need to wait so that node connections can be established
        time.sleep(1.0)
        
        self.pwm_receiver.reset_data_received()

        joy_input = Joy()
        joy_input.axes.append(1.0)

        self.pub_joy.publish(joy_input)

        self.pwm_receiver.wait_for_data_received()

        self.assertEqual(self.pwm_receiver.pwm_out_received, 255)
        self.assertEqual(self.pwm_receiver.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_receiver.pwm_direction_2_received, 0)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_joy_motor_control_integration', __name__, sys.argv)
