#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16
from sensor_msgs.msg import Joy

PKG = 'pwm_motor_control_ros'

class TestJoyMotorControl_Initialization(unittest.TestCase):
    pwm_motor_control_node_name = '/pwm_motor_control'
    joy_motor_control_node_name = '/joy_motor_control'
    topic_name_pwm_out = '/pwm/out'
    topic_name_pwm_direction_1 = '/pwm/direction_1'
    topic_name_pwm_direction_2 = '/pwm/direction_2'
    topic_name_motor_speed = '/motor_speed'
    topic_name_joy = '/joy'

    def setUp(self):
        self.pwm_out_received = None
        self.pwm_direction_1_received = None
        self.pwm_direction_2_received = None

    def test_node_starts(self):
        nodes = rosnode.get_node_names()
        self.assertIn(self.pwm_motor_control_node_name, nodes)
        self.assertIn(self.joy_motor_control_node_name, nodes)

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_control_node', __name__, sys.argv)
