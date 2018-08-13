#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16
from sensor_msgs.msg import Joy

PKG = 'pwm_motor_control_ros'

pwm_motor_control_node_name = '/pwm_motor_control'
joy_motor_control_node_name = '/joy_motor_control'

topic_name_pwm_out = '/pwm/out'
topic_name_pwm_direction_1 = '/pwm/direction_1'
topic_name_pwm_direction_2 = '/pwm/direction_2'
topic_name_motor_speed = '/motor_speed'
topic_name_joy = '/joy'

class TestJoyMotorControl_Initialization(unittest.TestCase):
    def test_node_starts(self):
        nodes = rosnode.get_node_names()
        self.assertIn(pwm_motor_control_node_name, nodes)
        self.assertIn(joy_motor_control_node_name, nodes)

    def test_node_subscribers_publishers(self):
        master = rosgraph.Master('/test')
        try:
            state = master.getSystemState()
        except:
            raise rosnode.ROSNodeIOException('Unable to communicate with master!')
        publishers_pwm = sorted([t for t, l in state[0] if pwm_motor_control_node_name in l]) 
        subscribers_pwm = sorted([t for t, l in state[1] if pwm_motor_control_node_name in l])

        publishers_joy = sorted([t for t, l in state[0] if joy_motor_control_node_name in l]) 
        subscribers_joy = sorted([t for t, l in state[1] if joy_motor_control_node_name in l])

        self.assertIn(topic_name_pwm_out, publishers_pwm)
        self.assertIn(topic_name_pwm_direction_1, publishers_pwm)
        self.assertIn(topic_name_pwm_direction_2, publishers_pwm)
        self.assertIn(topic_name_motor_speed, subscribers_pwm)

        self.assertIn(topic_name_motor_speed, publishers_joy)
        self.assertIn(topic_name_joy, subscribers_joy)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_joy_motor_control_initialization', __name__, sys.argv)
