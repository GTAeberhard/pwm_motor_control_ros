#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16
from pwm_test_support.pwm_receiver_node import PwmReceiverNode

TIMEOUT = 5.0

PKG = 'pwm_motor_control_ros'

node_name = '/pwm_servo_control'
topic_name_motor_speed = '/servo_angle'

topic_name_pwm_out = '/pwm/out'
topic_name_pwm_direction_1 = '/pwm/direction_1'
topic_name_pwm_direction_2 = '/pwm/direction_2'

class TestPwmServoControlNode_Initialization(unittest.TestCase):
    def test_node_starts(self):
        nodes = rosnode.get_node_names()
        self.assertIn(node_name, nodes)

    def test_node_subscribers_publishers(self):
        master = rosgraph.Master('/test')
        try:
            state = master.getSystemState()
        except:
            raise rosnode.ROSNodeIOException('Unable to communicate with master!')
        publishers = sorted([t for t, l in state[0] if node_name in l]) 
        subscribers = sorted([t for t, l in state[1] if node_name in l])

        self.assertIn(topic_name_pwm_out, publishers)
        self.assertIn(topic_name_pwm_direction_1, publishers)
        self.assertIn(topic_name_pwm_direction_2, publishers)

        self.assertIn(topic_name_motor_speed, subscribers)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_servo_control_node', __name__, sys.argv)
