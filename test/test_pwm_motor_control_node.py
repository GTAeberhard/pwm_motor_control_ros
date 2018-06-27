#!/usr/bin/env python
import os, sys, unittest

import rospy
import rostest
import rosnode
import rosgraph

PKG = 'pwm_motor_control_ros'

## Test basic functionality of PWM Motor Control Node using
## mock outputs from node instead of real hardware
class TestPwmMotorControlNode(unittest.TestCase):
    node_name = '/pwm_motor_control'
    topic_name_pwm_out = '/pwm/out'
    topic_name_pwm_direction_1 = '/pwm/direction_1'
    topic_name_pwm_direction_2 = '/pwm/direction_2'
    topic_name_motor_speed = '/motor_speed'

    def test_node_starts(self):
        nodes = rosnode.get_node_names()
        print('Running nodes:')
        print(nodes)
        self.assertEqual(nodes, [self.node_name])

    def test_node_subscribers_publishers(self):
        master = rosgraph.Master('/test')
        try:
            state = master.getSystemState()
        except:
            raise rosnode.ROSNodeIOException('Unable to communicate with master!')
        publishers = sorted([t for t, l in state[0] if self.node_name in l]) 
        subscribers = sorted([t for t, l in state[1] if self.node_name in l])

        self.assertIn(self.topic_name_pwm_out, publishers)
        self.assertIn(self.topic_name_pwm_direction_1, publishers)
        self.assertIn(self.topic_name_pwm_direction_2, publishers)

        self.assertIn(self.topic_name_motor_speed, subscribers)


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_control_node', TestPwmMotorControlNode)