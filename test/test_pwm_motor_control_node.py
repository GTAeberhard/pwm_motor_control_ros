#!/usr/bin/env python
import os, sys, unittest

import rostest
import rospkg

import rosnode
import roslib

PKG = 'pwm_motor_control_ros'

## Test basic functionality of PWM Motor Control Node using
## mock outputs from node instead of real hardware
class TestPwmMotorControlNode(unittest.TestCase):

    def test_node_starts(self):
        nodes = rosnode.get_node_names()
        print('Running nodes:')
        print(nodes)
        self.assertEqual(nodes, ['/pwm_motor_control'])

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_control_node', TestPwmMotorControlNode)