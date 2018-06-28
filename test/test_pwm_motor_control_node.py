#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import UInt8, UInt16

PKG = 'pwm_motor_control_ros'

## Test basic functionality of PWM Motor Control Node using
## mock outputs from node instead of real hardware
class TestPwmMotorControlNode(unittest.TestCase):
    node_name = '/pwm_motor_control'
    topic_name_pwm_out = '/pwm/out'
    topic_name_pwm_direction_1 = '/pwm/direction_1'
    topic_name_pwm_direction_2 = '/pwm/direction_2'
    topic_name_motor_speed = '/motor_speed'

    def setUp(self):
        self.pwm_out_received = None
        self.pwm_direction_1_received = None
        self.pwm_direction_2_received = None

    def test_node_starts(self):
        nodes = rosnode.get_node_names()
        print('Running nodes:')
        print(nodes)
        self.assertEqual(nodes, [ self.node_name ])

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

    def test_when_no_input_received(self):
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber(self.topic_name_pwm_out, UInt16, self.callback_pwm)
        rospy.Subscriber(self.topic_name_pwm_direction_1, UInt8, self.callback_direction_1)
        rospy.Subscriber(self.topic_name_pwm_direction_2, UInt8, self.callback_direction_2)

        timeout = time.time() + 5.0
        while not rospy.is_shutdown() and \
              self.pwm_out_received == None and \
              self.pwm_direction_1_received == None and \
              self.pwm_direction_2_received == None and \
              time.time() < timeout:
            time.sleep(0.1)

        self.assertEqual(self.pwm_out_received, 0)
        self.assertEqual(self.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_direction_2_received, 0)

    def callback_pwm(self, data):
        self.pwm_out_received = data.data

    def callback_direction_1(self, data):
        self.pwm_direction_1_received = data.data

    def callback_direction_2(self, data):
        self.pwm_direction_2_received = data.data


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_control_node', TestPwmMotorControlNode)