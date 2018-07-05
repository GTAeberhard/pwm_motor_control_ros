#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16

PKG = 'pwm_motor_control_ros'
TIMEOUT = 2.0

# Test basic functionality of PWM Motor Control Node using
# mock outputs from node instead of real hardware
class TestPwmMotorControlNode_Initialization(unittest.TestCase):
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
        self.assertIn(self.node_name, nodes)

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


class TestPwmMotorControlNode_MotorSpeeds(unittest.TestCase):
    node_name = '/pwm_motor_control'
    topic_name_pwm_out = '/pwm/out'
    topic_name_pwm_direction_1 = '/pwm/direction_1'
    topic_name_pwm_direction_2 = '/pwm/direction_2'
    topic_name_motor_speed = '/motor_speed'
    pwm_out_received = None
    pwm_direction_1_received = None
    pwm_direction_2_received = None

    def __init__(self, *args):
        super(TestPwmMotorControlNode_MotorSpeeds, self).__init__(*args)
        rospy.init_node('test', anonymous=True)

        rospy.Subscriber(self.topic_name_pwm_out, UInt16, self.callback_pwm)
        rospy.Subscriber(self.topic_name_pwm_direction_1, UInt8, self.callback_direction_1)
        rospy.Subscriber(self.topic_name_pwm_direction_2, UInt8, self.callback_direction_2)
        
        self.pub_motor_speed = rospy.Publisher(self.topic_name_motor_speed, Int8, queue_size=10)

    def test_1_when_no_input_received(self):
        self.wait_for_data_received()

        self.assertEqual(self.pwm_out_received, 0)
        self.assertEqual(self.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_direction_2_received, 0)

    def test_2_1_input_50_percent(self):
        self.reset_data_received()

        # Need to wait so that node connections can be established
        time.sleep(0.3)

        self.pub_motor_speed.publish(Int8(50))

        self.wait_for_data_received()

        self.assertEqual(self.pwm_out_received, 128)
        self.assertEqual(self.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_direction_2_received, 0)

    def test_2_2_input_50_percent(self):
        self.reset_data_received()

        # Need to wait so that node connections can be established
        time.sleep(0.3)

        self.pub_motor_speed.publish(Int8(-50))

        self.wait_for_data_received()

        self.assertEqual(self.pwm_out_received, 128)
        self.assertEqual(self.pwm_direction_1_received, 0)
        self.assertEqual(self.pwm_direction_2_received, 255)

    def test_3_reset_motor_speed_after_publish_silence(self):
        self.reset_data_received()
        # Need to wait so that node connections can be established
        time.sleep(0.3)

        self.pub_motor_speed.publish(Int8(50))

        self.wait_for_data_received()

        self.assertEqual(self.pwm_out_received, 128)
        self.assertEqual(self.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_direction_2_received, 0)

        time.sleep(1.1)

        self.wait_for_data_received()

        self.assertEqual(self.pwm_out_received, 0)
        self.assertEqual(self.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_direction_2_received, 0)

    def wait_for_data_received(self):
        self.reset_data_received()

        timeout = time.time() + TIMEOUT
        while not rospy.is_shutdown() and \
              self.pwm_out_received == None and \
              self.pwm_direction_1_received == None and \
              self.pwm_direction_2_received == None and \
              time.time() < timeout:
            time.sleep(0.1)

    def reset_data_received(self):
        self.pwm_out_received = None
        self.pwm_direction_1_received = None
        self.pwm_direction_2_received = None

    def callback_pwm(self, data):
        self.pwm_out_received = data.data

    def callback_direction_1(self, data):
        self.pwm_direction_1_received = data.data

    def callback_direction_2(self, data):
        self.pwm_direction_2_received = data.data


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_pwm_control_node', __name__, sys.argv)
