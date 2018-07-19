#!/usr/bin/env python
import os, sys, time, unittest
import rospy, rostest, rosnode, rosgraph
from std_msgs.msg import Int8, UInt8, UInt16
from sensor_msgs.msg import Joy

TIMEOUT = 2.0

PKG = 'pwm_motor_control_ros'

pwm_motor_control_node_name = '/pwm_motor_control'
joy_motor_control_node_name = '/joy_motor_control'

topic_name_pwm_out = '/pwm/out'
topic_name_pwm_direction_1 = '/pwm/direction_1'
topic_name_pwm_direction_2 = '/pwm/direction_2'
topic_name_motor_speed = '/motor_speed'
topic_name_joy = '/joy'

class TestJoyMotorControl_JoyTriggerInputForPositiveMotorSpeed(unittest.TestCase):
    pwm_out_received = None
    pwm_direction_1_received = None
    pwm_direction_2_received = None

    def __init__(self, *args):
        super(TestJoyMotorControl_JoyTriggerInputForPositiveMotorSpeed, self).__init__(*args)
        rospy.init_node('test', anonymous=True)

        rospy.Subscriber(topic_name_pwm_out, UInt16, self.callback_pwm)
        rospy.Subscriber(topic_name_pwm_direction_1, UInt8, self.callback_direction_1)
        rospy.Subscriber(topic_name_pwm_direction_2, UInt8, self.callback_direction_2)
        
        self.pub_joy = rospy.Publisher(topic_name_joy, Joy, queue_size=1)

    def test_1_full_motor_speed(self):
        # Need to wait so that node connections can be established
        time.sleep(0.3)
        self.reset_data_received()

        joy_input = Joy()
        joy_input.axes.append(1.0)

        self.pub_joy.publish(joy_input)

        self.wait_for_data_received()

        self.assertEqual(self.pwm_out_received, 255)
        self.assertEqual(self.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_direction_2_received, 0)

    def wait_for_data_received(self):
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
    rostest.rosrun(PKG, 'test_joy_motor_control', __name__, sys.argv)
