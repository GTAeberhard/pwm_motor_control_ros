import time
import rospy
from std_msgs.msg import UInt8, UInt16

TIMEOUT = 5.0

class PwmReceiverNode():
    pwm_out_received = None
    pwm_direction_1_received = None
    pwm_direction_2_received = None

    topic_name_pwm_out = '/pwm/out'
    topic_name_pwm_direction_1 = '/pwm/direction_1'
    topic_name_pwm_direction_2 = '/pwm/direction_2'

    def __init__(self):
        rospy.init_node('pwm_receiver', anonymous=True)

        rospy.Subscriber(self.topic_name_pwm_out, UInt16, self.callback_pwm)
        rospy.Subscriber(self.topic_name_pwm_direction_1, UInt8, self.callback_direction_1)
        rospy.Subscriber(self.topic_name_pwm_direction_2, UInt8, self.callback_direction_2)

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
