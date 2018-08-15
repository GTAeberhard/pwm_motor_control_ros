import unittest
import time
from pwm_test_support.pwm_receiver_node import PwmReceiverNode

class BasePwmTestFixture(unittest.TestCase):
    pwm_receiver = None

    def __init__(self, *args):
        super(BasePwmTestFixture, self).__init__(*args)
        self.pwm_receiver = PwmReceiverNode()

    def setUp(self):
        # Need to wait so that node connections can be established
        time.sleep(1.0)
        self.pwm_receiver.reset_data_received()

    def wait_and_assert_speed(self, speed_value):
        self.pwm_receiver.wait_for_data_received()
        self.assertEqual(self.pwm_receiver.pwm_out_received, speed_value)

    def assert_positive_motor_direction(self):
        self.assertEqual(self.pwm_receiver.pwm_direction_1_received, 255)
        self.assertEqual(self.pwm_receiver.pwm_direction_2_received, 0)

    def assert_negative_motor_direction(self):
        self.assertEqual(self.pwm_receiver.pwm_direction_1_received, 0)
        self.assertEqual(self.pwm_receiver.pwm_direction_2_received, 255)