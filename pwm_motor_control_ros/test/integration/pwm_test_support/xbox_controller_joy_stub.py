from sensor_msgs.msg import Joy

class XboxControllerJoyStub:
    @staticmethod
    def right_trigger_depressed():
        joy_msg = XboxControllerJoyStub.idle_controller()
        joy_msg.axes[5] = -1.0
        return joy_msg

    @staticmethod
    def right_trigger_half_depressed():
        joy_msg = XboxControllerJoyStub.idle_controller()
        joy_msg.axes[5] = 0.0
        return joy_msg

    @staticmethod
    def left_trigger_depressed():
        joy_msg = XboxControllerJoyStub.idle_controller()
        joy_msg.axes[2] = -1.0
        return joy_msg

    @staticmethod
    def left_trigger_half_depressed():
        joy_msg = XboxControllerJoyStub.idle_controller()
        joy_msg.axes[2] = 0.0
        return joy_msg

    @staticmethod
    def idle_controller():
        joy_msg = Joy()
        joy_msg.axes = [ 0.0 for i in range(8)]
        joy_msg.buttons = [ 0 for i in range(15)]
        joy_msg.axes[2] = 1.0
        joy_msg.axes[5] = 1.0
        return joy_msg
    