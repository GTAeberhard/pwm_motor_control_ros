#include "gpio_pwm_ros_stub.h"
#include "std_msgs/UInt16.h"

RosGpioPwm::RosGpioPwm(const uint8_t pin,
                       ros::NodeHandle& node,
                       const std::string& topic_name)
    : GpioPwmPin(pin)
{
    pin_publisher = node.advertise<std_msgs::UInt16>(topic_name, 10);
}

std_msgs::UInt16 CreateMessage(const uint16_t value)
{
    std_msgs::UInt16 msg;
    msg.data = value;
    return msg;
}

void RosGpioPwm::WriteDutyCycleToPin(const uint16_t value)
{
    pin_publisher.publish(CreateMessage(value));
}

void RosGpioPwm::SetRangeOnDevice(const uint16_t range)
{
    // TODO set ros parameter
}